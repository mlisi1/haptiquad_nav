#!/usr/bin/env python3
"""
nav_supervisor.py -- starts the navigation core only once its inputs are
healthy, then keeps checking them.

Preflight: every entry of ~inputs (topics) and ~transforms (TF) must pass
for ~preflight_stable_s without interruption, and ~wait_for_true_param (if
set) must be True -- navigation.launch points it at the flag
anymal_motion_init sets once walk mode is requested.
Nothing that can move the robot is running before that -- move_base,
contact_probe_manager and cmd_vel_gate all live in the core launch file
(~core_launch), which this node starts as a child roslaunch.

Runtime: the same checks keep running, plus ~runtime_inputs (heartbeats of
core nodes, after ~core_startup_grace_s). /haptiquad_nav/health (Bool, 10Hz)
is True only while all of them pass; cmd_vel_gate.py forwards no velocity
command and contact_probe_manager HALTs unless it is. The core nodes are
required="true", so one crashing takes the whole core launch down; this
node then reports health False for good (relaunch needed). If this node
dies, the heartbeat stops, which the gate and contact_probe_manager treat
the same as False.

Per-topic checks (all optional except the topic itself):
  min_rate   [Hz]  measured over the last 2s (also implies a silence timeout)
  max_age    [s]   header.stamp age; also rejects stamps > max_future_s ahead
                   (clock offset between robot PCs)
  finite     bool  every float field (incl. nested/arrays) finite
  nonempty   bool  PointCloud2 with at least one point
Status: /haptiquad_nav/supervisor_status (String, latched, on change) and
/diagnostics.
"""
import collections
import math
import os
import shlex
import signal
import subprocess
import threading

import rospy
import rostopic
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, String

RATE_WINDOW_S = 2.0
CONTENT_CHECK_PERIOD_S = 0.05   # at most 20 content checks/s per topic


def _all_finite(msg, depth=0):
    if depth > 8:
        return True
    if isinstance(msg, float):
        return math.isfinite(msg)
    if isinstance(msg, (list, tuple)):
        return all(_all_finite(v, depth + 1) for v in msg)
    slots = getattr(msg, '__slots__', None)
    if slots:
        return all(_all_finite(getattr(msg, a), depth + 1) for a in slots)
    return True   # ints, strings, bytes, time


class TopicCheck:

    def __init__(self, cfg, max_future):
        self.topic = cfg['topic']
        self.min_rate = cfg.get('min_rate', 0.0)
        self.max_age = cfg.get('max_age')
        self.finite = cfg.get('finite', False)
        self.nonempty = cfg.get('nonempty', False)
        self.max_future = max_future
        self.times = collections.deque()
        self.last_content_check = None
        self.content_problem = ''
        self.content_problem_t = None
        self.sub = None
        self.lock = threading.Lock()

    def try_subscribe(self):
        if self.sub is not None:
            return True
        cls, real_topic, _ = rostopic.get_topic_class(self.topic, blocking=False)
        if cls is None:
            return False
        self.sub = rospy.Subscriber(real_topic, cls, self._cb, queue_size=10)
        return True

    def _cb(self, msg):
        now = rospy.Time.now()
        with self.lock:
            self.times.append(now.to_sec())
            if (self.last_content_check is not None
                    and (now - self.last_content_check).to_sec() < CONTENT_CHECK_PERIOD_S):
                return
            self.last_content_check = now
        problem = self._content_problem(msg, now)
        with self.lock:
            if problem:
                self.content_problem, self.content_problem_t = problem, now
            elif self.content_problem and (now - self.content_problem_t).to_sec() > 1.0:
                self.content_problem = ''   # bad content clears after 1s of good messages

    def _content_problem(self, msg, now):
        header = getattr(msg, 'header', None)
        if self.max_age is not None and header is not None and not header.stamp.is_zero():
            age = (now - header.stamp).to_sec()
            if age > self.max_age:
                return 'stamp %.2fs old (> %.2fs)' % (age, self.max_age)
            if age < -self.max_future:
                return 'stamp %.2fs in the future (clock offset?)' % -age
        if self.nonempty and hasattr(msg, 'width') and msg.width * msg.height == 0:
            return 'empty point cloud'
        if self.finite and not _all_finite(msg):
            return 'non-finite values'
        return ''

    def problem(self, now):
        if self.sub is None:
            return 'not advertised'
        t = now.to_sec()
        with self.lock:
            while self.times and t - self.times[0] > RATE_WINDOW_S:
                self.times.popleft()
            times = list(self.times)
            content = self.content_problem
        if not times:
            return 'no messages for %.0fs' % RATE_WINDOW_S
        if self.min_rate > 0:
            silence = t - times[-1]
            if silence > max(2.0 / self.min_rate, 0.2):
                return 'silent for %.2fs' % silence
            if len(times) >= 2:
                rate = (len(times) - 1) / max(times[-1] - times[0], 1e-6)
                if rate < self.min_rate:
                    return 'rate %.1fHz < %.1fHz' % (rate, self.min_rate)
            elif self.min_rate * RATE_WINDOW_S >= 2:
                return 'only %d message(s) in %.0fs' % (len(times), RATE_WINDOW_S)
        return content


class NavSupervisor:

    def __init__(self):
        self._stable_s = rospy.get_param('~preflight_stable_s', 3.0)
        self._grace_s = rospy.get_param('~core_startup_grace_s', 30.0)
        self._report_s = rospy.get_param('~report_period_s', 5.0)
        max_future = rospy.get_param('~max_future_s', 0.2)
        self._inputs = [TopicCheck(c, max_future) for c in rospy.get_param('~inputs', [])]
        self._runtime = [TopicCheck(c, max_future) for c in rospy.get_param('~runtime_inputs', [])]
        self._tfs = rospy.get_param('~transforms', [])   # [target, source, max_age]
        self._wait_param = rospy.get_param('~wait_for_true_param', '')
        self._core_pkg = rospy.get_param('~core_launch/package', 'haptiquad_nav')
        self._core_file = rospy.get_param('~core_launch/file', 'navigation_core.launch')
        # roslaunch ignores "name:=" with an empty value; drop those so the
        # core's own default ('') applies instead of a missing-arg error.
        self._core_args = [a for a in shlex.split(rospy.get_param('~core_args', ''))
                           if not a.endswith(':=')]

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf)
        self._health_pub = rospy.Publisher('/haptiquad_nav/health', Bool, queue_size=1)
        self._status_pub = rospy.Publisher('/haptiquad_nav/supervisor_status', String,
                                           queue_size=1, latch=True)
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        self._core = None
        self._core_start = None
        self._fatal = ''
        self._last_status = None
        self._last_report = rospy.Time(0)
        rospy.on_shutdown(self._stop_core)

    # ── checks ────────────────────────────────────────────────────────────────

    def _tf_problem(self, now, target, source, max_age):
        try:
            tr = self._tf_buf.lookup_transform(target, source, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            return 'unavailable (%s)' % str(e).splitlines()[0]
        if tr.header.stamp.is_zero():
            return ''   # static
        age = (now - tr.header.stamp).to_sec()
        return 'transform %.2fs old (> %.2fs)' % (age, max_age) if age > max_age else ''

    def _problems(self, now, include_runtime):
        out = []
        for c in self._inputs:
            c.try_subscribe()
            p = c.problem(now)
            if p:
                out.append('%s: %s' % (c.topic, p))
        for target, source, max_age in self._tfs:
            p = self._tf_problem(now, target, source, max_age)
            if p:
                out.append('tf %s->%s: %s' % (target, source, p))
        if include_runtime:
            for c in self._runtime:
                c.try_subscribe()
                p = c.problem(now)
                if p:
                    out.append('%s: %s' % (c.topic, p))
        return out

    def _waiting_for_param(self):
        return bool(self._wait_param) and rospy.get_param(self._wait_param, False) is not True

    # ── core launch ───────────────────────────────────────────────────────────

    def _start_core(self):
        cmd = ['roslaunch', self._core_pkg, self._core_file] + self._core_args
        rospy.loginfo('[NavSupervisor] Preflight passed -- starting: %s', ' '.join(cmd))
        self._core = subprocess.Popen(cmd, start_new_session=True)
        self._core_start = rospy.Time.now()

    def _stop_core(self):
        if self._core is not None and self._core.poll() is None:
            os.killpg(self._core.pid, signal.SIGINT)
            try:
                self._core.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self._core.pid, signal.SIGKILL)

    # ── reporting ─────────────────────────────────────────────────────────────

    def _report(self, now, healthy, phase, problems):
        status = '%s: %s' % (phase, 'OK' if healthy else '; '.join(problems))
        if status != self._last_status:
            self._status_pub.publish(String(data=status))
            log = rospy.loginfo if healthy else rospy.logwarn
            log('[NavSupervisor] %s', status)
            self._last_status = status
        elif not healthy and (now - self._last_report).to_sec() >= self._report_s:
            rospy.logwarn('[NavSupervisor] %s', status)
        else:
            return
        self._last_report = now
        st = DiagnosticStatus(name='nav_supervisor', message=phase,
                              level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR)
        st.values = [KeyValue(key='problem_%d' % i, value=p) for i, p in enumerate(problems)]
        self._diag_pub.publish(DiagnosticArray(header=rospy.Header(stamp=now), status=[st]))

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        ok_since = None
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self._core is None:
                problems = self._problems(now, include_runtime=False)
                if self._waiting_for_param():
                    problems.append('waiting for %s (walk mode)' % self._wait_param)
                if problems:
                    ok_since = None
                elif ok_since is None:
                    ok_since = now
                stable = ok_since is not None and (now - ok_since).to_sec() >= self._stable_s
                self._report(now, False, 'PREFLIGHT', problems or
                             ['all checks passing, confirming for %.1fs' % self._stable_s])
                if stable:
                    self._start_core()
                healthy = False
            else:
                if not self._fatal and self._core.poll() is not None:
                    self._fatal = 'navigation core exited (code %s) -- a core node died; ' \
                                  'relaunch required' % self._core.returncode
                    rospy.logfatal('[NavSupervisor] %s', self._fatal)
                in_grace = (now - self._core_start).to_sec() < self._grace_s
                problems = [self._fatal] if self._fatal else self._problems(now, not in_grace)
                healthy = not problems
                self._report(now, healthy, 'RUNNING', problems)
            self._health_pub.publish(Bool(data=healthy))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('nav_supervisor')
    NavSupervisor().run()
