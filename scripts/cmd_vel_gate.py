#!/usr/bin/env python3
"""
cmd_vel_gate.py -- the only path from move_base to twist_mux.

Wraps move_base's Twist as TwistStamped (twist_mux expects TwistStamped, the
job twist_stamper.py used to do) and forwards it ONLY while all of these hold:
  - nav_supervisor.py's health heartbeat is fresh and True
    (every navigation input publishing at rate, fresh, finite; core alive)
  - contact_probe_manager's motion_allowed heartbeat is fresh and True
    (alive, calibrated, not HALTED, its own inputs fresh)
  - /odom is fresh (needed for the stuck check below)
  - no latched fault from this node
Every command is also checked for finite values and clamped to max_vel_*.

When the gate closes it publishes ONE zero twist and then nothing, so
twist_mux drops the navigation input and lower-priority sources (or none)
take over. If this node dies, nothing is published either: twist_mux times
the input out (0.5 s). If move_base stops publishing while the last command
was non-zero, a zero is sent after cmd_timeout instead of waiting for that.

Stuck detection: move_base commanding > stuck_cmd_speed while the robot moves
< stuck_actual_speed for stuck_time, outside CONTACT_PROBE/CLASSIFYING (where
pushing against an object is expected), latches a fault: the gate closes and
/haptiquad_nav/fault tells contact_probe_manager to HALT. A new user goal
clears the latch (contact_probe_manager still decides whether to resume).

  In:  cmd_vel_in (Twist), /haptiquad_nav/health (Bool),
       /haptiquad_nav/motion_allowed (Bool), /odom (Odometry),
       /haptiquad_nav/state (String), /move_base_simple/goal (PoseStamped)
  Out: cmd_vel_out (TwistStamped), /haptiquad_nav/fault (String),
       /haptiquad_nav/gate_open (Bool, latched, on change)
"""
import functools
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

PUSHING_STATES = ('CONTACT_PROBE', 'CLASSIFYING')


def _synchronized(fn):
    """Command, timer and goal callbacks run in separate rospy threads."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return fn(self, *args, **kwargs)
    return wrapper


class Heartbeat:
    """Latest Bool value and when it arrived."""

    def __init__(self, topic, timeout):
        self.timeout = timeout
        self.value = False
        self.stamp = None
        rospy.Subscriber(topic, Bool, self._cb, queue_size=1)

    def _cb(self, msg):
        self.value = msg.data
        self.stamp = rospy.Time.now()

    def ok(self, now):
        return (self.stamp is not None and self.value
                and (now - self.stamp).to_sec() <= self.timeout)


class CmdVelGate:

    def __init__(self):
        self._frame = rospy.get_param('~frame_id', 'base')
        self._max_vx = rospy.get_param('~max_vel_x', 0.6)
        self._max_vy = rospy.get_param('~max_vel_y', 0.6)
        self._max_wz = rospy.get_param('~max_vel_theta', 0.5)
        hb_timeout = rospy.get_param('~heartbeat_timeout', 0.3)
        self._odom_timeout = rospy.get_param('~odom_timeout', 0.3)
        self._cmd_timeout = rospy.get_param('~cmd_timeout', 0.3)
        self._stuck_time = rospy.get_param('~stuck_time', 4.0)
        self._stuck_cmd = rospy.get_param('~stuck_cmd_speed', 0.08)
        self._stuck_actual = rospy.get_param('~stuck_actual_speed', 0.02)

        self._health = Heartbeat('/haptiquad_nav/health', hb_timeout)
        self._allowed = Heartbeat('/haptiquad_nav/motion_allowed', hb_timeout)
        self._odom_t = None
        self._actual_speed = 0.0
        self._state = ''
        self._fault = ''
        self._last_cmd = None        # last Twist received
        self._last_cmd_t = None
        self._sent_nonzero = False   # last thing published was a non-zero twist
        self._open = None
        self._stuck_since = None
        self._lock = threading.RLock()

        self._pub = rospy.Publisher('cmd_vel_out', TwistStamped, queue_size=1)
        self._fault_pub = rospy.Publisher('/haptiquad_nav/fault', String, queue_size=10)
        self._open_pub = rospy.Publisher('/haptiquad_nav/gate_open', Bool, queue_size=1, latch=True)

        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber('/haptiquad_nav/state', String, self._state_cb, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_cb, queue_size=1)
        rospy.Subscriber('cmd_vel_in', Twist, self._cmd_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self._tick)
        rospy.on_shutdown(self._stop)   # don't leave twist_mux holding a moving command
        self._set_open(False, 'starting up')

    # ── inputs ────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        v = msg.twist.twist.linear
        self._actual_speed = math.hypot(v.x, v.y)
        self._odom_t = rospy.Time.now()

    def _state_cb(self, msg):
        self._state = msg.data

    @_synchronized
    def _goal_cb(self, _msg):
        if self._fault:
            rospy.loginfo('[CmdVelGate] New goal -- clearing fault latch (%s).', self._fault)
            self._fault = ''

    @_synchronized
    def _cmd_cb(self, msg):
        self._last_cmd = msg
        self._last_cmd_t = rospy.Time.now()
        if not self._is_open(self._last_cmd_t):
            return
        vals = (msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in vals):
            self._raise_fault('non-finite velocity command from move_base')
            return
        out = Twist()
        out.linear.x = max(-self._max_vx, min(self._max_vx, msg.linear.x))
        out.linear.y = max(-self._max_vy, min(self._max_vy, msg.linear.y))
        out.angular.z = max(-self._max_wz, min(self._max_wz, msg.angular.z))
        self._publish(out)

    # ── gate logic ────────────────────────────────────────────────────────────

    def _why_closed(self, now):
        if self._fault:
            return 'fault: %s' % self._fault
        if not self._health.ok(now):
            return 'nav_supervisor health heartbeat missing/stale/False'
        if not self._allowed.ok(now):
            return 'contact_probe_manager motion_allowed missing/stale/False'
        if self._odom_t is None or (now - self._odom_t).to_sec() > self._odom_timeout:
            return '/odom missing or stale'
        return ''

    def _is_open(self, now):
        why = self._why_closed(now)
        self._set_open(not why, why)
        return not why

    def _set_open(self, is_open, why):
        if is_open == self._open:
            return
        self._open = is_open
        self._open_pub.publish(Bool(data=is_open))
        if is_open:
            rospy.loginfo('[CmdVelGate] Gate OPEN.')
        else:
            rospy.logwarn('[CmdVelGate] Gate CLOSED: %s', why)
            self._stop()

    def _stop(self):
        """One zero twist so the robot stops now, then silence."""
        if self._sent_nonzero:
            self._publish(Twist())

    def _publish(self, twist):
        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self._frame
        out.twist = twist
        self._pub.publish(out)
        self._sent_nonzero = any(abs(v) > 1e-6 for v in (
            twist.linear.x, twist.linear.y, twist.angular.z))

    @_synchronized
    def _raise_fault(self, reason):
        if self._fault:
            return
        self._fault = reason
        rospy.logerr('[CmdVelGate] FAULT: %s', reason)
        self._fault_pub.publish(String(data=reason))
        self._is_open(rospy.Time.now())   # closes and sends the zero twist

    @_synchronized
    def _tick(self, _event):
        now = rospy.Time.now()
        if not self._is_open(now):
            return
        # move_base went quiet after a non-zero command: stop now rather than
        # waiting for twist_mux's own timeout.
        if (self._sent_nonzero and self._last_cmd_t is not None
                and (now - self._last_cmd_t).to_sec() > self._cmd_timeout):
            rospy.logwarn('[CmdVelGate] No command from move_base for %.2fs -- sending stop.',
                          (now - self._last_cmd_t).to_sec())
            self._publish(Twist())
        self._check_stuck(now)

    def _check_stuck(self, now):
        cmd = self._last_cmd
        commanding = (cmd is not None and self._last_cmd_t is not None
                      and (now - self._last_cmd_t).to_sec() <= self._cmd_timeout
                      and math.hypot(cmd.linear.x, cmd.linear.y) > self._stuck_cmd)
        if not commanding or self._state in PUSHING_STATES or self._actual_speed >= self._stuck_actual:
            self._stuck_since = None
            return
        if self._stuck_since is None:
            self._stuck_since = now
        elif (now - self._stuck_since).to_sec() >= self._stuck_time:
            self._raise_fault('controller stuck: commanding %.2f m/s, moving %.3f m/s for %.1fs (%s)'
                              % (math.hypot(cmd.linear.x, cmd.linear.y), self._actual_speed,
                                 self._stuck_time, self._state or 'unknown state'))


if __name__ == '__main__':
    rospy.init_node('cmd_vel_gate')
    CmdVelGate()
    rospy.spin()
