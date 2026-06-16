#!/usr/bin/env python3
"""
Contact-probe state machine for HaptiQuad tactile navigation.

States:
  NORMAL_NAV      — full speed, obstacle distance > slow_approach_distance
  SLOW_APPROACH   — slow_approach_speed, obstacle distance ≤ slow_approach_distance
  CONTACT_PROBE   — probe_speed, wrench monitored for contact
  CLASSIFYING     — short forward push to decide movable vs solid
"""
import math
import struct
import collections

import rospy
import tf.transformations
import actionlib

from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal


# ── State identifiers ─────────────────────────────────────────────────────────
NORMAL_NAV    = 'NORMAL_NAV'
SLOW_APPROACH = 'SLOW_APPROACH'
CONTACT_PROBE = 'CONTACT_PROBE'
CLASSIFYING   = 'CLASSIFYING'


class ContactProbeManager:

    def __init__(self):
        # ── Parameters ────────────────────────────────────────────────────────
        self._thr_fx  = rospy.get_param('wrench_threshold/fx', 15.0)
        self._thr_fy  = rospy.get_param('wrench_threshold/fy', 10.0)
        self._thr_fz  = rospy.get_param('wrench_threshold/fz', 30.0)
        self._thr_tx  = rospy.get_param('wrench_threshold/tx', 3.0)
        self._thr_ty  = rospy.get_param('wrench_threshold/ty', 3.0)
        self._thr_tz  = rospy.get_param('wrench_threshold/tz', 2.0)

        self._filter_win      = rospy.get_param('wrench_filter_window', 10)
        self._normal_frac     = rospy.get_param('normal_speed_fraction', 1.0)
        self._slow_speed      = rospy.get_param('slow_approach_speed', 0.10)
        self._probe_speed     = rospy.get_param('probe_speed', 0.03)
        self._slow_dist       = rospy.get_param('slow_approach_distance', 0.8)
        self._probe_dist      = rospy.get_param('probe_distance', 0.25)
        self._probe_timeout   = rospy.get_param('probe_timeout', 8.0)
        self._hold_time       = rospy.get_param('contact_hold_time', 0.5)
        self._mark_radius     = rospy.get_param('mark_radius', 0.2)
        self._classify_time   = rospy.get_param('classify_time', 2.5)

        # ── State ─────────────────────────────────────────────────────────────
        self._state = NORMAL_NAV
        self._state_enter_time = rospy.Time.now()

        # ── Wrench filtering ─────────────────────────────────────────────────
        self._wrench_buf = {
            'fx': collections.deque(maxlen=self._filter_win),
            'fy': collections.deque(maxlen=self._filter_win),
            'fz': collections.deque(maxlen=self._filter_win),
            'tx': collections.deque(maxlen=self._filter_win),
            'ty': collections.deque(maxlen=self._filter_win),
            'tz': collections.deque(maxlen=self._filter_win),
        }
        self._wrench_above_since = None  # time when wrench first exceeded threshold

        # ── Obstacle distance ─────────────────────────────────────────────────
        self._nearest_obs = float('inf')

        # ── Robot pose (map frame) ────────────────────────────────────────────
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0

        # ── Classify bookkeeping ──────────────────────────────────────────────
        self._classify_start_pos  = None   # (x, y) when entering CLASSIFYING
        self._classify_start_wrench_mag = None
        self._contact_point = None          # (x, y, z) in map frame

        # ── Confirmed solid obstacle list ─────────────────────────────────────
        self._solid_obstacles = []  # list of (x, y, z)

        # ── Last move_base goal ───────────────────────────────────────────────
        self._last_goal = None

        # ── Latest cmd_vel from move_base ─────────────────────────────────────
        self._latest_cmd = Twist()

        # ── Publishers ────────────────────────────────────────────────────────
        self._twist_pub = rospy.Publisher(
            'cmd_vel_out', TwistStamped, queue_size=1)
        self._obstacles_pub = rospy.Publisher(
            '/haptiquad_nav/solid_obstacles', PointCloud2, queue_size=1, latch=True)

        # ── move_base action client (for goal management) ─────────────────────
        self._mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('cmd_vel_in',  Twist,                     self._cmd_cb,   queue_size=1)
        rospy.Subscriber('base_wrench', WrenchStamped,             self._wrench_cb, queue_size=1)
        rospy.Subscriber('/scan_obstacles', LaserScan,             self._scan_cb,  queue_size=1)
        rospy.Subscriber('/slam/pose',  PoseWithCovarianceStamped, self._pose_cb,  queue_size=1)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal,     self._goal_cb,  queue_size=1)

        # ── Main loop ─────────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.1), self._update)  # 10 Hz

        rospy.loginfo('[ContactProbe] Started in state %s', self._state)

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _cmd_cb(self, msg):
        self._latest_cmd = msg

    def _wrench_cb(self, msg):
        w = msg.wrench
        self._wrench_buf['fx'].append(w.force.x)
        self._wrench_buf['fy'].append(w.force.y)
        self._wrench_buf['fz'].append(w.force.z)
        self._wrench_buf['tx'].append(w.torque.x)
        self._wrench_buf['ty'].append(w.torque.y)
        self._wrench_buf['tz'].append(w.torque.z)

    def _scan_cb(self, msg):
        sector = math.pi / 4  # ±45° around forward direction
        angles = [msg.angle_min + i * msg.angle_increment
                  for i in range(len(msg.ranges))]
        forward = [r for r, a in zip(msg.ranges, angles)
                   if abs(a) <= sector
                   and msg.range_min < r < msg.range_max
                   and math.isfinite(r)]
        self._nearest_obs = min(forward) if forward else float('inf')

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._robot_x = p.position.x
        self._robot_y = p.position.y
        q = p.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self._robot_yaw = yaw

    def _goal_cb(self, msg):
        self._last_goal = msg.goal  # MoveBaseGoal, what send_goal() expects

    # ── Wrench helpers ────────────────────────────────────────────────────────

    def _avg(self, key):
        buf = self._wrench_buf[key]
        return sum(buf) / len(buf) if buf else 0.0

    def _wrench_above_threshold(self):
        return (abs(self._avg('fx')) > self._thr_fx or
                abs(self._avg('fy')) > self._thr_fy or
                abs(self._avg('fz')) > self._thr_fz or
                abs(self._avg('tx')) > self._thr_tx or
                abs(self._avg('ty')) > self._thr_ty or
                abs(self._avg('tz')) > self._thr_tz)

    def _wrench_magnitude(self):
        return math.sqrt(self._avg('fx')**2 + self._avg('fy')**2 +
                         self._avg('fz')**2)

    # ── Velocity output ───────────────────────────────────────────────────────

    def _publish_cmd(self, linear_x, angular_z=None):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base'
        msg.twist.linear.x  = linear_x
        msg.twist.linear.y  = self._latest_cmd.linear.y
        msg.twist.angular.z = (angular_z if angular_z is not None
                               else self._latest_cmd.angular.z)
        self._twist_pub.publish(msg)

    def _clamp_and_publish(self, max_speed):
        cmd_vx = self._latest_cmd.linear.x
        clamped = math.copysign(min(abs(cmd_vx), max_speed), cmd_vx) \
                  if cmd_vx != 0.0 else 0.0
        self._publish_cmd(clamped)

    # ── Contact point estimation ──────────────────────────────────────────────

    def _compute_contact_point(self):
        cx = self._robot_x + self._probe_dist * math.cos(self._robot_yaw)
        cy = self._robot_y + self._probe_dist * math.sin(self._robot_yaw)
        return (cx, cy, 0.0)

    # ── Solid obstacle cloud ──────────────────────────────────────────────────

    def _publish_obstacles(self):
        if not self._solid_obstacles:
            return

        pts = []
        for ox, oy, oz in self._solid_obstacles:
            # Sample a ring of points at mark_radius to fill the costmap cell area
            pts.append((ox, oy, oz))
            n = 8
            for i in range(n):
                a = 2 * math.pi * i / n
                pts.append((ox + self._mark_radius * math.cos(a),
                             oy + self._mark_radius * math.sin(a),
                             oz))

        cloud = PointCloud2()
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width  = len(pts)
        cloud.fields = [
            PointField('x', 0,  PointField.FLOAT32, 1),
            PointField('y', 4,  PointField.FLOAT32, 1),
            PointField('z', 8,  PointField.FLOAT32, 1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step   = 12 * len(pts)
        cloud.is_dense   = True
        cloud.data       = b''.join(struct.pack('fff', *p) for p in pts)
        self._obstacles_pub.publish(cloud)

    # ── Goal management ───────────────────────────────────────────────────────

    def _cancel_and_replan(self):
        rospy.loginfo('[ContactProbe] Cancelling move_base goal and replanning.')
        self._mb_client.cancel_all_goals()
        rospy.sleep(1.5)  # wait for costmap update to propagate
        if self._last_goal is not None:
            self._mb_client.send_goal(self._last_goal)
            rospy.loginfo('[ContactProbe] Goal re-sent to move_base.')

    # ── State transitions ─────────────────────────────────────────────────────

    def _enter(self, new_state):
        rospy.loginfo('[ContactProbe] %s → %s  (obs=%.2fm)',
                      self._state, new_state, self._nearest_obs)
        self._state = new_state
        self._state_enter_time = rospy.Time.now()

    def _elapsed(self):
        return (rospy.Time.now() - self._state_enter_time).to_sec()

    # ── Main update loop ──────────────────────────────────────────────────────

    def _update(self, _event):
        state = self._state

        if state == NORMAL_NAV:
            self._clamp_and_publish(float('inf'))
            if self._nearest_obs <= self._slow_dist:
                self._enter(SLOW_APPROACH)

        elif state == SLOW_APPROACH:
            self._clamp_and_publish(self._slow_speed)
            if self._nearest_obs > self._slow_dist:
                self._enter(NORMAL_NAV)
            elif self._nearest_obs <= self._probe_dist:
                self._wrench_above_since = None
                self._enter(CONTACT_PROBE)

        elif state == CONTACT_PROBE:
            self._clamp_and_publish(self._probe_speed)

            if self._wrench_above_threshold():
                if self._wrench_above_since is None:
                    self._wrench_above_since = rospy.Time.now()
                    rospy.loginfo(
                        '[ContactProbe] Wrench above threshold — '
                        'fx=%.1f fy=%.1f fz=%.1f tx=%.1f ty=%.1f tz=%.1f',
                        self._avg('fx'), self._avg('fy'), self._avg('fz'),
                        self._avg('tx'), self._avg('ty'), self._avg('tz'))
                elif (rospy.Time.now() - self._wrench_above_since).to_sec() >= self._hold_time:
                    self._contact_point = self._compute_contact_point()
                    self._classify_start_pos = (self._robot_x, self._robot_y)
                    self._classify_start_wrench_mag = self._wrench_magnitude()
                    self._enter(CLASSIFYING)
            else:
                if self._wrench_above_since is not None:
                    rospy.logdebug('[ContactProbe] Wrench dropped below threshold — resetting hold timer')
                self._wrench_above_since = None

            elapsed = self._elapsed()
            # Suspend the timeout once the wrench is above threshold — don't let
            # an early probe_timeout (from a false-trigger entry) race against
            # the 1 s hold_time needed to confirm real contact.
            if self._wrench_above_since is None and elapsed > self._probe_timeout:
                rospy.logwarn(
                    '[ContactProbe] Probe timeout (%.1fs) — no contact detected '
                    '(obs=%.2fm fx=%.1f fy=%.1f fz=%.1f) — resuming nav.',
                    elapsed, self._nearest_obs,
                    self._avg('fx'), self._avg('fy'), self._avg('fz'))
                self._enter(NORMAL_NAV)
            elif int(elapsed) != int(elapsed - 0.1) and elapsed > 0:
                # Log wrench values once per second while probing
                rospy.loginfo(
                    '[ContactProbe] Probing… t=%.0fs obs=%.2fm '
                    'fx=%.1f fy=%.1f fz=%.1f tx=%.1f ty=%.1f tz=%.1f',
                    elapsed, self._nearest_obs,
                    self._avg('fx'), self._avg('fy'), self._avg('fz'),
                    self._avg('tx'), self._avg('ty'), self._avg('tz'))

        elif state == CLASSIFYING:
            self._clamp_and_publish(self._probe_speed)

            if self._elapsed() >= self._classify_time:
                # Measure forward displacement in robot-heading direction
                dx = self._robot_x - self._classify_start_pos[0]
                dy = self._robot_y - self._classify_start_pos[1]
                forward = (dx * math.cos(self._robot_yaw) +
                           dy * math.sin(self._robot_yaw))

                w_now = self._wrench_magnitude()
                w_init = self._classify_start_wrench_mag or 1.0
                wrench_dropped = w_now < 0.5 * w_init

                movable_threshold = self._probe_speed * self._classify_time * 0.4

                # Primary discriminator: forward displacement.
                # A movable obstacle lets the robot advance; a solid one blocks it.
                # wrench_dropped is NOT used as a gate: when pushing a movable box
                # the robot maintains contact throughout classify_time, so the wrench
                # stays constant (not dropped) even though the box is clearly movable.
                if forward > movable_threshold:
                    rospy.loginfo(
                        '[ContactProbe] MOVABLE — Δfwd=%.3fm > %.3fm '
                        '[wrench_drop=%s w_now=%.1f w_init=%.1f]',
                        forward, movable_threshold, wrench_dropped, w_now, w_init)
                    self._enter(NORMAL_NAV)
                else:
                    rospy.logwarn(
                        '[ContactProbe] SOLID obstacle at (%.2f, %.2f) '
                        '[Δfwd=%.3fm <= %.3fm, wrench_drop=%s]',
                        self._contact_point[0], self._contact_point[1],
                        forward, movable_threshold, wrench_dropped)
                    self._solid_obstacles.append(self._contact_point)
                    self._publish_obstacles()
                    self._cancel_and_replan()
                    self._enter(NORMAL_NAV)


if __name__ == '__main__':
    rospy.init_node('contact_probe_manager')
    ContactProbeManager()
    rospy.spin()
