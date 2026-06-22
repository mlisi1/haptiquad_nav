#!/usr/bin/env python3
"""
Contact-probe state machine for HaptiQuad tactile navigation.

States:
  NORMAL_NAV    — full speed; original move_base goal active
  SLOW_APPROACH — speed_limit=slow_speed; original goal still active
  CONTACT_PROBE — probe goal sent to move_base; speed_limit=probe_speed; wrench monitored
  CLASSIFYING   — probe goal still active; displacement measured for classify_time
  BACKING_UP    — backup goal sent to move_base; speed_limit=backup_speed

contact_probe_manager is NOT in the velocity pipeline.  It manages move_base goals
and publishes /haptiquad_nav/speed_limit consumed by the carrot planner.
A separate twist_stamper node wraps move_base Twist → TwistStamped for twist_mux.
"""
import math
import struct
import collections
import numpy as np

import rospy
import tf.transformations
import tf2_ros
import actionlib

from std_msgs.msg import Float64
from geometry_msgs.msg import (WrenchStamped, PoseStamped, Quaternion,
                                PoseWithCovarianceStamped)
from sensor_msgs.msg import PointCloud2, PointField
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


# ── State identifiers ─────────────────────────────────────────────────────────
NORMAL_NAV    = 'NORMAL_NAV'
SLOW_APPROACH = 'SLOW_APPROACH'
CONTACT_PROBE = 'CONTACT_PROBE'
CLASSIFYING   = 'CLASSIFYING'
BACKING_UP    = 'BACKING_UP'


class ContactProbeManager:

    def __init__(self):
        # ── Parameters ────────────────────────────────────────────────────────
        self._thr_fx  = rospy.get_param('wrench_threshold/fx', 15.0)
        self._thr_fy  = rospy.get_param('wrench_threshold/fy', 10.0)
        self._thr_fz  = rospy.get_param('wrench_threshold/fz', 30.0)
        self._thr_tx  = rospy.get_param('wrench_threshold/tx', 3.0)
        self._thr_ty  = rospy.get_param('wrench_threshold/ty', 3.0)
        self._thr_tz  = rospy.get_param('wrench_threshold/tz', 2.0)

        self._filter_win   = rospy.get_param('wrench_filter_window', 10)
        self._slow_speed   = rospy.get_param('slow_approach_speed', 0.20)
        self._probe_speed  = rospy.get_param('probe_speed', 0.05)
        self._slow_dist    = rospy.get_param('slow_approach_distance', 1.0)
        self._probe_dist   = rospy.get_param('probe_distance', 0.65)
        self._probe_timeout = rospy.get_param('probe_timeout', 20.0)
        self._hold_time    = rospy.get_param('contact_hold_time', 1.0)
        self._mark_radius  = rospy.get_param('mark_radius', 0.05)
        self._classify_time = rospy.get_param('classify_time', 1.5)
        self._backup_speed  = rospy.get_param('backup_speed', 0.20)
        _robot_radius      = rospy.get_param('robot_radius', 0.55)
        _inflation_radius  = rospy.get_param('inflation_radius', 0.80)
        _backup_margin     = rospy.get_param('backup_extra_margin', 0.25)
        self._backup_clearance = _robot_radius + _inflation_radius + _backup_margin
        self._backup_min_clear = rospy.get_param('backup_min_clearance', 0.60)
        self._backup_time   = rospy.get_param('backup_time', 20.0)
        self._post_backup_cooldown = rospy.get_param('post_backup_cooldown', 3.0)

        self._probe_push_depth = rospy.get_param('probe_push_depth', 0.5)
        _probe_heading_deg     = rospy.get_param('probe_heading_max_deg', 25.0)
        self._probe_heading_thr = math.radians(_probe_heading_deg)

        # ── 3D cloud parameters ────────────────────────────────────────────────
        self._cloud_h_min  = rospy.get_param('cloud_height_min', -0.10)
        self._cloud_h_max  = rospy.get_param('cloud_height_max', 1.20)
        self._cloud_r_min  = rospy.get_param('cloud_range_min', 0.55)
        self._cloud_r_max  = rospy.get_param('cloud_range_max', 8.0)
        self._fwd_sector   = math.radians(rospy.get_param('forward_sector_deg', 45.0))
        solid_excl         = rospy.get_param('solid_excl_radius', 0.7)
        self._solid_excl_r2 = solid_excl ** 2

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
        self._wrench_above_since = None

        # ── Obstacle distance ─────────────────────────────────────────────────
        self._nearest_obs       = float('inf')
        self._nearest_obs_angle = 0.0        # base-frame angle to nearest fwd obs
        self._nearest_rear_obs  = float('inf')
        self._nearest_fwd_raw       = float('inf')
        self._last_fwd_pts_raw_map  = None   # (N,2) map-frame forward pts

        # ── Robot pose (map frame) ────────────────────────────────────────────
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0

        # ── Classify bookkeeping ──────────────────────────────────────────────
        self._classify_start_pos        = None
        self._classify_start_wrench_mag = None
        self._contact_point             = None   # (x, y, z) in map frame
        self._probe_entry_yaw           = 0.0
        self._probe_obs_dist            = 0.0    # measured obstacle dist at CONTACT_PROBE entry

        # ── Goal watchdog ─────────────────────────────────────────────────────
        self._goal_needs_resend = False

        # ── Confirmed solid obstacle list ─────────────────────────────────────
        self._solid_obstacles = []   # list of (x, y, z) centers
        self._solid_pts       = []   # list of [(x,y,z)...] — actual lidar pts per solid

        # ── Post-backup cooldown ──────────────────────────────────────────────
        self._backup_cooldown_until = rospy.Time(0)

        # ── Last user goal (updated only from /move_base_simple/goal) ─────────
        self._last_goal = None

        # ── TF buffer ─────────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf)

        # ── Publishers ────────────────────────────────────────────────────────
        self._speed_limit_pub = rospy.Publisher(
            '/haptiquad_nav/speed_limit', Float64, queue_size=1, latch=True)
        self._obstacles_pub = rospy.Publisher(
            '/haptiquad_nav/solid_obstacles', PointCloud2, queue_size=1, latch=True)

        # Publish uncapped speed at startup
        self._set_speed_limit(float('inf'))

        # ── move_base action client ───────────────────────────────────────────
        self._mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('[ContactProbe] Waiting for move_base action server…')
        self._mb_client.wait_for_server(timeout=rospy.Duration(30.0))
        rospy.loginfo('[ContactProbe] move_base ready.')

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('base_wrench',       WrenchStamped,              self._wrench_cb,  queue_size=1)
        rospy.Subscriber('point_cloud_in',    PointCloud2,                self._cloud_cb,   queue_size=1)
        rospy.Subscriber('/slam/pose',        PoseWithCovarianceStamped,  self._pose_cb,    queue_size=1)
        # Track USER goals only — internal probe/backup goals are sent via _mb_client
        # directly and must NOT overwrite _last_goal.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,           self._simple_goal_cb, queue_size=1)

        # ── Main loop ─────────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.1), self._update)
        rospy.Timer(rospy.Duration(2.0), lambda _: self._publish_obstacles())

        rospy.loginfo(
            '[ContactProbe] Started. cloud h=[%.2f,%.2f] r=[%.2f,%.2f] sector=±%.0f°',
            self._cloud_h_min, self._cloud_h_max,
            self._cloud_r_min, self._cloud_r_max,
            math.degrees(self._fwd_sector))

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _simple_goal_cb(self, msg):
        """Store the user's navigation goal (PoseStamped → MoveBaseGoal)."""
        goal = MoveBaseGoal()
        goal.target_pose = msg
        self._last_goal = goal
        rospy.loginfo('[ContactProbe] User goal: (%.2f, %.2f)',
                      msg.pose.position.x, msg.pose.position.y)

    def _wrench_cb(self, msg):
        w = msg.wrench
        self._wrench_buf['fx'].append(w.force.x)
        self._wrench_buf['fy'].append(w.force.y)
        self._wrench_buf['fz'].append(w.force.z)
        self._wrench_buf['tx'].append(w.torque.x)
        self._wrench_buf['ty'].append(w.torque.y)
        self._wrench_buf['tz'].append(w.torque.z)

    def _cloud_cb(self, msg):
        """Find nearest obstacle in forward ±sector cone from 3D pointcloud."""
        try:
            tf_msg = self._tf_buf.lookup_transform(
                'base', msg.header.frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, '[ContactProbe] TF lookup failed: %s', e)
            return

        n = msg.width * msg.height
        if n == 0:
            self._nearest_obs = float('inf')
            return
        step = msg.point_step
        fmap = {f.name: f.offset for f in msg.fields}
        if not all(k in fmap for k in ('x', 'y', 'z')):
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size != n * step:
            return
        raw = buf.reshape(n, step)

        ox, oy, oz = fmap['x'], fmap['y'], fmap['z']
        x = raw[:, ox:ox+4].copy().view(np.float32).reshape(-1)
        y = raw[:, oy:oy+4].copy().view(np.float32).reshape(-1)
        z = raw[:, oz:oz+4].copy().view(np.float32).reshape(-1)

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        if x.size == 0:
            self._nearest_obs = float('inf')
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        T = np.array([t.x, t.y, t.z])
        pts_b = np.stack([x, y, z], axis=1) @ R.T + T
        xb, yb, zb = pts_b[:, 0], pts_b[:, 1], pts_b[:, 2]

        h_ok = (zb >= self._cloud_h_min) & (zb <= self._cloud_h_max)
        xb, yb = xb[h_ok], yb[h_ok]
        if xb.size == 0:
            self._nearest_obs = float('inf')
            return

        r = np.sqrt(xb**2 + yb**2)
        r_ok = (r >= self._cloud_r_min) & (r <= self._cloud_r_max)
        xb, yb, r = xb[r_ok], yb[r_ok], r[r_ok]
        if r.size == 0:
            self._nearest_obs = float('inf')
            return

        angles = np.arctan2(yb, xb)

        # Rear sector (with solid exclusion)
        rear_ok = np.abs(angles) >= (math.pi - self._fwd_sector)
        if rear_ok.any():
            r_rear = r[rear_ok]
            if self._solid_obstacles:
                cyw_r = math.cos(self._robot_yaw)
                syw_r = math.sin(self._robot_yaw)
                xb_r, yb_r = xb[rear_ok], yb[rear_ok]
                keep_r = np.ones(r_rear.size, dtype=bool)
                for sox, soy, _ in self._solid_obstacles:
                    dx, dy = sox - self._robot_x, soy - self._robot_y
                    sbx =  dx * cyw_r + dy * syw_r
                    sby = -dx * syw_r + dy * cyw_r
                    keep_r &= (xb_r - sbx)**2 + (yb_r - sby)**2 > self._solid_excl_r2
                r_rear = r_rear[keep_r]
            self._nearest_rear_obs = float(np.min(r_rear)) if r_rear.size > 0 else float('inf')
        else:
            self._nearest_rear_obs = float('inf')

        # Forward sector
        a_ok = np.abs(angles) <= self._fwd_sector
        xb, yb, r = xb[a_ok], yb[a_ok], r[a_ok]
        angles_fwd = angles[a_ok]

        self._nearest_fwd_raw = float(np.min(r)) if r.size > 0 else float('inf')

        if xb.size > 0:
            cy_r = math.cos(self._robot_yaw)
            sy_r = math.sin(self._robot_yaw)
            self._last_fwd_pts_raw_map = np.stack(
                [self._robot_x + xb * cy_r - yb * sy_r,
                 self._robot_y + xb * sy_r + yb * cy_r], axis=1)
        else:
            self._last_fwd_pts_raw_map = None

        # Exclude confirmed solid obstacles from forward nearest-obstacle computation
        if self._solid_obstacles and r.size > 0:
            cyw = math.cos(self._robot_yaw)
            syw = math.sin(self._robot_yaw)
            keep = np.ones(r.size, dtype=bool)
            for sox, soy, _ in self._solid_obstacles:
                dx = sox - self._robot_x
                dy = soy - self._robot_y
                sbx =  dx * cyw + dy * syw
                sby = -dx * syw + dy * cyw
                keep &= (xb - sbx)**2 + (yb - sby)**2 > self._solid_excl_r2
            r = r[keep]
            angles_fwd = angles_fwd[keep]

        if r.size > 0:
            idx = int(np.argmin(r))
            self._nearest_obs = float(r[idx])
            self._nearest_obs_angle = float(angles_fwd[idx])
        else:
            self._nearest_obs = float('inf')
            self._nearest_obs_angle = 0.0

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._robot_x = p.position.x
        self._robot_y = p.position.y
        q = p.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self._robot_yaw = yaw

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

    # ── Speed limit ───────────────────────────────────────────────────────────

    def _set_speed_limit(self, v):
        """Publish a speed cap consumed by the carrot planner."""
        self._speed_limit_pub.publish(Float64(data=v))

    # ── Solid obstacle cloud ──────────────────────────────────────────────────

    def _publish_obstacles(self):
        if not self._solid_pts:
            return
        pts = []
        for obstacle_pts in self._solid_pts:
            pts.extend(obstacle_pts)
        if not pts:
            return

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

    def _refine_last_solid(self):
        """Mark the solid obstacle in the costmap using the current forward lidar scan.

        Called at the end of BACKING_UP so the robot is already clear of the lethal
        zone before the mark is published.  Falls back to a provisional ring if no
        lidar points are available near the known solid center.
        """
        if not self._solid_obstacles:
            return
        cx_known = self._solid_obstacles[-1][0]
        cy_known = self._solid_obstacles[-1][1]

        pts_map = self._last_fwd_pts_raw_map
        lidar_pts = []
        if pts_map is not None and pts_map.shape[0] > 0:
            known_pos = np.array([cx_known, cy_known])
            dists = np.linalg.norm(pts_map - known_pos[np.newaxis, :], axis=1)
            near = pts_map[dists <= 0.75]
            lidar_pts = [(float(r[0]), float(r[1]), 0.0) for r in near]

        if lidar_pts:
            self._solid_pts[-1] = lidar_pts
            rospy.loginfo('[ContactProbe] Marked solid at (%.2f,%.2f) — %d lidar pts',
                          cx_known, cy_known, len(lidar_pts))
        else:
            ox, oy = cx_known, cy_known
            prov = [(ox, oy, 0.0)]
            for _i in range(8):
                _a = 2 * math.pi * _i / 8
                prov.append((ox + self._mark_radius * math.cos(_a),
                             oy + self._mark_radius * math.sin(_a), 0.0))
            self._solid_pts[-1] = prov
            rospy.logwarn('[ContactProbe] No lidar pts near solid (%.2f,%.2f) — '
                          'using provisional ring.', cx_known, cy_known)
        self._publish_obstacles()

    # ── Goal management ───────────────────────────────────────────────────────

    def _send_goal(self):
        """Re-send the stored user goal to move_base."""
        if self._last_goal is not None:
            self._mb_client.send_goal(self._last_goal)
            rospy.loginfo('[ContactProbe] Original goal re-sent.')

    def _send_probe_goal(self):
        """Send a goal past the obstacle face by probe_push_depth so movable objects get pushed."""
        goal_dist = self._probe_obs_dist + self._probe_push_depth
        gx = self._robot_x + goal_dist * math.cos(self._probe_entry_yaw)
        gy = self._robot_y + goal_dist * math.sin(self._probe_entry_yaw)
        q = tf.transformations.quaternion_from_euler(0, 0, self._probe_entry_yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self._mb_client.send_goal(goal)
        rospy.loginfo('[ContactProbe] Probe goal → (%.2f, %.2f) [obs=%.2f push=%.2f]',
                      gx, gy, self._probe_obs_dist, self._probe_push_depth)

    def _send_backup_goal(self):
        """Send a goal backup_clearance_target m behind the contact point.

        The global planner handles feasibility: if the backup point is inside
        a lethal zone it will route around it rather than fail outright.
        """
        gx = self._contact_point[0] - self._backup_clearance * math.cos(self._probe_entry_yaw)
        gy = self._contact_point[1] - self._backup_clearance * math.sin(self._probe_entry_yaw)

        if self._last_goal is not None:
            orient = self._last_goal.target_pose.pose.orientation
        else:
            q = tf.transformations.quaternion_from_euler(
                0, 0, self._probe_entry_yaw + math.pi)
            orient = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation = orient
        self._mb_client.send_goal(goal)
        rospy.loginfo('[ContactProbe] Backup goal → (%.2f, %.2f)', gx, gy)

    # ── State transitions ─────────────────────────────────────────────────────

    def _enter(self, new_state):
        prev_state = self._state
        rospy.loginfo('[ContactProbe] %s → %s  (obs=%.2fm)',
                      prev_state, new_state, self._nearest_obs)
        self._state = new_state
        self._state_enter_time = rospy.Time.now()

        if new_state == NORMAL_NAV:
            self._set_speed_limit(float('inf'))
            self._goal_needs_resend = False
            if prev_state in (CONTACT_PROBE, CLASSIFYING, BACKING_UP):
                self._send_goal()   # preempts any active goal implicitly
                self._goal_needs_resend = True   # watch for move_base abort
            if prev_state == BACKING_UP:
                self._backup_cooldown_until = (
                    rospy.Time.now() + rospy.Duration(self._post_backup_cooldown))
                rospy.loginfo('[ContactProbe] Post-backup cooldown %.1f s',
                              self._post_backup_cooldown)

        elif new_state == SLOW_APPROACH:
            self._set_speed_limit(self._slow_speed)

        elif new_state == CONTACT_PROBE:
            obs_dist = self._nearest_obs if math.isfinite(self._nearest_obs) else self._probe_dist
            obs_dist = max(0.1, min(obs_dist, self._probe_dist))
            self._probe_obs_dist  = obs_dist
            self._contact_point   = (self._robot_x + obs_dist * math.cos(self._robot_yaw),
                                     self._robot_y + obs_dist * math.sin(self._robot_yaw),
                                     0.0)
            self._probe_entry_yaw = self._robot_yaw
            self._send_probe_goal()   # preempts active user/probe goal implicitly
            self._set_speed_limit(self._probe_speed)

        elif new_state == CLASSIFYING:
            self._classify_start_pos        = (self._robot_x, self._robot_y)
            self._classify_start_wrench_mag = self._wrench_magnitude()
            # Probe goal stays active; speed_limit stays at probe_speed.

        elif new_state == BACKING_UP:
            self._send_backup_goal()   # preempts active probe/classify goal implicitly
            self._set_speed_limit(self._backup_speed)

    def _elapsed(self):
        return (rospy.Time.now() - self._state_enter_time).to_sec()

    # ── Main update loop ──────────────────────────────────────────────────────

    def _update(self, _event):
        state = self._state

        if state == NORMAL_NAV:
            # Goal watchdog: if move_base aborted the goal we just resent, try again.
            if self._goal_needs_resend and self._last_goal is not None:
                mb_state = self._mb_client.get_state()
                if mb_state == GoalStatus.ABORTED and self._elapsed() > 0.5:
                    rospy.logwarn('[ContactProbe] move_base aborted resent goal — retrying.')
                    self._send_goal()
                    self._state_enter_time = rospy.Time.now()  # reset so we don't loop
                elif mb_state not in (GoalStatus.ABORTED, GoalStatus.LOST):
                    self._goal_needs_resend = False  # goal accepted and alive

            if (self._nearest_obs <= self._slow_dist and
                    rospy.Time.now() >= self._backup_cooldown_until):
                self._enter(SLOW_APPROACH)

        elif state == SLOW_APPROACH:
            if self._nearest_obs > self._slow_dist:
                self._enter(NORMAL_NAV)
            elif (self._nearest_obs <= self._probe_dist and
                    abs(self._nearest_obs_angle) <= self._probe_heading_thr):
                self._wrench_above_since = None
                self._enter(CONTACT_PROBE)

        elif state == CONTACT_PROBE:
            mb_state = self._mb_client.get_state()
            if mb_state in (GoalStatus.ABORTED, GoalStatus.RECALLED):
                rospy.logwarn('[ContactProbe] Probe goal aborted/recalled — resuming nav.')
                self._enter(NORMAL_NAV)
                return

            if self._wrench_above_threshold():
                if self._wrench_above_since is None:
                    self._wrench_above_since = rospy.Time.now()
                    rospy.loginfo(
                        '[ContactProbe] Wrench above threshold — '
                        'fx=%.1f fy=%.1f fz=%.1f tx=%.1f ty=%.1f tz=%.1f',
                        self._avg('fx'), self._avg('fy'), self._avg('fz'),
                        self._avg('tx'), self._avg('ty'), self._avg('tz'))
                elif (rospy.Time.now() - self._wrench_above_since).to_sec() >= self._hold_time:
                    self._enter(CLASSIFYING)
            else:
                if self._wrench_above_since is not None:
                    rospy.logdebug('[ContactProbe] Wrench dropped — resetting hold timer')
                self._wrench_above_since = None

            elapsed = self._elapsed()
            if self._wrench_above_since is None and elapsed > self._probe_timeout:
                rospy.logwarn(
                    '[ContactProbe] Probe timeout (%.1fs) — no contact '
                    '(obs=%.2fm fx=%.1f fy=%.1f fz=%.1f) — resuming nav.',
                    elapsed, self._nearest_obs,
                    self._avg('fx'), self._avg('fy'), self._avg('fz'))
                self._enter(NORMAL_NAV)
            elif int(elapsed / 5) != int((elapsed - 0.1) / 5) and elapsed > 0:
                rospy.loginfo(
                    '[ContactProbe] Probing… t=%.0fs obs=%.2fm '
                    'fx=%.1f fy=%.1f fz=%.1f tx=%.1f ty=%.1f tz=%.1f',
                    elapsed, self._nearest_obs,
                    self._avg('fx'), self._avg('fy'), self._avg('fz'),
                    self._avg('tx'), self._avg('ty'), self._avg('tz'))

        elif state == CLASSIFYING:
            mb_state = self._mb_client.get_state()
            if mb_state in (GoalStatus.ABORTED, GoalStatus.RECALLED):
                rospy.logwarn('[ContactProbe] Classify goal aborted/recalled — resuming nav.')
                self._enter(NORMAL_NAV)
                return

            if self._elapsed() >= self._classify_time:
                dx = self._robot_x - self._classify_start_pos[0]
                dy = self._robot_y - self._classify_start_pos[1]
                forward = (dx * math.cos(self._probe_entry_yaw) +
                           dy * math.sin(self._probe_entry_yaw))

                w_now  = self._wrench_magnitude()
                w_init = self._classify_start_wrench_mag or 1.0
                wrench_dropped    = w_now < 0.5 * w_init
                movable_threshold = self._probe_speed * self._classify_time * 0.4

                if forward > movable_threshold:
                    rospy.loginfo(
                        '[ContactProbe] MOVABLE — Δfwd=%.3fm > %.3fm '
                        '[wrench_drop=%s w_now=%.1f w_init=%.1f]',
                        forward, movable_threshold, wrench_dropped, w_now, w_init)
                    self._enter(NORMAL_NAV)
                else:
                    rospy.logwarn(
                        '[ContactProbe] SOLID at (%.2f, %.2f) '
                        '[Δfwd=%.3fm ≤ %.3fm, wrench_drop=%s]',
                        self._contact_point[0], self._contact_point[1],
                        forward, movable_threshold, wrench_dropped)
                    self._solid_obstacles.append(self._contact_point)
                    # Placeholder — filled by _refine_last_solid() after backup.
                    # Delaying the costmap mark until after backup ensures the
                    # robot's start cell is not inside the lethal zone of its own
                    # mark when the backup goal is planned (NavFnROS refuses to
                    # plan from a lethal cell, causing immediate ABORTED backup).
                    self._solid_pts.append([])
                    self._enter(BACKING_UP)

        elif state == BACKING_UP:
            # Rear unknown-obstacle safety stop
            if self._nearest_rear_obs <= self._backup_min_clear:
                rospy.logwarn(
                    '[ContactProbe] Rear obstacle (%.2fm ≤ %.2fm) — ending backup.',
                    self._nearest_rear_obs, self._backup_min_clear)
                self._refine_last_solid()
                self._enter(NORMAL_NAV)
                return

            mb_state     = self._mb_client.get_state()
            backup_done  = mb_state in (GoalStatus.SUCCEEDED,
                                        GoalStatus.ABORTED,
                                        GoalStatus.PREEMPTED)
            dist_to_solid = (math.hypot(self._robot_x - self._contact_point[0],
                                        self._robot_y - self._contact_point[1])
                             if self._contact_point else float('inf'))

            if backup_done or dist_to_solid >= self._backup_clearance or \
                    self._elapsed() >= self._backup_time:
                if self._elapsed() >= self._backup_time:
                    rospy.logwarn('[ContactProbe] Backup timeout — %.2fm from solid.',
                                  dist_to_solid)
                elif mb_state == GoalStatus.ABORTED:
                    rospy.logwarn('[ContactProbe] Backup goal aborted by planner — '
                                  '%.2fm from solid.', dist_to_solid)
                else:
                    rospy.loginfo('[ContactProbe] Backup complete — %.2fm from solid.',
                                  dist_to_solid)
                self._refine_last_solid()
                self._enter(NORMAL_NAV)


if __name__ == '__main__':
    rospy.init_node('contact_probe_manager')
    ContactProbeManager()
    rospy.spin()
