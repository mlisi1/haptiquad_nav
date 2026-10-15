#!/usr/bin/env python3
"""
Contact-probe state machine for HaptiQuad tactile navigation.

States:
  NORMAL_NAV    — full speed; original move_base goal active
  SLOW_APPROACH — speed_limit=slow_speed; original goal still active
  CONTACT_PROBE — probe goal sent to move_base; speed_limit=probe_speed; wrench monitored
  CLASSIFYING   — probe goal still active; displacement measured for classify_time
  BACKING_UP    — backup goal sent to move_base; speed_limit=backup_speed
  CALIBRATING   — startup only (wrench_bias_calibration_s > 0): wrench bias averaged, no motion
  HALTED        — stale/unhealthy input, unexpected contact or a reported fault:
                  goals cancelled, speed_limit=0; a new user goal resumes

contact_probe_manager is NOT in the velocity pipeline.  It manages move_base goals
and publishes /haptiquad_nav/speed_limit consumed by the carrot planner, plus the
/haptiquad_nav/motion_allowed heartbeat cmd_vel_gate requires to forward commands.
"""
import math
import struct
import collections
import functools
import threading
import traceback
import numpy as np

import rospy
import tf.transformations
import tf2_ros
import actionlib

from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import (WrenchStamped, PoseStamped, Quaternion,
                                PoseWithCovarianceStamped, PolygonStamped)
from sensor_msgs.msg import PointCloud2, PointField
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse


# ── State identifiers ─────────────────────────────────────────────────────────
NORMAL_NAV    = 'NORMAL_NAV'
SLOW_APPROACH = 'SLOW_APPROACH'
CONTACT_PROBE = 'CONTACT_PROBE'
CLASSIFYING   = 'CLASSIFYING'
BACKING_UP    = 'BACKING_UP'
# Startup only: averaging the standing wrench to remove its bias. No motion.
CALIBRATING   = 'CALIBRATING'
# Safety stop: goals cancelled, speed_limit 0, motion_allowed False. Left only
# by a NEW user goal once every input is healthy again (see _simple_goal_cb).
HALTED        = 'HALTED'


def _synchronized(fn):
    """Run a callback under self._lock. rospy runs every subscriber, timer
    and service callback in its own thread, so without this the ~reset
    service (or a probing toggle / known-solid injection) can interleave
    with _update mid-transition and leave state from the previous trial --
    observed in haptiquad_nav_eval's tuning sweep as a SOLID decision
    logged before the new trial ever entered CONTACT_PROBE."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return fn(self, *args, **kwargs)
    return wrapper


class ContactProbeManager:

    def __init__(self):
        self._lock = threading.RLock()

        # ── Parameters ────────────────────────────────────────────────────────
        # fz/tx/ty/tz are deliberately NOT used for contact detection: fz and the
        # torque axes are dominated by walking-gait dynamics (see contact_probe.yaml),
        # not genuine object contact, and using them caused both false CONTACT_PROBE
        # triggers and a SOLID bias in force-mode classification. Detection and
        # classification both rely on the horizontal force (fx/fy) only.
        # Single threshold on the HEADING-PROJECTED force (see
        # _wrench_forward_mag), not two independent per-axis (fx, fy)
        # thresholds -- matches the pipeline formalization's contact-detection
        # rule |f_hat(t)| > theta exactly. The previous per-axis OR check was
        # anisotropic: at a 45 degree approach heading it required a resultant
        # force up to threshold*sqrt(2) to trigger (neither raw axis component
        # alone reached the threshold), while a 0/90 degree heading only
        # needed the threshold itself. A single threshold on the projected
        # magnitude is isotropic -- the same required force regardless of
        # approach heading.
        self._contact_threshold_n = rospy.get_param('~contact_force_threshold_n', 12.0)

        self._filter_win   = rospy.get_param('~wrench_filter_window', 10)
        self._slow_speed   = rospy.get_param('~slow_approach_speed', 0.20)
        self._probe_speed  = rospy.get_param('~probe_speed', 0.05)
        self._slow_dist    = rospy.get_param('~slow_approach_distance', 1.0)
        self._probe_dist   = rospy.get_param('~probe_distance', 0.65)
        self._probe_timeout = rospy.get_param('~probe_timeout', 20.0)
        self._hold_time    = rospy.get_param('~contact_hold_time', 1.0)
        self._mark_radius  = rospy.get_param('~mark_radius', 0.05)
        self._classify_time = rospy.get_param('~classify_time', 1.5)

        # ── Classification criterion (ablation toggle) ──────────────────────────
        # 'force' (default): MOVABLE iff the settled forward-push force during the
        #   classify window stays below classification_force_threshold_n. An earlier
        #   relative-drop criterion (is w_now < 0.5*w_init) was replaced: under
        #   Coulomb friction a continuously-pushed sliding object exerts a roughly
        #   CONSTANT resisting force (~mu*m*g), it doesn't decay over time, so a
        #   relative-drop test can't distinguish "sliding at steady kinetic friction"
        #   from "genuinely stuck" -- confirmed empirically (test_sweep_force_5:
        #   28/32 trials SOLID, 0/32 MOVABLE across the whole mass/mu sweep). An
        #   absolute force ceiling is the physically meaningful criterion instead.
        # 'displacement': MOVABLE iff the base keeps translating forward through
        #   the object during the classify window (kinematic criterion).
        self._classification_mode = rospy.get_param('~classification_mode', 'force')
        if self._classification_mode not in ('displacement', 'force'):
            rospy.logwarn(
                "[ContactProbe] Unknown classification_mode '%s' -- falling back to "
                "'force'. Valid values: 'displacement', 'force'.",
                self._classification_mode)
            self._classification_mode = 'force'
        # How long into CLASSIFYING to wait before sampling starts, so the initial
        # contact transient (impact spike / static-friction breakaway) isn't
        # averaged into the "settled" reading.
        self._force_settle_time = rospy.get_param('~classification_force_settle_time', 0.5)
        # MOVABLE iff the mean settled forward force stays below this. Default is
        # push_capability_mass_fraction(0.25) * robot_mass_kg(52.13) * g -- the same
        # "robot could still move ~25% of its own mass at this speed" definition
        # already used as offline ground truth in plotting.py's default_config().
        # Treat this as a starting point: refine using the sweep's own
        # force_vs_static_friction / confusion-matrix plots once you have data.
        self._force_threshold_n = rospy.get_param('~classification_force_threshold_n', 127.8)
        rospy.loginfo('[ContactProbe] classification_mode=%s (force_threshold=%.1fN, settle=%.1fs)',
                      self._classification_mode, self._force_threshold_n, self._force_settle_time)
        self._backup_speed  = rospy.get_param('~backup_speed', 0.20)
        _robot_radius      = rospy.get_param('~robot_radius', 0.55)
        _inflation_radius  = rospy.get_param('~inflation_radius', 0.80)
        _backup_margin     = rospy.get_param('~backup_extra_margin', 0.25)
        self._backup_clearance = _robot_radius + _inflation_radius + _backup_margin
        self._backup_min_clear = rospy.get_param('~backup_min_clearance', 0.60)
        self._backup_time   = rospy.get_param('~backup_time', 20.0)
        self._post_backup_cooldown = rospy.get_param('~post_backup_cooldown', 3.0)
        self._obstacle_depth = rospy.get_param('~obstacle_assumed_depth', 0.5)

        self._probe_push_depth = rospy.get_param('~probe_push_depth', 0.5)
        _probe_heading_deg     = rospy.get_param('~probe_heading_max_deg', 25.0)
        self._probe_heading_thr = math.radians(_probe_heading_deg)

        # ── Safety ─────────────────────────────────────────────────────────────
        # Inputs older than these (receive time) halt the robot: without them
        # the state machine would act on frozen wrench/pose/obstacle data.
        self._wrench_timeout = rospy.get_param('~input_timeout/wrench', 0.3)
        self._pose_timeout   = rospy.get_param('~input_timeout/pose', 1.0)
        self._cloud_timeout  = rospy.get_param('~input_timeout/cloud', 1.0)
        # Heartbeat from nav_supervisor.py (all navigation inputs healthy).
        self._require_health = rospy.get_param('~require_health', True)
        self._health_timeout = rospy.get_param('~health_timeout', 0.5)
        # Contact outside CONTACT_PROBE/CLASSIFYING (side/rear hits, anything
        # the lidar filter missed): |fx,fy| above this for collision_hold_time
        # halts the robot. While pushing an object already classified MOVABLE
        # the bound is classification_force_threshold_n instead.
        self._collision_stop  = rospy.get_param('~collision_stop_enabled', True)
        self._collision_thr   = rospy.get_param('~collision_force_threshold_n', 30.0)
        self._collision_hold  = rospy.get_param('~collision_hold_time', 0.3)
        self._collision_backup_grace = rospy.get_param('~collision_backup_grace_s', 1.5)
        # Standing-wrench bias removal at startup (0 disables). Also measures
        # the wrench rate, so wrench_filter_window_s (if > 0) can set the
        # filter window in seconds rather than samples.
        self._bias_calib_time = rospy.get_param('~wrench_bias_calibration_s', 0.0)
        self._filter_win_s    = rospy.get_param('~wrench_filter_window_s', 0.0)
        # Reset/override/injection hooks for haptiquad_nav_eval only. Off by
        # default so nothing on the robot network can disable probing or
        # change probe parameters.
        self._eval_mode = rospy.get_param('~eval_mode', False)

        # ── 3D cloud parameters ────────────────────────────────────────────────
        self._cloud_h_min  = rospy.get_param('~cloud_height_min', -0.10)
        self._cloud_h_max  = rospy.get_param('~cloud_height_max', 1.20)
        self._cloud_r_min  = rospy.get_param('~cloud_range_min', 0.55)
        self._cloud_r_max  = rospy.get_param('~cloud_range_max', 8.0)
        self._fwd_sector   = math.radians(rospy.get_param('~forward_sector_deg', 45.0))
        # 'base': cloud_height_min/max are along the base z axis (tilts with
        # the body). 'gravity': along gravity, still relative to the base
        # origin, so pitching on slopes doesn't shift the band into the
        # ground or over low obstacles.
        self._cloud_h_ref  = rospy.get_param('~cloud_height_reference', 'base')
        self._odom_frame   = rospy.get_param('~odom_frame', 'odom')
        # [x_min, x_max, y_min, y_max] in base frame: drop returns from the
        # robot's own body/legs. Empty = disabled (cloud_range_min only).
        self._self_box     = rospy.get_param('~self_filter_box', [])
        solid_excl         = rospy.get_param('~solid_excl_radius', 0.7)
        self._solid_excl_r2 = solid_excl ** 2
        movable_excl        = rospy.get_param('~movable_excl_radius', 1.5)
        self._movable_excl_r2 = movable_excl ** 2
        self._movable_track_r    = rospy.get_param('~movable_track_radius', 0.5)
        self._movable_track_min  = rospy.get_param('~movable_track_min_pts', 3)
        # Radius (m) around a confirmed solid's contact point within which
        # _refine_last_solid() collects lidar points to measure the face width.
        # Was a hardcoded 0.75 -- promoted to a param so haptiquad_nav_eval's
        # timing sweep can override it live (see the ~*_override subscribers
        # below) for obstacles much longer than the ~0.5m test-world default.
        self._refine_capture_r = rospy.get_param('~solid_refine_capture_radius', 0.75)

        # ── Probing bypass (haptiquad_nav_eval timing-comparison harness only) ──
        # Always True in normal operation and for the existing mass/mu sweep --
        # only haptiquad_nav_eval's timing_sweep_orchestrator.py ever calls
        # ~set_probing_enabled(False), to measure a "classical" (obstacle
        # pre-known, no tactile probing) baseline against the same move_base
        # stack. See _update()'s early-return and _set_probing_enabled_cb.
        self._probing_enabled = True

        # ── State ─────────────────────────────────────────────────────────────
        self._state = CALIBRATING if self._bias_calib_time > 0 else NORMAL_NAV
        self._state_enter_time = rospy.Time.now()
        self._halt_reason = ''
        self._last_wrench_t = None
        self._last_pose_t   = None
        self._last_cloud_t  = None
        self._last_health_t = None
        self._health_ok     = False
        self._collision_since = None
        self._bias = {'fx': 0.0, 'fy': 0.0}
        self._calib_samples = []   # (t, fx, fy) while CALIBRATING
        self._motion_allowed = False
        self._max_bias = rospy.get_param('~max_wrench_bias_n', 40.0)

        # ── Runtime diagnostics (loop rate / wrench latency) ────────────────────
        # Consumed by haptiquad_nav_eval's trial_logger.py, but standalone-useful
        # (view with rqt_runtime_monitor or `rostopic echo /diagnostics`).
        self._update_period_target = 0.1   # must match the Timer period below
        self._loop_period_buf   = collections.deque(maxlen=50)
        self._last_update_wall  = None
        self._wrench_latency_buf = collections.deque(maxlen=50)

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
        self._last_all_pts_map      = None   # (N,2) map-frame full-360° pts

        # ── Robot pose (map frame) ────────────────────────────────────────────
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0

        # ── Classify bookkeeping ──────────────────────────────────────────────
        self._classify_start_pos        = None
        self._classify_force_samples    = []   # _wrench_forward_mag() samples, post-settle
        self._contact_point             = None   # (x, y, z) in map frame
        self._probe_entry_yaw           = 0.0
        self._probe_obs_dist            = 0.0    # measured obstacle dist at CONTACT_PROBE entry

        # ── Goal watchdog ─────────────────────────────────────────────────────
        self._goal_needs_resend = False

        # ── Confirmed solid obstacle list ─────────────────────────────────────
        self._solid_obstacles = []   # list of (x, y, z) centers
        self._solid_pts       = []   # list of [(x,y,z)...] — actual lidar pts per solid

        # ── Confirmed movable obstacle list ───────────────────────────────────
        # Contact centers of obstacles classified as movable.  Excluded from
        # _nearest_obs so the robot does not re-trigger SLOW_APPROACH after
        # pushing through.  NOT published to the costmap.
        self._movable_obstacles = []   # list of (x, y, z) contact centers

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
        self._movable_pub = rospy.Publisher(
            '/haptiquad_nav/movable_obstacles', PointCloud2, queue_size=1, latch=True)
        # Current state-machine state, published on every transition. Consumed by
        # haptiquad_nav_eval to bound each trial's time-series recording window
        # and to derive the classification outcome (BACKING_UP visited => SOLID,
        # CLASSIFYING->NORMAL_NAV directly => MOVABLE) without re-implementing
        # the state machine's own logic.
        self._state_pub = rospy.Publisher(
            '/haptiquad_nav/state', String, queue_size=1, latch=True)
        self._diag_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1)
        # Safety-margin telemetry: distance to nearest confirmed-unknown
        # obstacle, forward and rear sector. Not used by this node's own
        # control logic (that's still self._nearest_obs / self._nearest_rear_obs
        # directly) -- this is purely so an external observer (trial_logger.py)
        # can log how close the robot actually got, without duplicating the
        # lidar-processing logic in _cloud_cb.
        self._nearest_obs_pub = rospy.Publisher(
            '/haptiquad_nav/nearest_obstacle_distance', Float64, queue_size=1)
        self._nearest_rear_obs_pub = rospy.Publisher(
            '/haptiquad_nav/nearest_rear_obstacle_distance', Float64, queue_size=1)
        # 10Hz heartbeat read by cmd_vel_gate.py: True only when this node is
        # alive, calibrated, not HALTED and all its inputs are fresh. If this
        # node (or its update loop) dies, the heartbeat stops and the gate
        # stops forwarding velocity commands.
        self._motion_allowed_pub = rospy.Publisher(
            '/haptiquad_nav/motion_allowed', Bool, queue_size=1)
        self._halt_reason_pub = rospy.Publisher(
            '/haptiquad_nav/halt_reason', String, queue_size=1, latch=True)

        # Reset hook for external harnesses (e.g. haptiquad_nav_eval) that
        # reset the Gazebo world / robot pose between trials. Without this,
        # _solid_obstacles / _movable_obstacles / the permanent costmap marks
        # accumulate for the node's entire lifetime and silently leak into
        # the next trial after a world reset.
    def _register_eval_hooks(self):
        self._reset_srv = rospy.Service('~reset', Empty, self._reset_cb)

        # Opt-in bypass switch + live radius overrides for haptiquad_nav_eval's
        # timing_sweep_orchestrator.py. Neither is ever touched by the existing
        # mass/mu sweep (sweep_orchestrator.py/trial_logger.py) -- the two
        # sweeps run against this same long-lived node but stay behaviorally
        # independent as long as nothing publishes/calls these.
        self._set_probing_srv = rospy.Service(
            '~set_probing_enabled', SetBool, self._set_probing_enabled_cb)
        rospy.Subscriber('~solid_excl_radius_override', Float64,
                         self._solid_excl_override_cb, queue_size=1)
        rospy.Subscriber('~movable_excl_radius_override', Float64,
                         self._movable_excl_override_cb, queue_size=1)
        rospy.Subscriber('~solid_refine_capture_radius_override', Float64,
                         self._refine_capture_override_cb, queue_size=1)

        # Live overrides for haptiquad_nav_eval's tuning_sweep_orchestrator.py
        # (contact-probe PARAMETER-TUNING harness) -- every parameter that can
        # influence classification TIME (approach-to-contact speed/range, the
        # contact-confirmation dwell, the decision window and its settle
        # time, and wrench smoothing). Deliberately excludes force thresholds
        # (contact_force_threshold_n, classification_force_threshold_n -- those
        # affect correctness, not timing, and the mass/mu sweep already
        # covers them) and all map/costmap parameters. Untouched by the
        # other two sweeps, same independence guarantee as the radius
        # overrides above.
        rospy.Subscriber('~probe_speed_override', Float64,
                         self._probe_speed_override_cb, queue_size=1)
        rospy.Subscriber('~probe_distance_override', Float64,
                         self._probe_distance_override_cb, queue_size=1)
        rospy.Subscriber('~contact_hold_time_override', Float64,
                         self._contact_hold_time_override_cb, queue_size=1)
        rospy.Subscriber('~classify_time_override', Float64,
                         self._classify_time_override_cb, queue_size=1)
        rospy.Subscriber('~classification_force_settle_time_override', Float64,
                         self._force_settle_time_override_cb, queue_size=1)
        rospy.Subscriber('~wrench_filter_window_override', Float64,
                         self._filter_window_override_cb, queue_size=1)

        # "Classical" (a priori known obstacle) baseline for
        # haptiquad_nav_eval's timing sweep: injects a ground-truth
        # rectangle straight into the global costmap through the SAME
        # _solid_obstacles/_solid_pts path a real probe uses, without ever
        # running CONTACT_PROBE/CLASSIFYING. ~set_probing_enabled alone is
        # NOT enough to produce a valid "classical" trial -- it only stops
        # the state machine from slowing down/probing, it does nothing to
        # tell the GLOBAL PLANNER the obstacle exists, so without this the
        # robot walks straight through it.
        rospy.Subscriber('~inject_known_solid', PolygonStamped,
                         self._inject_known_solid_cb, queue_size=1)

    def start(self):
        # No motion until calibrated: speed 0 while CALIBRATING.
        self._set_speed_limit(float('inf'))
        self._state_pub.publish(String(data=self._state))

        # ── move_base action client ───────────────────────────────────────────
        # Wait as long as it takes: running without move_base would make
        # every goal/cancel below a silent no-op.
        self._mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not rospy.is_shutdown():
            rospy.loginfo('[ContactProbe] Waiting for move_base action server…')
            if self._mb_client.wait_for_server(timeout=rospy.Duration(10.0)):
                break
        rospy.loginfo('[ContactProbe] move_base ready.')
        if self._eval_mode:   # the hooks use _mb_client, so only now
            rospy.logwarn('[ContactProbe] eval_mode: reset/override/injection hooks enabled.')
            self._register_eval_hooks()

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('base_wrench',       WrenchStamped,              self._wrench_cb,  queue_size=1)
        rospy.Subscriber('point_cloud_in',    PointCloud2,                self._cloud_cb,   queue_size=1)
        rospy.Subscriber('/slam/pose',        PoseWithCovarianceStamped,  self._pose_cb,    queue_size=1)
        # Track USER goals only — internal probe/backup goals are sent via _mb_client
        # directly and must NOT overwrite _last_goal.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,           self._simple_goal_cb, queue_size=1)
        rospy.Subscriber('/haptiquad_nav/health', Bool,                   self._health_cb, queue_size=1)
        # Faults detected downstream (cmd_vel_gate.py: controller stuck, bad
        # command) -- halt here too so no goal stays active behind a closed gate.
        rospy.Subscriber('/haptiquad_nav/fault', String, self._fault_cb, queue_size=10)

        # Calibration window starts now that wrench samples can arrive.
        self._state_enter_time = rospy.Time.now()

        # ── Main loop ─────────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(self._update_period_target), self._update_guarded)
        rospy.Timer(rospy.Duration(2.0), lambda _: self._publish_obstacles())
        rospy.Timer(rospy.Duration(2.0), lambda _: self._publish_movable_obstacles())
        rospy.Timer(rospy.Duration(1.0), self._publish_diagnostics)

        rospy.loginfo(
            '[ContactProbe] Started. cloud h=[%.2f,%.2f] r=[%.2f,%.2f] sector=±%.0f°',
            self._cloud_h_min, self._cloud_h_max,
            self._cloud_r_min, self._cloud_r_max,
            math.degrees(self._fwd_sector))

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    @_synchronized
    def _simple_goal_cb(self, msg):
        """Store the user's navigation goal (PoseStamped → MoveBaseGoal).
        move_base receives the same goal directly; while CALIBRATING, or
        HALTED with an input still unhealthy, it is cancelled straight away.
        A new goal is the only way out of HALTED."""
        if self._state == CALIBRATING:
            rospy.logwarn('[ContactProbe] Goal ignored: wrench bias calibration in progress.')
            self._mb_client.cancel_all_goals()
            return
        if self._state == HALTED:
            problem = self._input_problem()
            if problem:
                rospy.logwarn('[ContactProbe] Goal ignored, still HALTED: %s', problem)
                self._mb_client.cancel_all_goals()
                return
            rospy.loginfo('[ContactProbe] New goal received -- leaving HALTED.')
            self._halt_reason = ''
            self._halt_reason_pub.publish(String(data=''))
            self._enter(NORMAL_NAV)
        goal = MoveBaseGoal()
        goal.target_pose = msg
        self._last_goal = goal
        rospy.loginfo('[ContactProbe] User goal: (%.2f, %.2f)',
                      msg.pose.position.x, msg.pose.position.y)

    @_synchronized
    def _wrench_cb(self, msg):
        w = msg.wrench
        fx, fy = w.force.x, w.force.y
        if not (math.isfinite(fx) and math.isfinite(fy)):
            rospy.logwarn_throttle(2.0, '[ContactProbe] Non-finite wrench dropped.')
            return
        self._last_wrench_t = rospy.Time.now()
        if self._state == CALIBRATING:
            self._calib_samples.append((self._last_wrench_t.to_sec(), fx, fy))
        self._wrench_buf['fx'].append(fx - self._bias['fx'])
        self._wrench_buf['fy'].append(fy - self._bias['fy'])
        self._wrench_buf['fz'].append(w.force.z)
        self._wrench_buf['tx'].append(w.torque.x)
        self._wrench_buf['ty'].append(w.torque.y)
        self._wrench_buf['tz'].append(w.torque.z)

        latency = (rospy.Time.now() - msg.header.stamp).to_sec()
        if 0.0 <= latency < 5.0:   # discard clock-skew/sim-time-jump outliers
            self._wrench_latency_buf.append(latency)

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
            # Not counted as fresh: a real lidar never returns nothing, so a
            # run of empty clouds is a sensor problem, not free space.
            self._nearest_obs = float('inf')
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        T = np.array([t.x, t.y, t.z])
        pts_b = np.stack([x, y, z], axis=1) @ R.T + T
        xb, yb, zb = pts_b[:, 0], pts_b[:, 1], pts_b[:, 2]

        if self._cloud_h_ref == 'gravity':
            try:
                q_ob = self._tf_buf.lookup_transform(
                    self._odom_frame, 'base', rospy.Time(0)).transform.rotation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, '[ContactProbe] TF lookup failed: %s', e)
                return
            R_ob = tf.transformations.quaternion_matrix([q_ob.x, q_ob.y, q_ob.z, q_ob.w])[:3, :3]
            zb = pts_b @ R_ob[2, :]   # height along gravity, base origin
        # Fresh only once every lookup succeeded; the rest is filtering.
        self._last_cloud_t = rospy.Time.now()

        h_ok = (zb >= self._cloud_h_min) & (zb <= self._cloud_h_max)
        if len(self._self_box) == 4:
            bx0, bx1, by0, by1 = self._self_box
            h_ok &= ~((xb >= bx0) & (xb <= bx1) & (yb >= by0) & (yb <= by1))
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

        # Full-360° map-frame snapshot (height+range filtered, no sector cut)
        # Used by _refine_last_solid to capture side faces visible after backup.
        if xb.size > 0:
            cy_a = math.cos(self._robot_yaw)
            sy_a = math.sin(self._robot_yaw)
            self._last_all_pts_map = np.stack(
                [self._robot_x + xb * cy_a - yb * sy_a,
                 self._robot_y + xb * sy_a + yb * cy_a], axis=1)
        else:
            self._last_all_pts_map = None

        # Refine stored movable obstacle positions from the current full-360° scan.
        # Only updates via lidar centroid when the obstacle is outside the blind
        # spot. While inside the blind spot (presumed still in contact, being
        # pushed), dead-reckon the tracked position to stay directly ahead of
        # the robot instead of leaving it frozen at the stale classification-time
        # seed -- without this, continued pushing after classification (e.g.
        # toward a goal well past the obstacle) carries the true object position
        # outside movable_excl_radius of the frozen seed, so the object
        # re-emerges as a "new" lidar obstacle mid-push and re-triggers
        # SLOW_APPROACH/CONTACT_PROBE against the very object already
        # classified MOVABLE (sending a fresh probe goal into it).
        if self._movable_obstacles and self._last_all_pts_map is not None:
            pts_map = self._last_all_pts_map
            changed = False
            cy_h = math.cos(self._robot_yaw)
            sy_h = math.sin(self._robot_yaw)
            for i, (mox, moy, moz) in enumerate(self._movable_obstacles):
                if math.hypot(mox - self._robot_x, moy - self._robot_y) < self._cloud_r_min:
                    new_mox = self._robot_x + self._cloud_r_min * cy_h
                    new_moy = self._robot_y + self._cloud_r_min * sy_h
                    if math.hypot(new_mox - mox, new_moy - moy) > 0.01:
                        self._movable_obstacles[i] = (new_mox, new_moy, moz)
                        changed = True
                    continue  # in blind spot -- dead-reckoned above, not lidar-refined
                known = np.array([mox, moy])
                d2 = np.linalg.norm(pts_map - known[np.newaxis, :], axis=1)
                near = pts_map[d2 <= self._movable_track_r]
                if near.shape[0] >= self._movable_track_min:
                    cx, cy = float(near[:, 0].mean()), float(near[:, 1].mean())
                    if math.hypot(cx - mox, cy - moy) > 0.01:
                        self._movable_obstacles[i] = (cx, cy, moz)
                        changed = True
            if changed:
                self._publish_movable_obstacles()

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

        # Exclude confirmed solid and movable obstacles from nearest-obstacle
        # computation so neither re-triggers SLOW_APPROACH.
        if (self._solid_obstacles or self._movable_obstacles) and r.size > 0:
            cyw = math.cos(self._robot_yaw)
            syw = math.sin(self._robot_yaw)
            keep = np.ones(r.size, dtype=bool)
            for sox, soy, _ in self._solid_obstacles:
                dx = sox - self._robot_x
                dy = soy - self._robot_y
                sbx =  dx * cyw + dy * syw
                sby = -dx * syw + dy * cyw
                keep &= (xb - sbx)**2 + (yb - sby)**2 > self._solid_excl_r2
            for mox, moy, _ in self._movable_obstacles:
                dx = mox - self._robot_x
                dy = moy - self._robot_y
                mbx =  dx * cyw + dy * syw
                mby = -dx * syw + dy * cyw
                keep &= (xb - mbx)**2 + (yb - mby)**2 > self._movable_excl_r2
            xb, yb, r = xb[keep], yb[keep], r[keep]
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
        if not all(math.isfinite(v) for v in (p.position.x, p.position.y, p.orientation.z,
                                              p.orientation.w)):
            rospy.logwarn_throttle(2.0, '[ContactProbe] Non-finite pose dropped.')
            return
        self._last_pose_t = rospy.Time.now()
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
        # Projected onto the approach heading (_wrench_forward_mag), a single
        # threshold for both directions -- see contact_force_threshold_n's
        # comment above for why this replaced two independent per-axis (fx,
        # fy) thresholds. fz/torques are still excluded: dominated by
        # walking-gait dynamics, not genuine contact, and including them
        # caused both false CONTACT_PROBE triggers and a SOLID bias in
        # force-mode classification.
        return self._wrench_forward_mag() > self._contact_threshold_n

    def _wrench_forward_mag(self):
        """|Horizontal force along probe_entry_yaw| -- used for force-mode
        classification. fz is deliberately excluded: it's dominated by
        walking-gait bounce (see contact_force_threshold_n's comment) and oscillates
        independently of whether the object actually moved, which would swamp
        the settled-force reading force-mode classification is based on.
        Projecting onto the approach heading (rather than sqrt(fx^2+fy^2)) also
        drops the off-axis fy component, mirroring how displacement mode already
        projects base motion onto the same axis for `forward`.
        """
        return abs(self._avg('fx') * math.cos(self._probe_entry_yaw) +
                   self._avg('fy') * math.sin(self._probe_entry_yaw))

    # ── Speed limit ───────────────────────────────────────────────────────────

    def _set_speed_limit(self, v):
        """Publish a speed cap consumed by the carrot planner. HALTED and
        CALIBRATING always publish 0, whatever a caller asks for."""
        if self._state in (HALTED, CALIBRATING) or math.isnan(v) or v < 0.0:
            v = 0.0
        self._speed_limit_pub.publish(Float64(data=v))

    # ── Solid obstacle cloud ──────────────────────────────────────────────────

    def _publish_obstacles(self):
        # No early-return-on-empty here: publishing a zero-point cloud is how a
        # reset (see _reset_cb) clears the latched topic for late subscribers
        # (e.g. the global costmap's obstacle_layer after a fresh subscribe).
        pts = []
        for obstacle_pts in self._solid_pts:
            pts.extend(obstacle_pts)

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

    def _publish_movable_obstacles(self):
        # Same no-early-return rule as _publish_obstacles -- see comment there.
        pts = [(float(x), float(y), 0.0) for x, y, _ in self._movable_obstacles]
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
        self._movable_pub.publish(cloud)

    def _refine_last_solid(self):
        """Mark the solid obstacle as a filled rectangle in the costmap.

        The front face width is measured from the lidar scan; the depth is
        extruded in the probe approach direction by obstacle_assumed_depth.
        Called at the end of BACKING_UP once the robot is clear.
        Falls back to a provisional ring if no lidar data is available.
        """
        if not self._solid_obstacles:
            return
        cx_known = self._solid_obstacles[-1][0]
        cy_known = self._solid_obstacles[-1][1]

        pts_map = self._last_all_pts_map
        near = np.empty((0, 2))
        if pts_map is not None and pts_map.shape[0] > 0:
            known_pos = np.array([cx_known, cy_known])
            dists = np.linalg.norm(pts_map - known_pos[np.newaxis, :], axis=1)
            near = pts_map[dists <= self._refine_capture_r]

        if near.shape[0] > 0:
            # Orthonormal basis aligned to the approach direction.
            # dx/dy: into the obstacle; px/py: along the face.
            dx =  math.cos(self._probe_entry_yaw)
            dy =  math.sin(self._probe_entry_yaw)
            px = -dy
            py =  dx

            proj_face  = near[:, 0] * px + near[:, 1] * py
            proj_depth = near[:, 0] * dx + near[:, 1] * dy

            depth_front = float(proj_depth.min())
            face_lo = float(proj_face.min())
            face_hi = float(proj_face.max())

            # Dense grid covering front-face width × assumed depth.
            step = 0.05
            face_vals  = np.arange(face_lo - step, face_hi + step * 1.5, step)
            depth_vals = np.arange(depth_front,
                                   depth_front + self._obstacle_depth + step * 0.5,
                                   step)

            pts = [(float(fv * px + dv * dx),
                    float(fv * py + dv * dy), 0.0)
                   for fv in face_vals
                   for dv in depth_vals]

            self._solid_pts[-1] = pts
            rospy.loginfo(
                '[ContactProbe] Marked solid at (%.2f,%.2f) — %d pts '
                '(face %.2fm × depth %.2fm)',
                cx_known, cy_known, len(pts),
                face_hi - face_lo, self._obstacle_depth)
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
        self._state_pub.publish(String(data=new_state))

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
            self._classify_start_pos     = (self._robot_x, self._robot_y)
            self._classify_force_samples = []
            # Probe goal stays active; speed_limit stays at probe_speed.

        elif new_state == BACKING_UP:
            self._send_backup_goal()   # preempts active probe/classify goal implicitly
            self._set_speed_limit(self._backup_speed)

        elif new_state == HALTED:
            self._set_speed_limit(0.0)

    def _elapsed(self):
        return (rospy.Time.now() - self._state_enter_time).to_sec()

    # ── External reset hook ──────────────────────────────────────────────────

    @_synchronized
    def _reset_cb(self, _req):
        """Clear accumulated obstacle memory and return to NORMAL_NAV.

        Does NOT clear the move_base costmaps or Gazebo/estimator state --
        callers (e.g. haptiquad_nav_eval's orchestrator) are expected to
        also call move_base's own /move_base/clear_costmaps and whatever
        Gazebo/state-estimator reset sequence they use, in whichever order
        makes sense for their world reset. This only resets what this node
        itself owns.
        """
        rospy.loginfo('[ContactProbe] Reset requested -- clearing obstacle memory and state.')
        self._halt_reason = ''
        self._mb_client.cancel_all_goals()
        self._solid_obstacles = []
        self._solid_pts = []
        self._movable_obstacles = []
        self._wrench_above_since = None
        self._goal_needs_resend = False
        self._last_goal = None
        self._backup_cooldown_until = rospy.Time(0)
        self._contact_point = None
        self._state = NORMAL_NAV
        self._state_enter_time = rospy.Time.now()
        self._set_speed_limit(float('inf'))
        self._state_pub.publish(String(data=NORMAL_NAV))
        self._publish_obstacles()          # republish now-empty cloud -- clears the latch
        self._publish_movable_obstacles()  # same
        return EmptyResponse()

    @_synchronized
    def _set_probing_enabled_cb(self, req):
        """haptiquad_nav_eval's timing_sweep_orchestrator.py only -- disables
        the entire contact-probe state machine so a "classical" trial measures
        plain move_base navigation (the obstacle is never slowed for/probed/
        pushed). Forces a clean NORMAL_NAV/full-speed state immediately on
        disable so bypass takes effect before the orchestrator's next goal is
        sent, regardless of what state the machine happened to be in."""
        self._probing_enabled = req.data
        if not req.data:
            self._mb_client.cancel_all_goals()
            self._state = NORMAL_NAV
            self._state_enter_time = rospy.Time.now()
            self._set_speed_limit(float('inf'))
            self._state_pub.publish(String(data=NORMAL_NAV))
            self._goal_needs_resend = False
        rospy.loginfo('[ContactProbe] probing_enabled=%s', req.data)
        return SetBoolResponse(success=True, message='probing_enabled=%s' % req.data)

    def _solid_excl_override_cb(self, msg):
        self._solid_excl_r2 = msg.data ** 2

    def _movable_excl_override_cb(self, msg):
        self._movable_excl_r2 = msg.data ** 2

    def _refine_capture_override_cb(self, msg):
        self._refine_capture_r = msg.data

    def _probe_speed_override_cb(self, msg):
        self._probe_speed = msg.data

    def _probe_distance_override_cb(self, msg):
        self._probe_dist = msg.data

    def _contact_hold_time_override_cb(self, msg):
        self._hold_time = msg.data

    def _classify_time_override_cb(self, msg):
        self._classify_time = msg.data

    def _force_settle_time_override_cb(self, msg):
        self._force_settle_time = msg.data

    @_synchronized
    def _filter_window_override_cb(self, msg):
        """wrench_filter_window controls the wrench deques' maxlen, which
        (unlike the other overrides here) can't just be reassigned -- a
        Python deque's maxlen is fixed at construction. Rebuild each deque
        from its own current contents at the new length instead of clearing
        it, so an override mid-run doesn't throw away recent samples for no
        reason."""
        new_win = max(1, int(round(msg.data)))
        if new_win == self._filter_win:
            return
        self._filter_win = new_win
        for k in self._wrench_buf:
            self._wrench_buf[k] = collections.deque(self._wrench_buf[k], maxlen=new_win)

    @_synchronized
    def _inject_known_solid_cb(self, msg):
        """haptiquad_nav_eval's timing_sweep_orchestrator.py only (classical
        arm) -- injects a KNOWN ground-truth rectangle (any orientation, map
        frame) directly into _solid_obstacles/_solid_pts, exactly like a
        real probe's _refine_last_solid() would after classifying SOLID and
        backing up, so the global costmap knows about it a priori and the
        global planner routes around it from the very first plan -- with
        zero CONTACT_PROBE/CLASSIFYING/BACKING_UP ever run. Dense grid built
        the same way _refine_last_solid() builds one, just from the given
        corners instead of a lidar-measured face."""
        pts = np.array([[p.x, p.y] for p in msg.polygon.points], dtype=float)
        if pts.shape[0] < 3:
            rospy.logwarn('[ContactProbe] inject_known_solid: need >=3 polygon '
                          'points, got %d -- ignoring.', pts.shape[0])
            return

        center = pts.mean(axis=0)
        edge = pts[1] - pts[0]
        face_ang = math.atan2(edge[1], edge[0])
        fx, fy = math.cos(face_ang), math.sin(face_ang)
        px, py = -fy, fx
        proj_face = (pts[:, 0] - center[0]) * fx + (pts[:, 1] - center[1]) * fy
        proj_side = (pts[:, 0] - center[0]) * px + (pts[:, 1] - center[1]) * py
        face_lo, face_hi = float(proj_face.min()), float(proj_face.max())
        side_lo, side_hi = float(proj_side.min()), float(proj_side.max())

        step = 0.05
        face_vals = np.arange(face_lo, face_hi + step * 0.5, step)
        side_vals = np.arange(side_lo, side_hi + step * 0.5, step)
        grid_pts = [(float(center[0] + fv * fx + sv * px),
                    float(center[1] + fv * fy + sv * py), 0.0)
                   for fv in face_vals for sv in side_vals]

        self._solid_obstacles.append((float(center[0]), float(center[1]), 0.0))
        self._solid_pts.append(grid_pts)
        self._publish_obstacles()
        rospy.loginfo('[ContactProbe] Injected known solid at (%.2f,%.2f) — %d pts '
                      '(%.2fm × %.2fm)', center[0], center[1], len(grid_pts),
                      face_hi - face_lo, side_hi - side_lo)

    # ── Diagnostics ────────────────────────────────────────────────────────────

    def _publish_diagnostics(self, _event):
        if self._loop_period_buf:
            mean_period = sum(self._loop_period_buf) / len(self._loop_period_buf)
            loop_rate = 1.0 / mean_period if mean_period > 0 else 0.0
        else:
            loop_rate = 0.0
        if self._wrench_latency_buf:
            latency_ms = 1000.0 * sum(self._wrench_latency_buf) / len(self._wrench_latency_buf)
        else:
            latency_ms = 0.0

        status = DiagnosticStatus()
        status.name = 'contact_probe_manager'
        status.level = DiagnosticStatus.ERROR if self._state == HALTED else DiagnosticStatus.OK
        status.message = self._state
        status.values = [
            KeyValue(key='halt_reason',         value=self._halt_reason),
            KeyValue(key='loop_rate_hz',        value='%.2f' % loop_rate),
            KeyValue(key='loop_rate_target_hz', value='%.2f' % (1.0 / self._update_period_target)),
            KeyValue(key='wrench_latency_ms',   value='%.2f' % latency_ms),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = rospy.Time.now()
        arr.status = [status]
        self._diag_pub.publish(arr)

    # ── Safety ───────────────────────────────────────────────────────────────

    def _health_cb(self, msg):
        self._last_health_t = rospy.Time.now()
        self._health_ok = msg.data

    def _fault_cb(self, msg):
        self._halt('fault reported: %s' % msg.data)

    def _input_problem(self):
        """'' if every input this node acts on is fresh and healthy, else why not."""
        now = rospy.Time.now()
        for name, t, timeout in (('wrench', self._last_wrench_t, self._wrench_timeout),
                                 ('pose', self._last_pose_t, self._pose_timeout),
                                 ('point cloud', self._last_cloud_t, self._cloud_timeout)):
            if t is None:
                return 'no %s received yet' % name
            age = (now - t).to_sec()
            if age > timeout:
                return '%s stale (%.2fs > %.2fs)' % (name, age, timeout)
        if self._require_health:
            if self._last_health_t is None:
                return 'no health heartbeat from nav_supervisor'
            age = (now - self._last_health_t).to_sec()
            if age > self._health_timeout:
                return 'nav_supervisor heartbeat stale (%.2fs)' % age
            if not self._health_ok:
                return 'nav_supervisor reports an unhealthy input'
        return ''

    @_synchronized
    def _halt(self, reason):
        """Stop everything: cancel goals, speed 0, motion_allowed False.
        Obstacle memory is kept. Leaving HALTED needs a new user goal."""
        if self._state == HALTED:
            return
        rospy.logerr('[ContactProbe] HALT in %s: %s', self._state, reason)
        if self._state == BACKING_UP:
            self._refine_last_solid()   # don't leave the solid's mark as an empty placeholder
        self._halt_reason = reason
        self._halt_reason_pub.publish(String(data=reason))
        self._mb_client.cancel_all_goals()
        self._wrench_above_since = None
        self._collision_since = None
        self._goal_needs_resend = False
        self._enter(HALTED)

    def _calibration_step(self):
        """End of CALIBRATING: wrench bias = mean standing fx/fy."""
        if self._elapsed() < self._bias_calib_time:
            return
        samples = self._calib_samples
        if len(samples) < 10:
            rospy.logwarn_throttle(5.0, '[ContactProbe] Calibration: only %d wrench samples '
                                   'so far -- waiting for more.', len(samples))
            return
        n = len(samples)
        bias_fx = sum(sm[1] for sm in samples) / n
        bias_fy = sum(sm[2] for sm in samples) / n
        span = samples[-1][0] - samples[0][0]
        rate = (n - 1) / span if span > 0 else 0.0
        self._calib_samples = []
        if math.hypot(bias_fx, bias_fy) > self._max_bias:
            # Robot leaning on something, or the estimate is off: don't adopt
            # it, stay CALIBRATING (no motion) and measure again.
            rospy.logerr('[ContactProbe] Wrench bias (%.1f, %.1f)N exceeds max_wrench_bias_n='
                         '%.1fN -- check the robot and the HaptiQuad estimate; recalibrating.',
                         bias_fx, bias_fy, self._max_bias)
            self._state_enter_time = rospy.Time.now()
            return
        self._bias = {'fx': bias_fx, 'fy': bias_fy}
        if self._filter_win_s > 0 and rate > 0:
            self._filter_win = max(1, int(round(rate * self._filter_win_s)))
        for k in self._wrench_buf:   # drop pre-calibration (unbiased) samples
            self._wrench_buf[k] = collections.deque(maxlen=self._filter_win)
        rospy.loginfo('[ContactProbe] Calibrated: bias fx=%.2fN fy=%.2fN, wrench rate %.0fHz, '
                      'filter window %d samples (%.0fms).', bias_fx, bias_fy, rate,
                      self._filter_win, 1000.0 * self._filter_win / rate if rate else 0.0)
        self._mb_client.cancel_all_goals()
        self._enter(NORMAL_NAV)

    def _near_movable(self):
        r2 = self._movable_excl_r2
        return any((mx - self._robot_x) ** 2 + (my - self._robot_y) ** 2 <= r2
                   for mx, my, _ in self._movable_obstacles)

    def _collision_check(self):
        """True if an unexpected contact just halted the robot."""
        if not self._collision_stop or self._state not in (NORMAL_NAV, SLOW_APPROACH, BACKING_UP):
            self._collision_since = None
            return False
        f = math.hypot(self._avg('fx'), self._avg('fy'))
        if self._state == BACKING_UP and self._elapsed() < self._collision_backup_grace:
            thr = float('inf')   # contact force from the solid still decaying
        elif self._near_movable():
            thr = self._force_threshold_n   # pushing a MOVABLE through is expected contact
        else:
            thr = self._collision_thr
        if f <= thr:
            self._collision_since = None
            return False
        now = rospy.Time.now()
        if self._collision_since is None:
            self._collision_since = now
            return False
        held = (now - self._collision_since).to_sec()
        if held < self._collision_hold:
            return False
        self._halt('unexpected contact in %s: |F_xy|=%.1fN > %.1fN for %.2fs'
                   % (self._state, f, thr, held))
        return True

    def _safety_step(self):
        """Runs every tick before the state machine; returns motion_allowed."""
        if self._state == CALIBRATING:
            self._calibration_step()
            return False
        if self._state == HALTED:
            return False
        if None in (self._last_wrench_t, self._last_pose_t, self._last_cloud_t) or \
                (self._require_health and self._last_health_t is None):
            return False   # not ready yet (startup): no motion, but nothing to halt
        problem = self._input_problem()
        if problem:
            self._halt(problem)
            return False
        return not self._collision_check()

    # ── Main update loop ──────────────────────────────────────────────────────

    def _update_guarded(self, event):
        """An exception escaping a rospy.Timer callback kills that timer's
        thread silently, freezing the state machine with speed_limit latched
        at whatever it last was. Log it and keep the loop alive instead."""
        try:
            self._update(event)
            allowed = self._motion_allowed
        except Exception:
            # Never report motion_allowed from a tick that didn't complete:
            # a state machine raising every tick must close the gate.
            allowed = False
            rospy.logerr_throttle(5.0, '[ContactProbe] _update raised:\n%s' % traceback.format_exc())
        self._motion_allowed_pub.publish(Bool(data=allowed))

    @_synchronized
    def _update(self, _event):
        now = rospy.Time.now()
        if self._last_update_wall is not None:
            self._loop_period_buf.append((now - self._last_update_wall).to_sec())
        self._last_update_wall = now

        self._motion_allowed = self._safety_step()
        if not self._motion_allowed:
            return

        if not self._probing_enabled:
            # Bypass mode (haptiquad_nav_eval timing harness only, see
            # _set_probing_enabled_cb) -- state is already pinned at
            # NORMAL_NAV/speed_limit=inf from the moment it was disabled;
            # skip the state machine and nearest-obstacle telemetry entirely.
            return

        # Published at the fixed 10Hz update rate rather than threaded through
        # _cloud_cb's several early-return branches -- simpler, and a safety
        # margin log doesn't need full lidar rate. Consumed by
        # haptiquad_nav_eval's trial_logger.py to track minimum clearance
        # during BACKING_UP (the safety-critical phase: is the robot getting
        # dangerously close to an unknown rear obstacle while reversing).
        self._nearest_obs_pub.publish(Float64(data=self._nearest_obs))
        self._nearest_rear_obs_pub.publish(Float64(data=self._nearest_rear_obs))

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

            elapsed = self._elapsed()
            # Sample the settled forward force every tick once past the settle
            # window, so the end-of-window decision averages over the steady-state
            # push rather than a single noisy instantaneous reading.
            if elapsed >= self._force_settle_time:
                self._classify_force_samples.append(self._wrench_forward_mag())

            if elapsed >= self._classify_time:
                dx = self._robot_x - self._classify_start_pos[0]
                dy = self._robot_y - self._classify_start_pos[1]
                forward = (dx * math.cos(self._probe_entry_yaw) +
                           dy * math.sin(self._probe_entry_yaw))

                if self._classify_force_samples:
                    w_settled = sum(self._classify_force_samples) / len(self._classify_force_samples)
                else:
                    w_settled = self._wrench_forward_mag()   # settle_time >= classify_time edge case
                movable_threshold = self._probe_speed * self._classify_time * 0.4

                # Ablation toggle (classification_mode param): both metrics are
                # always computed and logged so a run's classify_time samples one
                # criterion while the other is still visible for offline comparison,
                # but only the selected one drives the MOVABLE/SOLID decision.
                if self._classification_mode == 'force':
                    is_movable = w_settled < self._force_threshold_n
                    decision_detail = 'force-mode: w_settled=%.1fN vs thr=%.1fN' % (
                        w_settled, self._force_threshold_n)
                else:
                    is_movable = forward > movable_threshold
                    decision_detail = 'displacement-mode: Δfwd=%.3fm vs %.3fm' % (
                        forward, movable_threshold)

                if is_movable:
                    rospy.loginfo(
                        '[ContactProbe] MOVABLE at (%.2f, %.2f) — %s '
                        '[Δfwd=%.3fm thr=%.3fm w_settled=%.1fN thr=%.1fN]',
                        self._contact_point[0], self._contact_point[1], decision_detail,
                        forward, movable_threshold, w_settled, self._force_threshold_n)
                    # Seed at contact_point + measured push so the tracking
                    # radius covers the actual post-push position immediately.
                    pushed = (
                        self._contact_point[0] + forward * math.cos(self._probe_entry_yaw),
                        self._contact_point[1] + forward * math.sin(self._probe_entry_yaw),
                        0.0)
                    self._movable_obstacles.append(pushed)
                    self._publish_movable_obstacles()
                    # _nearest_obs still holds whatever _cloud_cb last computed
                    # BEFORE this obstacle existed in _movable_obstacles (that
                    # exclusion is only applied inside _cloud_cb itself, an
                    # async callback independent of this 10Hz timer) -- almost
                    # certainly very close, since the robot was just in
                    # contact. Left alone, the very next _update() tick's
                    # NORMAL_NAV branch can see that stale close reading and
                    # re-enter SLOW_APPROACH against the object it just
                    # classified MOVABLE, before the next fresh point cloud
                    # arrives and correctly excludes it. Forcing it to inf
                    # here closes that race instead of waiting on cloud timing.
                    self._nearest_obs = float('inf')
                    self._enter(NORMAL_NAV)
                else:
                    rospy.logwarn(
                        '[ContactProbe] SOLID at (%.2f, %.2f) — %s '
                        '[Δfwd=%.3fm thr=%.3fm w_settled=%.1fN thr=%.1fN]',
                        self._contact_point[0], self._contact_point[1], decision_detail,
                        forward, movable_threshold, w_settled, self._force_threshold_n)
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
    ContactProbeManager().start()
    rospy.spin()
