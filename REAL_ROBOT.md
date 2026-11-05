# Running on the real ANYmal C

Branch `real_robot`. Launch with

```bash
roslaunch haptiquad_nav navigation.launch            # platform:=robot, use_sim_time:=false
```

In simulation, use `platform:=sim use_sim_time:=true` (plus `eval_mode:=true` for
the `haptiquad_nav_eval` sweeps; `tools/run_sweep.sh` passes all three).

## Startup sequence

1. `navigation.launch` starts only `odom_bridge`, `wrench_bridge`,
   `anymal_motion_init` (e-stop clear + trekker/walk, immediately) and
   `nav_supervisor`.
2. `nav_supervisor` runs the preflight in `config/nav_supervisor.yaml`.
   - Every listed topic must publish at its minimum rate, with fresh and finite data.
   - The point cloud must not be empty.
   - `map→base` and `odom→base` TF must be available and recent.
   - `anymal_motion_init` must have reached walk mode, meaning the
     `execute_motion` action returned SUCCEEDED (within 30 s). If it doesn't,
     navigation never starts.

   All of this must hold continuously for 3 s. Until then nothing that can
   move the robot is running. The current reason is in
   `/haptiquad_nav/supervisor_status`.
3. It then starts `navigation_core.launch`: move_base, `contact_probe_manager`,
   `cmd_vel_gate`, `goal_relay`, `debug_costmap`, and rviz if requested.
4. `contact_probe_manager` stays in `CALIBRATING` for
   `wrench_bias_calibration_s` (5 s): the standing base-wrench bias (in base
   frame, so it stays valid after the robot turns) and the
   wrench rate are measured, and goals are rejected meanwhile. Samples only
   count while `/odom` says the base is still (`calibration_max_speed`,
   `calibration_max_yaw_rate`); moving it, e.g. with the joystick, restarts
   the 5 s. The log line `Calibrated: bias ... wrench rate ...` gives the
   numbers to check.

## What stops the robot

Velocity commands reach twist_mux only through `cmd_vel_gate`. It forwards a
command only while all of these hold:

| Condition | Source |
|---|---|
| `/haptiquad_nav/health` is fresh and True | `nav_supervisor`: all inputs keep passing the preflight checks; move_base status and the probe manager heartbeat are alive |
| `/haptiquad_nav/motion_allowed` is fresh and True | `contact_probe_manager`: calibrated, not HALTED, its own wrench/pose/cloud inputs fresh |
| `/odom` is fresh | needed for stuck detection |
| no latched fault | stuck controller, or a non-finite command |

When the gate closes, it sends one zero twist and then publishes nothing.
If the gate itself dies, nothing is published and twist_mux times out.

`contact_probe_manager` goes to **HALTED** in these cases:
- an input goes stale;
- the supervisor reports a problem;
- unexpected contact (`|F_xy|` > `collision_force_threshold_n` for
  `collision_hold_time` while not probing; the threshold is
  `classification_force_threshold_n` while pushing a known movable);
- `cmd_vel_gate` reports a fault.

HALTED cancels all goals and sets `speed_limit=0`. The reason is on
`/haptiquad_nav/halt_reason` (latched). Only a new goal resumes navigation,
and only once everything is healthy again.

Crashes: every core node is `required`, so a crash takes the core down.
`nav_supervisor` then reports health False permanently, and the stack must be
relaunched. If `nav_supervisor` itself dies, the whole launch goes down.

The joystick (twist_mux priority 20) and the hardware e-stop stay with the
operator and are independent of all of the above.

## Before the first tethered test

- `config/nav_supervisor.yaml`: measure real rates with `rostopic hz` and
  tighten the minimums (they are conservative placeholders). Check the topic
  names (`/lidar/point_cloud`, `/slam/pose`) on the robot.
- twist_mux on the robot: `cmd_vel_topic` must rank below the joystick and
  must not be shared with the native path follower.
- `config/contact_probe_robot.yaml`:
  - Check the lidar band (`cloud_height_min`, gravity-referenced) and
    `self_filter_box` against real returns in rviz, standing and walking.
    `probe_distance` (0.70), `slow_approach_distance` (0.90) and
    `backup_min_clearance` (0.70) must stay outside the box (±0.55 m),
    or obstacles vanish from the cloud right when they matter.
  - Re-derive `contact_force_threshold_n` (12 N) and
    `classification_force_threshold_n` (100 N). Both come from Gazebo with
    a generic URDF and no joint friction. First record the wrench noise
    floor standing and walking.
  - `collision_force_threshold_n` is 40 N on the robot: walking at 0.4-0.6 m/s
    peaked at 27.8 N in the 2026-11-05 bags, 2 N under the old 30 N.
- Probe timing (`probe_speed` 0.4, `contact_hold_time` 0.4, `classify_time` 0.6)
  comes from the Gazebo tuning sweep (`tuning_sweep_02`), not from hardware. For
  the first contact tests, a slower probe is the safer start and was equally
  accurate in that sweep: `probe_speed: 0.3` with `contact_hold_time: 0.6`
  gave 100% SOLID accuracy for every classify/settle value. Put both in
  `contact_probe_robot.yaml` if you want it. Below 0.3 m/s the 100 N
  classification misses solids (force still ramping when the window closes).
  On hardware that showed at 0.05-0.2 m/s (2026-11-05: the same box read
  59-118 N in the window depending on speed), so the robot config sets
  `classification_decision: event`: SOLID as soon as the force reaches the
  threshold, MOVABLE only once the robot advances again (>= half its approach
  speed) with the force below the threshold and no longer rising, SOLID after
  `classification_timeout` (4 s). Replayed on those bags it gives SOLID for
  all four pushes at any speed, and MOVABLE for a light sliding object under
  real walking noise. Simulation keeps `window`, as in the paper runs.
- Backing up always ends on `backup_time` in simulation (≈1.40 m from the
  solid, target 1.60 m): the planner's `slow_down_dist` (1.5 m) is longer than
  the backup. Expected; on the robot that also keeps the reverse slow.
- The global costmap is a 40×40 m rolling window (no map_server), so goals
  must be within ±20 m.
- The HaptiQuad model (`haptiquad_ros`) still uses the generic
  `anymal_c_simple_description` URDF with friction off. The bias calibration
  removes a constant offset, not model errors that depend on motion.

## Recording sessions

`rosrun haptiquad_nav record_robot.sh <profile> <name> [extra topics...]`
writes `<name>_<date>.bag` (in `$BAG_DIR`, default the current directory).
`record_robot.sh list` prints every topic per profile; `DRY_RUN=1` prints the
command only. Each profile includes the one before it:

| Profile | Adds | Use for |
|---|---|---|
| `haptiquad` | `anymal_state`, `/estimated_forces`, residuals, TF, diagnostics | estimator rate / noise / bias, offline replay |
| `push` | lidar, `/twist_mux/twist`, `/odom` | manual pushes without the nav stack |
| `nav` | `/slam/pose`, `/rosout`, `/haptiquad_nav/*` state, move_base goal/status/plan | autonomous runs |
| `debug` | global/local/debug costmaps | planner problems (large bags) |
