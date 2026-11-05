#!/bin/bash
# Record a hardware session with a named topic profile.
#
#   rosrun haptiquad_nav record_robot.sh <profile> <name> [extra topics...]
#   rosrun haptiquad_nav record_robot.sh list
#
# Profiles (each includes the one above it):
#   haptiquad  estimator only: its input and outputs, enough to replay
#              HaptiQuad offline and check rate / noise / bias
#   push       + lidar, commands, odometry: manual pushes, no nav stack
#   nav        + localization and haptiquad_nav state machine / move_base:
#              autonomous runs
#   debug      + costmaps and plans: planner debugging (large bags)
#
# The bag is written as <name>_<date>.bag in $BAG_DIR (default: current dir).
# DRY_RUN=1 prints the rosbag command instead of running it.
set -e

haptiquad=(/state_estimator/anymal_state /estimated_forces /residuals
           /residual_errors /tf /tf_static /diagnostics)
push=("${haptiquad[@]}" /lidar/point_cloud /twist_mux/twist /odom)
nav=("${push[@]}" /slam/pose /anymal/base_wrench /rosout
     /haptiquad_nav/state /haptiquad_nav/speed_limit
     /haptiquad_nav/motion_allowed /haptiquad_nav/halt_reason
     /haptiquad_nav/health /haptiquad_nav/supervisor_status
     /haptiquad_nav/solid_obstacles /haptiquad_nav/movable_obstacles
     /haptiquad_nav/nearest_obstacle_distance
     /haptiquad_nav/nearest_rear_obstacle_distance
     /move_base_simple/goal /move_base/goal /move_base/status /move_base/result
     /move_base/GlobalPlanner/plan /haptiquad_nav/cmd_vel_raw)
debug=("${nav[@]}" /move_base/global_costmap/costmap
       /move_base/global_costmap/costmap_updates
       /move_base/local_costmap/costmap /haptiquad_nav/debug_costmap)

usage() {
    echo "usage: $(basename "$0") <haptiquad|push|nav|debug> <name> [extra topics...]" >&2
    echo "       $(basename "$0") list" >&2
    exit 1
}

profile=${1:-}
if [ "$profile" = list ]; then
    for p in haptiquad push nav debug; do
        declare -n t=$p
        echo "$p (${#t[@]} topics):"; printf '  %s\n' "${t[@]}"
    done
    exit 0
fi
case "$profile" in haptiquad|push|nav|debug) ;; *) usage ;; esac
name=${2:-}
[ -n "$name" ] || usage
shift 2
declare -n topics=$profile

cmd=(rosbag record -o "${BAG_DIR:-.}/$name" "${topics[@]}" "$@")
if [ -n "$DRY_RUN" ]; then echo "${cmd[@]}"; exit 0; fi
echo "[record_robot] profile '$profile' (${#topics[@]} topics$([ $# -gt 0 ] && echo " + $#")) -> ${BAG_DIR:-.}/${name}_<date>.bag"
exec "${cmd[@]}"
