#!/bin/bash
# Record a hardware session for offline analysis.
#   rosrun haptiquad_nav record_robot.sh push [name]   manual pushes (no nav stack)
#   rosrun haptiquad_nav record_robot.sh nav  [name]   autonomous runs (full stack)
# Bags go to the current directory as <name>_<date>.bag (rosbag adds the date).
set -e
mode=${1:-}; name=${2:-$mode}
base=(/estimated_forces /state_estimator/anymal_state /tf /tf_static
      /lidar/point_cloud /twist_mux/twist /odom /diagnostics)
case "$mode" in
  push) topics=("${base[@]}") ;;
  nav)  topics=("${base[@]}" /slam/pose /anymal/base_wrench /rosout
                /haptiquad_nav/state /haptiquad_nav/speed_limit
                /haptiquad_nav/motion_allowed /haptiquad_nav/halt_reason
                /haptiquad_nav/health /haptiquad_nav/supervisor_status
                /haptiquad_nav/solid_obstacles /haptiquad_nav/movable_obstacles
                /haptiquad_nav/nearest_obstacle_distance
                /move_base/goal /move_base/status /move_base/result
                /move_base/GlobalPlanner/plan /move_base_simple/goal) ;;
  *) echo "usage: $(basename "$0") push|nav [name]" >&2; exit 1 ;;
esac
exec rosbag record -o "$name" "${topics[@]}"
