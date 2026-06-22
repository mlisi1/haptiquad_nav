cmake_minimum_required(VERSION 3.0.2)
project(haptiquad_nav)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  geometry_msgs
  move_base
  nav_core
  nav_msgs
  navfn
  base_local_planner
  dwa_local_planner
  map_server
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  move_base_msgs
  actionlib
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES haptiquad_nav_planner
  CATKIN_DEPENDS
    angles costmap_2d geometry_msgs nav_core nav_msgs
    pluginlib roscpp tf2 tf2_ros tf2_geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(haptiquad_nav_planner
  src/haptiquad_nav_planner.cpp
)
target_link_libraries(haptiquad_nav_planner ${catkin_LIBRARIES})

###########
## Install
###########

install(TARGETS haptiquad_nav_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES haptiquad_nav_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
catkin_install_python(PROGRAMS
  scripts/wrench_bridge.py
  scripts/odom_bridge.py
  scripts/contact_probe_manager.py
  scripts/twist_stamper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
