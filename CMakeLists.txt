cmake_minimum_required(VERSION 3.0.2)
project(haptiquad_nav)

find_package(catkin REQUIRED COMPONENTS
  move_base
  nav_core
  costmap_2d
  navfn
  base_local_planner
  dwa_local_planner
  map_server
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  rospy
)

catkin_package()

###########
## Install
###########

# Launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Maps
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

# Python nodes
catkin_install_python(PROGRAMS
  src/contact_probe_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)