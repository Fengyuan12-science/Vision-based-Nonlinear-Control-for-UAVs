cmake_minimum_required(VERSION 3.0.2)
project(vision_lyapunov_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  mavros_msgs
  visualization_msgs 
  image_geometry
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport message_filters mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install runnable scripts (only the ones you actually use now)
catkin_install_python(PROGRAMS
  scripts/min_avoid_node.py
  scripts/takeoff_offboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

