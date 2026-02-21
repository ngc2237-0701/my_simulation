cmake_minimum_required(VERSION 3.0.2)
project(my_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  mavros_msgs
  tf
  tf2_ros
  image_transport
  cv_bridge
  depth_image_proc
  pcl_ros
  rviz
  gazebo_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY worlds models config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/trajectory_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
