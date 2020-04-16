cmake_minimum_required(VERSION 2.8.3)
project(mmp30_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  geometry_msgs
  diagnostic_updater
  gps_common
  roslib
  roslaunch
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs actionlib_msgs actionlib diagnostic_updater sensor_msgs geometry_msgs tf gps_common
   DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})











