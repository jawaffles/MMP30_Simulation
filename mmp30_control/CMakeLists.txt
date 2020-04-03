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


add_executable(calibrate_heading src/calibrate_heading.cpp)
target_link_libraries(calibrate_heading ${catkin_LIBRARIES})

add_executable(calibrate_heading_sim src/calibrate_heading_sim.cpp)
target_link_libraries(calibrate_heading_sim ${catkin_LIBRARIES})

add_executable(calibrate_heading_indoor src/calibrate_heading_indoor.cpp)
target_link_libraries(calibrate_heading_indoor ${catkin_LIBRARIES})

