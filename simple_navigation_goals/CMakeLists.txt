cmake_minimum_required(VERSION 2.8.3)
project(simple_navigation_goals)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(gps_waypoints src/gps_waypoints.cpp)
target_link_libraries(gps_waypoints ${catkin_LIBRARIES})

add_executable(sim_gps_waypoints src/sim_gps_waypoints.cpp)
target_link_libraries(sim_gps_waypoints ${catkin_LIBRARIES})

add_executable(odom_waypoints src/odom_waypoints.cpp)
target_link_libraries(odom_waypoints ${catkin_LIBRARIES})

add_executable(indoor_waypoints src/indoor_waypoints.cpp)
target_link_libraries(indoor_waypoints ${catkin_LIBRARIES})




