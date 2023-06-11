cmake_minimum_required(VERSION 3.0.2)
project(sp_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  move_base_msgs
  actionlib
  tf2_ros
  tf2_geometry_msgs
  base_local_planner
  nav_core
  costmap_2d
  pluginlib
  robot_msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(navigation_model src/navigation_model.cpp)
target_link_libraries(navigation_model 
                                ${catkin_LIBRARIES})                              
add_library(sp_local_planner src/sp_local_planner.cpp)
target_link_libraries(sp_local_planner 
                                ${catkin_LIBRARIES})  

