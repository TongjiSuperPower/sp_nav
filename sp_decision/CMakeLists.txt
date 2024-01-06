cmake_minimum_required(VERSION 3.0.2)
project(sp_decision)
# 设置编译选项
option(DEBUG_MODE "Enable debug mode" ON)

# 添加可选的编译宏定义
if(DEBUG_MODE)
    add_compile_definitions(DEBUG_MODE)
endif()


find_package(catkin REQUIRED COMPONENTS
  robot_msg
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sp_decision
#  CATKIN_DEPENDS robot_msg roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  include 
  behaviors 
  executor 
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  decision_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)