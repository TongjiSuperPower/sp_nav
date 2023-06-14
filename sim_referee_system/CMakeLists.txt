cmake_minimum_required(VERSION 3.0.2)
project(sim_referee_system)
find_package(catkin REQUIRED COMPONENTS
  robot_msg
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_referee_system
#  CATKIN_DEPENDS robot_msg roscpp
#  DEPENDS system_lib
)
include_directories(
  include/sim_referee_system
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/fake_info_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)