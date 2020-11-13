cmake_minimum_required(VERSION 2.8.3)
project(ros2can)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  can_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs can_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(ros2can_node src/ros2can_node.cpp)
target_link_libraries(ros2can_node ${catkin_LIBRARIES})
