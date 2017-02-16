cmake_minimum_required(VERSION 2.8.3)
project(self_navi)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  message_generation
  nav_msgs
  kobuki_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_cleaner
#  CATKIN_DEPENDS roscpp rospy std_mesgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(gazebo_follow_node src/robot_cleaner.cpp)
target_link_libraries (gazebo_follow_node ${catkin_LIBRARIES})





