cmake_minimum_required(VERSION 2.8.3)
project(gazebo_cleaner)

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


add_executable(gazebo_cleaner_node src/robot_cleaner.cpp)
target_link_libraries (gazebo_cleaner_node ${catkin_LIBRARIES})





