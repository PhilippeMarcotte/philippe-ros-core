cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  rospy
  std_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
