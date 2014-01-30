cmake_minimum_required(VERSION 2.8.3)
project(skynav_gui)

find_package(catkin REQUIRED COMPONENTS skynav_msgs)

catkin_package(
  CATKIN_DEPENDS skynav_msgs
)