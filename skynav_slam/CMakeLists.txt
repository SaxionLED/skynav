cmake_minimum_required(VERSION 2.8.3)
project(skynav_slam)

find_package(catkin REQUIRED COMPONENTS roscpp skynav_msgs tf)

catkin_package(
  CATKIN_DEPENDS roscpp skynav_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(map_reader src/map_reader.cpp)
add_executable(pose_estimation src/pose_estimation.cpp)

target_link_libraries(map_reader ${catkin_LIBRARIES})
target_link_libraries(pose_estimation ${catkin_LIBRARIES})

add_dependencies(map_reader skynav_msgs_gencpp)
add_dependencies(pose_estimation skynav_msgs_gencpp)