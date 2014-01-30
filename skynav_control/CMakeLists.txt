cmake_minimum_required(VERSION 2.8.3)
project(skynav_control)

find_package(catkin REQUIRED COMPONENTS roscpp skynav_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp skynav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motion_control src/motion_control.cpp)
add_executable(data_verifier src/data_verifier.cpp)
add_executable(waypoint_filter src/waypoint_filter.cpp)

target_link_libraries(motion_control ${catkin_LIBRARIES})
target_link_libraries(data_verifier ${catkin_LIBRARIES})
target_link_libraries(waypoint_filter ${catkin_LIBRARIES})

add_dependencies(motion_control skynav_msgs_gencpp)
add_dependencies(data_verifier skynav_msgs_gencpp)
add_dependencies(waypoint_filter skynav_msgs_gencpp)