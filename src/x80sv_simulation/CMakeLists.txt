cmake_minimum_required(VERSION 2.8.3)
project(x80sv_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp skynav_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp skynav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simulator src/simulator.cpp)

target_link_libraries(simulator ${catkin_LIBRARIES})

add_dependencies(simulator skynav_msgs_gencpp)