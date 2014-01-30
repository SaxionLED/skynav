cmake_minimum_required(VERSION 2.8.3)
project(skynav_globalnav)

find_package(catkin REQUIRED COMPONENTS roscpp skynav_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp skynav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PROJECT_DIR}/include)

add_executable(environment src/environment/environment.cpp)
add_executable(global_planner src/global_planner/global_planner.cpp)

add_library(graph src/global_planner/graph.cpp)
add_library(node src/global_planner/node.cpp)
add_library(map_data src/global_planner/map_data.cpp)
add_library(edge src/global_planner/edge.cpp)
add_library(path_finder src/global_planner/path_finder.cpp)

target_link_libraries(environment ${catkin_LIBRARIES})
target_link_libraries(global_planner ${catkin_LIBRARIES})
target_link_libraries(graph ${catkin_LIBRARIES})

target_link_libraries(graph node map_data edge path_finder)
target_link_libraries(global_planner graph node map_data)

add_dependencies(environment skynav_msgs_gencpp)
add_dependencies(global_planner skynav_msgs_gencpp)
