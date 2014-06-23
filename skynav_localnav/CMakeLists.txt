cmake_minimum_required(VERSION 2.8.3)
project(skynav_localnav)

find_package(catkin REQUIRED COMPONENTS roscpp skynav_msgs laser_geometry tf pcl_ros rostest )

include_directories(src)

catkin_package(
  CATKIN_DEPENDS roscpp skynav_msgs laser_geometry tf pcl_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(local_planner src/local_planner.cpp src/local_planner_lib.cpp)
add_executable(obstacle_detector src/obstacle_detector.cpp src/obstacle_detector_lib.cpp)

target_link_libraries(local_planner ${catkin_LIBRARIES})
target_link_libraries(obstacle_detector ${catkin_LIBRARIES})

add_dependencies(local_planner skynav_msgs_gencpp)
add_dependencies(obstacle_detector skynav_msgs_gencpp)

catkin_add_gtest(localnav_test test/test_localplanner.cpp src/local_planner_lib.cpp)
target_link_libraries(localnav_test ${catkin_LIBRARIES})

catkin_add_gtest(obstacle_detector_test test/test_obstacle_detector.cpp src/obstacle_detector_lib.cpp)
target_link_libraries(obstacle_detector_test ${catkin_LIBRARIES})
