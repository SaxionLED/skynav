cmake_minimum_required(VERSION 2.8.3)
project(skynav_tests)

find_package(catkin REQUIRED COMPONENTS roscpp tf pcl_ros rostest skynav_msgs )

catkin_package(
  CATKIN_DEPENDS roscpp tf pcl_ros skynav_msgs rostest
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTEST_LIBRARIES})

add_executable(static_PC_publisher src/static_PC_publisher.cpp)
target_link_libraries(static_PC_publisher ${catkin_LIBRARIES})

add_rostest_gtest(test_obstacle_detector launch/test_obstacle_detector.test  src/test_obstacle_detector.cpp)

target_link_libraries(test_obstacle_detector ${catkin_LIBRARIES})
add_dependencies(test_obstacle_detector skynav_msgs_gencpp)

add_rostest_gtest(test_local_planner launch/test_local_planner.test src/test_local_planner.cpp)

target_link_libraries(test_local_planner ${catkin_LIBRARIES})
add_dependencies(test_local_planner skynav_msgs_gencpp)
