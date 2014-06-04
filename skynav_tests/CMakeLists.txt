cmake_minimum_required(VERSION 2.8.3)
project(skynav_tests)

find_package(catkin REQUIRED COMPONENTS roscpp tf pcl_ros rostest )

catkin_package(
  CATKIN_DEPENDS roscpp tf pcl_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(static_PC_publisher src/static_PC_publisher.cpp)
target_link_libraries(static_PC_publisher ${catkin_LIBRARIES})

catkin_add_gtest(test_obstacle_detector src/test_obstacle_detector.cpp)
target_link_libraries(test_obstacle_detector ${catkin_LIBRARIES})

catkin_add_gtest(test_local_planner src/test_local_planner.cpp)
target_link_libraries(test_local_planner ${catkin_LIBRARIES})
