cmake_minimum_required(VERSION 2.8.3)
project(skynav_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs nav_msgs sensor_msgs)

add_message_files(
  FILES
  Object.msg
  Objects.msg
  Point32Stamped.msg
  Pose2DArray.msg
  RangeArray.msg
  RangeDefined.msg
  RangeDefinedArray.msg
  TimedPose.msg
  environment_info.msg
  navigation_state.msg
  user_init.msg
  user_input_mapInit.msg
  user_input_query.msg
)

add_service_files(
  FILES
  current_pose.srv
  edit_fixedWPs_srv.srv
  mapreader_srv.srv
  path_query_srv.srv
  waypoint_check.srv
  roadmap_vars_srv.srv
  environment_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
	CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs sensor_msgs
)
