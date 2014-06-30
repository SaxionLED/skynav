#include <ros/ros.h>
#include <gtest/gtest.h>
#include <stdio.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <skynav_msgs/current_pose.h>
#include <std_msgs/UInt8.h>

#include <boost/thread/mutex.hpp>


using namespace std;

ros::Publisher pubPath;
ros::Subscriber subNavigationState;
ros::ServiceClient servClientCurrentPose;

uint8_t mControl_NavigationState;				//the state which motion_control is in.
boost::mutex mMutex;

bool mNav_stop;

//request the current pose
geometry_msgs::Pose getCurrentPose() 
{
	try{
		geometry_msgs::Pose currentPose;

		skynav_msgs::current_pose poseService;

		if (servClientCurrentPose.call(poseService)) 
		{
			currentPose = poseService.response.pose;

		} else {
			ROS_ERROR("Failed to call current_pose service from skynav_system_tests");
			ros::shutdown();
		}
		return currentPose;
	}
	catch(exception& e)
	{
		ROS_ERROR("exception caught: %s",e.what());
		ros::shutdown();
	}
}


void subNavigationStateCallback(const std_msgs::UInt8& msg )
{
	mControl_NavigationState = msg.data;
	if(mControl_NavigationState == 8)
	{
		mNav_stop = true;
	}	
}

double calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
	return sqrt(pow((a.position.x - b.position.x),2) + pow((a.position.y - b.position.y),2));
}

bool validate_location(geometry_msgs::Pose targetPose)
{
	geometry_msgs::Pose currentPose = getCurrentPose();
	
	if(calcDistance(currentPose, targetPose) < 0.1)
	{
		ROS_INFO("target reached, location is: ( %f,%f )", currentPose.position.x, currentPose.position.y);
		return true;
	}	
	ROS_ERROR("target not reached, location is: ( %f,%f )", currentPose.position.x, currentPose.position.y);
	return false;
}

TEST(Skynav_systemTestSuite, DISABLED_drive_2m_straight)
{
	nav_msgs::Path path;
	geometry_msgs::PoseStamped ps, start, target;
	ps.header.frame_id  = "/map";
	ps.header.stamp = ros::Time::now();	
	
	{	//start
		start.pose.position.x = 0;
		start.pose.position.y = 0;
		start.pose.orientation.z = 0;	
		start.header = ps.header;
		path.poses.push_back(start);
	}
	{	//target
		target.pose.position.x = 2;
		target.pose.position.y = 0;
		target.pose.orientation.z = 180;	
		target.header = ps.header;
		path.poses.push_back(target);
	}
	
	pubPath.publish(path);
	ros::Rate loop_rate(2);	
	while(!mNav_stop)
	{
		ros::spinOnce();
		loop_rate.sleep();		
		
	}
	mNav_stop = false;

	EXPECT_TRUE(validate_location(target.pose));
	
}
TEST(Skynav_systemTestSuite, DISABLED_drive_2m_straight_return)
{
	nav_msgs::Path path;
	path.header.frame_id = "/map";
	geometry_msgs::PoseStamped ps, start, target;
	ps.header.frame_id  = "/map";
	ps.header.stamp = ros::Time::now();	
	
	{	//start
		start.pose.position.x = 0;
		start.pose.position.y = 0;
		start.pose.orientation.z = 0;	
		start.header = ps.header;
		path.poses.push_back(start);
	}
	{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
	{	//target
		target.pose.position.x = 0;
		target.pose.position.y = 0;
		target.pose.orientation.z = 180;	
		target.header = ps.header;
		path.poses.push_back(target);
	}
	
	pubPath.publish(path);	
	
	ros::Rate loop_rate(2);	
	while(!mNav_stop)
	{
		ros::spinOnce();
		loop_rate.sleep();		
		
	}
	mNav_stop = false;

	EXPECT_TRUE(validate_location(target.pose));
	
}
TEST(Skynav_systemTestSuite, drive_4m_square)
{
	nav_msgs::Path path;
	path.header.frame_id = "/map";
	geometry_msgs::PoseStamped ps, start, target;
	ps.header.frame_id  = "/map";
	ps.header.stamp = ros::Time::now();	
	
	{	//start
		start.pose.position.x = 0;
		start.pose.position.y = 0;
		start.pose.orientation.z = 0;	
		start.header = ps.header;
		path.poses.push_back(start);
	}
	{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
	{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = 2;
		path.poses.push_back(ps);
	}
	{	//waypoint
		ps.pose.position.x = -2;
		ps.pose.position.y = 2;
		path.poses.push_back(ps);
	}
	{	//waypoint
		ps.pose.position.x = -2;
		ps.pose.position.y = -2;
		path.poses.push_back(ps);
	}
	{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = -2;
		path.poses.push_back(ps);
	}
	{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
	{	//target
		target.pose.position.x = 0;
		target.pose.position.y = 0;
		target.pose.orientation.z = 180;	
		target.header = ps.header;
		path.poses.push_back(target);
	}
	
	pubPath.publish(path);	
	
	ros::Rate loop_rate(2);	
	while(!mNav_stop)
	{
		ros::spinOnce();
		loop_rate.sleep();		
		
	}
	mNav_stop = false;

	EXPECT_TRUE(validate_location(target.pose));
	
}
TEST(Skynav_systemTestSuite, DISABLED_drive_path)
{
	nav_msgs::Path path;
	path.header.frame_id = "/map";

	geometry_msgs::PoseStamped ps, start, target;
	ps.header.frame_id  = "/map";
	ps.header.stamp = ros::Time::now();	
	
	{	//start
		start.pose.position.x = 0;
		start.pose.position.y = 0;
		start.pose.orientation.z = 0;	
		start.header = ps.header;
		path.poses.push_back(start);
	}
		{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 4;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 5;
		ps.pose.position.y = -0.75;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 6.5;
		ps.pose.position.y = -2;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 5;
		ps.pose.position.y = -4;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 4;
		ps.pose.position.y = -5;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 2;
		ps.pose.position.y = -5;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 0;
		ps.pose.position.y = -5;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = 0;
		ps.pose.position.y = -2;
		path.poses.push_back(ps);
	}
	{	//target
		target.pose.position.x = 0;
		target.pose.position.y = 0;
		target.pose.orientation.z = 180;	
		target.header = ps.header;
		path.poses.push_back(target);
	}
	
	pubPath.publish(path);	
	
	ros::Rate loop_rate(2);	
	while(!mNav_stop)
	{
		ros::spinOnce();
		loop_rate.sleep();		
		
	}
	mNav_stop = false;

	EXPECT_TRUE(validate_location(target.pose));
	
}	
TEST(Skynav_systemTestSuite, DISABLED_drive_path_2)
{
	nav_msgs::Path path;
	path.header.frame_id = "/map";
	geometry_msgs::PoseStamped ps, start, target;
	ps.header.frame_id  = "/map";
	ps.header.stamp = ros::Time::now();	
	
	{	//start
		start.pose.position.x = 0;
		start.pose.position.y = 0;
		start.pose.orientation.z = 0;	
		start.header = ps.header;
		path.poses.push_back(start);
	}
		{	//waypoint
		ps.pose.position.x = -2;
		ps.pose.position.y = -2;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -3;
		ps.pose.position.y = -0.5;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -5;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -7;
		ps.pose.position.y = 0;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -7;
		ps.pose.position.y = 2;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -4;
		ps.pose.position.y = 5.75;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -2;
		ps.pose.position.y = 6.2;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -2;
		ps.pose.position.y = 4;
		path.poses.push_back(ps);
	}
		{	//waypoint
		ps.pose.position.x = -1;
		ps.pose.position.y = 3;
		path.poses.push_back(ps);
	}
	{	//target
		target.pose.position.x = 0;
		target.pose.position.y = 0;
		target.pose.orientation.z = 180;	
		target.header = ps.header;
		path.poses.push_back(target);
	}
	
	pubPath.publish(path);	
	
	ros::Rate loop_rate(2);	
	while(!mNav_stop)
	{
		ros::spinOnce();
		loop_rate.sleep();		
		
	}
	mNav_stop = false;

	EXPECT_TRUE(validate_location(target.pose));
	
	
}
TEST(Skynav_systemTestSuite, DISABLED_Test6)
{
	//pass
	
}
TEST(Skynav_systemTestSuite, DISABLED_Test7)
{
	//pass
	
}
TEST(Skynav_systemTestSuite, DISABLED_Test8)
{
	//pass
	
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "Skynav full system tests");
	ROS_INFO("started skynav tests");

	ros::NodeHandle n("");
	ros::NodeHandle n_localnav("/localnav");
	ros::NodeHandle n_control("/control");
	ros::NodeHandle n_slam("/slam");
    ros::NodeHandle n_globalnav("/globalnav");

	subNavigationState = n_control.subscribe("navigation_state", 0, subNavigationStateCallback);
	
	pubPath = n_globalnav.advertise<nav_msgs::Path>("waypoints", 1);
	
	servClientCurrentPose = n_slam.serviceClient<skynav_msgs::current_pose>("current_pose");
	
	mControl_NavigationState = 0;
	mNav_stop = false;
	
	ros::Duration(20).sleep();
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		RUN_ALL_TESTS();
		
		ros::Duration(5).sleep();
		
		ros::shutdown();
	}
	return 0;
}
