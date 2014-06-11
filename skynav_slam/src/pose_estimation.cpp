#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/current_velocity.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;
using namespace geometry_msgs;

ros::ServiceServer servServerCurrentPose, servServerCurrentVelocity;
ros::Publisher pubTraversedPath;

Pose mCurrentPose;
nav_msgs::Path mTraversedPath;

tf::TransformListener* mTransformListener;


bool servServerCurrentPoseCallback(skynav_msgs::current_pose::Request &request, skynav_msgs::current_pose::Response &response)
{

    //response.pose = mCurrentPose;
    
    // using TF to get current pose relative to /map (global frame)

    tf::StampedTransform st;
    
    try
    {
		mTransformListener->lookupTransform("/map", "/base_link", ros::Time(0), st);
	}
    catch (tf::TransformException ex)
    {
		ROS_ERROR("%s", ex.what());
		return false;
	}
	
    response.pose.position.x = st.getOrigin().x();
    response.pose.position.y = st.getOrigin().y();
    response.pose.orientation.z = tf::getYaw(st.getRotation());
    
    //ROS_INFO("current tf pose: %f, %f, %f", response.pose.position.x, response.pose.position.y, response.pose.orientation.z * (180 / M_PI));    

    return true;
}

bool servServerCurrentVelocityCallback(skynav_msgs::current_velocity::Request &request, skynav_msgs::current_velocity::Response &response){
	
	//using TF to get the current speed of the reference frame to the global frame (/map)
	
	Twist tw;
	
	try
    {
		mTransformListener->lookupTwist("/base_link", "/map", ros::Time(0), ros::Duration(1), tw);
	}
    catch (tf::TransformException ex)
    {
		ROS_ERROR("%s", ex.what());
		return false;
	}
	
	response.velocity = tw;
	
	//ROS_INFO("current velocity: %f",response.velocity.linear.x);
	
	return true;
}


void publishTraversedPath()
{
	
	PoseStamped ps;
	
	tf::StampedTransform st;
    
    try
    {
		mTransformListener->lookupTransform("/map", "/base_link", ros::Time(0), st);
	}
    catch (tf::TransformException ex)
    {
		ROS_ERROR("%s", ex.what());
		return;
	}
	
    ps.pose.position.x = st.getOrigin().x();
    ps.pose.position.y = st.getOrigin().y();
    ps.pose.orientation.z = tf::getYaw(st.getRotation());
    
		// add new pose to traversed path
	ps.header.frame_id = "/map";
	ps.header.stamp = ros::Time::now();
	
	mTraversedPath.poses.push_back(ps);
    
    
    pubTraversedPath.publish(mTraversedPath);    
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pose_estimation");

    ros::NodeHandle n("/slam");
    ros::NodeHandle n_control("/control");
    
    // init
    mTraversedPath.header.frame_id = "/map";
    mTraversedPath.header.stamp = ros::Time::now();
    mTransformListener = new tf::TransformListener();
    
    // pubs
    pubTraversedPath = n.advertise<nav_msgs::Path>("traversed_path", 32);

    //services
    servServerCurrentPose = n.advertiseService("current_pose", servServerCurrentPoseCallback);
    servServerCurrentVelocity = n.advertiseService("current_velocity", servServerCurrentVelocityCallback);


    ros::Rate loop_rate(10);    //10hz
    
    while (ros::ok())
    {

        ros::spinOnce();
        
        publishTraversedPath();

        loop_rate.sleep();
    }
    
    delete mTransformListener;

    return 0;
}

