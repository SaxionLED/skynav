#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/current_velocity.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

#define SIMULATOR 						true 	// set this to true when using the simulator, false indicates the actual robot with lower specs is used

using namespace std;
using namespace geometry_msgs;

bool mFirstDataSkipped = false; // this is because ROS or the X80 (not known which) will often report a change of large degrees that are not correct. This will filter it out

ros::ServiceServer servServerCurrentPose, servServerCurrentVelocity;
ros::Publisher pubTraversedPath;

Pose mCurrentPose;
nav_msgs::Path mTraversedPath;

tf::TransformListener* mTransformListener;

void publishCurrentPoseTF() {

    
    // tf current pose
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(mCurrentPose.position.x, mCurrentPose.position.y, mCurrentPose.position.z));      // derp ROS... must convert Point to Vector3 manually, even though they are *exactly* the same
    transform.setRotation(tf::Quaternion(mCurrentPose.orientation.x, mCurrentPose.orientation.y, mCurrentPose.orientation.z)); // same as above... geometry_msgs::Quaternion and tf::Quaternion... derp
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));
}



bool servServerCurrentPoseCallback(skynav_msgs::current_pose::Request &request, skynav_msgs::current_pose::Response &response) {

    //response.pose = mCurrentPose;
    
    // using TF to get current pose relative to /map (global frame)

    tf::StampedTransform st;
    
    try {
		mTransformListener->lookupTransform("/map", "/base_link", ros::Time(0), st);
	}	catch (tf::TransformException ex)	{
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
	
	try{
		mTransformListener->lookupTwist("/base_link", "/map", ros::Time(0), ros::Duration(1), tw);
	}catch (tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		return false;
	}
	
	response.velocity = tw;
	
	//ROS_INFO("current velocity: %f",response.velocity.linear.x);
	
	return true;
}

void subRelativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {		// this is currently only called by the actual robot, not simulator

    if (mFirstDataSkipped) {
        if (msg->position.x != 0.0 || msg->position.y != 0.0 || msg->orientation.z != 0.0) {

            double distance = msg->position.x;
            double angle = msg->orientation.z;

            double x = mCurrentPose.position.x;
            double y = mCurrentPose.position.y;
            double theta = mCurrentPose.orientation.z;

            // recalculate pose
            theta += angle;
            x += cos(theta) * distance;
            y += sin(theta) * distance;


            // save new pose
            mCurrentPose.position.x = floor(x * 1000) / 1000;    // truncate to mm
            mCurrentPose.position.y = floor(y * 1000) / 1000;
            mCurrentPose.orientation.z = theta; // dont truncate theta
            
            // add new pose to traversed path
            PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.header.stamp = ros::Time::now();
            ps.pose = mCurrentPose;
            
            mTraversedPath.poses.push_back(ps);
            
            publishCurrentPoseTF();

            ROS_INFO("pose updated! x: %f, y: %f, theta: %.2f degrees", mCurrentPose.position.x, mCurrentPose.position.y, (mCurrentPose.orientation.z * 180 / M_PI));

        }
    } else {
        mFirstDataSkipped = true;
    }

}


void publishTraversedPath()     {
	
	PoseStamped ps;
	
	tf::StampedTransform st;
    
    try {
		mTransformListener->lookupTransform("/map", "/base_link", ros::Time(0), st);
	}	catch (tf::TransformException ex)	{
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

int main(int argc, char **argv) {

    ros::init(argc, argv, "pose_estimation");

    ros::NodeHandle n("/slam");
    ros::NodeHandle n_control("/control");
    
    // init
    mTraversedPath.header.frame_id = "/map";
    mTraversedPath.header.stamp = ros::Time::now();
    mTransformListener = new tf::TransformListener();
    
    
    // pubs
    pubTraversedPath = n.advertise<nav_msgs::Path>("traversed_path", 32);

    //subs
    ros::Subscriber subRelativePose = n_control.subscribe("odometry", 1024, subRelativePoseCallback);

    //services
    servServerCurrentPose = n.advertiseService("current_pose", servServerCurrentPoseCallback);
    servServerCurrentVelocity = n.advertiseService("current_velocity", servServerCurrentVelocityCallback);


    ros::Rate loop_rate(10);    //10hz
    
    while (ros::ok()) {

        ros::spinOnce();
        
        if(!SIMULATOR){
			publishCurrentPoseTF(); //only use with real robot. in the simulator this function causes massive errors
		}
        publishTraversedPath();

        loop_rate.sleep();

    }
    
    delete mTransformListener;

    return 0;
}

