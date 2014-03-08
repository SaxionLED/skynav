#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/current_pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace geometry_msgs;

bool mFirstDataSkipped = false; // this is because ROS or the X80 (not known which) will often report a change of large degrees that are not correct. This will filter it out

ros::ServiceServer servServerCurrentPose;
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

void subRelativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {

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

    ros::Rate loop_rate(10);    //10hz
    
    while (ros::ok()) {

        ros::spinOnce();
        
        publishTraversedPath();

        loop_rate.sleep();

    }
    
    delete mTransformListener;

    return 0;
}

