#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/current_pose.h>
#include <tf/transform_broadcaster.h>
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

bool servServerCurrentPoseCallback(skynav_msgs::current_pose::Request &request, skynav_msgs::current_pose::Response &response) {

    response.pose = mCurrentPose;

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

            ROS_INFO("pose updated! x: %f, y: %f, theta: %.2f degrees", mCurrentPose.position.x, mCurrentPose.position.y, (mCurrentPose.orientation.z * 180 / M_PI));

        }
    } else {
        mFirstDataSkipped = true;
    }

}

void publishCurrentPoseTF() {

    
    // tf current pose
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(mCurrentPose.position.x, mCurrentPose.position.y, mCurrentPose.position.z));      // derp ROS... must convert Point to Vector3 manually, even though they are *exactly* the same
    transform.setRotation(tf::Quaternion(mCurrentPose.orientation.x, mCurrentPose.orientation.y, mCurrentPose.orientation.z)); // same as above... geometry_msgs::Quaternion and tf::Quaternion... derp
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));
}

void publishTraversedPath()     {
    
    
    pubTraversedPath.publish(mTraversedPath);    
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pose_estimation");

    ros::NodeHandle n("/SLAM");
    ros::NodeHandle n_control("/control");
    
    // init
    mTraversedPath.header.frame_id = "/map";
    mTraversedPath.header.stamp = ros::Time::now();
    
    // pubs
    pubTraversedPath = n.advertise<nav_msgs::Path>("traversed_path", 32);

    //subs
    ros::Subscriber subRelativePose = n_control.subscribe("odometry", 1024, subRelativePoseCallback);

    //services
    servServerCurrentPose = n.advertiseService("current_pose", servServerCurrentPoseCallback);

    ros::Rate loop_rate(10);    //10hz

    while (ros::ok()) {

        ros::spinOnce();
        
        publishCurrentPoseTF();
        publishTraversedPath();

        loop_rate.sleep();

    }

    return 0;
}

