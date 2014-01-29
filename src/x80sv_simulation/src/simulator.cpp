#include <ros/ros.h>
#include <skynav_msgs/TimedPose.h>
#include <skynav_msgs/RangeArray.h>
#include <geometry_msgs/Pose.h>


ros::Publisher pubOdometry;


void subTargetPoseCallback(const skynav_msgs::TimedPose::ConstPtr& targetPose) {

    // add optional delay here
    
    pubOdometry.publish(targetPose->pose);

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "simulator");

    ros::NodeHandle n("/robot");
    ros::NodeHandle n_control("/control");

    //pubs
    pubOdometry = n.advertise<geometry_msgs::Pose>("odometry", 32);
    ros::Publisher pubSensors = n.advertise<skynav_msgs::RangeArray>("sensors", 1024);

    //subs
    ros::Subscriber subTargetPose = n_control.subscribe("target_pose", 32, subTargetPoseCallback);

    ros::spin();

    return 0;

}
