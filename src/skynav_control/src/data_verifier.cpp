#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/RangeDefinedArray.h>
#include <skynav_msgs/RangeDefined.h>
#include <sensor_msgs/Range.h>

ros::Publisher pubOdometry;
ros::Publisher pubSensors;


void subSensorsCallback(const skynav_msgs::RangeDefinedArray::ConstPtr& msg) {
    
    // check limits here
    
    

    pubSensors.publish(msg);
}

void subRelativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg)        {
    
    // check limits here
    
    pubOdometry.publish(msg);    
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "data_verifier");

    ros::NodeHandle n("/control");
    ros::NodeHandle n_robot("/robot");

    //pubs
    pubOdometry = n.advertise<geometry_msgs::Pose>("odometry", 32);
    pubSensors = n.advertise<skynav_msgs::RangeDefinedArray>("sensors", 1024);

    //subs
    ros::Subscriber subSensors = n_robot.subscribe("sensors", 1024, subSensorsCallback);
    ros::Subscriber subRelativePose = n_robot.subscribe("odometry", 32, subRelativePoseCallback);

    ros::spin();

    return 0;
}

