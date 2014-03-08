#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/RangeDefinedArray.h>
#include <skynav_msgs/RangeDefined.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pubOdometry;
ros::Publisher pubSensors;
ros::Publisher pubLaser;


void subSensorsCallback(const skynav_msgs::RangeDefinedArray::ConstPtr& msg) {
    
    // check limits here
    
    pubSensors.publish(msg);
}

void subRelativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg)        {
    
    // check limits here
    
    pubOdometry.publish(msg);    
}

void subLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)	{

	// check limits here
	
	pubLaser.publish(msg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "data_verifier");

    ros::NodeHandle n("/control");
    ros::NodeHandle n_robot("/x80sv");

    //pubs
    pubOdometry = n.advertise<geometry_msgs::Pose>("odometry", 32);
    pubSensors = n.advertise<skynav_msgs::RangeDefinedArray>("sensors", 1024);
    pubLaser = n.advertise<sensor_msgs::LaserScan>("laser_scan", 1024);

    //subs
    ros::Subscriber subSensors = n_robot.subscribe("sensors", 1024, subSensorsCallback);
    ros::Subscriber subRelativePose = n_robot.subscribe("odometry", 32, subRelativePoseCallback);
    ros::Subscriber subLaserRobot = n_robot.subscribe("laser/scan_filtered", 256, subLaserScanCallback);

    ros::spin();

    return 0;
}

