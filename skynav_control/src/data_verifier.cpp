#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/RangeDefinedArray.h>
#include <skynav_msgs/RangeDefined.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <skynav_msgs/current_pose.h>


using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std;

ros::Publisher pubOdometry;
ros::Publisher pubSensors;
ros::Publisher pubLaser;
ros::Publisher pubLaserFiltered;

ros::ServiceClient servClientCurrentPose;


Pose getCurrentPose() {

    Pose currentPose;

    skynav_msgs::current_pose poseService;

    if (servClientCurrentPose.call(poseService)) {
        currentPose = poseService.response.pose;
    } else {
        ROS_ERROR("Failed to call current_pose service from data varifier");
    }
    return currentPose;
}

void subSensorsCallback(const skynav_msgs::RangeDefinedArray::ConstPtr& msg) {
    
    // check limits here
    
    pubSensors.publish(msg);
}

void subRelativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg)        {
    
    // check limits here
    
    pubOdometry.publish(msg);    
}

void subLaserScanCallback(const LaserScan::ConstPtr& msg)	{
	// check limits here
	
	pubLaser.publish(msg);
}

//filter the (raw)LaserScan data coming from the neato
void subFilterLaserCallback(const LaserScan::ConstPtr& scan) {

	LaserScan scan_filtered;
	scan_filtered.header.frame_id = scan->header.frame_id;
	scan_filtered.header.stamp = scan->header.stamp;

	scan_filtered.angle_min = scan->angle_min;
	scan_filtered.angle_max = scan->angle_max;
	scan_filtered.angle_increment = scan->angle_increment;
	scan_filtered.scan_time = scan->scan_time;
	scan_filtered.range_min = scan->range_min;
	scan_filtered.range_max = scan->range_max;	
	
	int discard_angle = 30;
	for(uint i = 0; i < scan->ranges.size(); ++i)	{
		
		//	// UNCOMMENT IF USING ROBOT INSTEAD OF SIMULATOR!
		//	//limit the laser angle on the actual robot so the robot itself wont be seen as object.
		//	//the lds has been mounted on the robot with a 90 degree turn.. which results in the 0 degree (front) of the robot being at the 270 degree of the lds output

		//if(i>discard_angle && i<180-discard_angle){
			//scan_filtered.ranges.push_back( 0 );	// add 0 if outside limit or it ruins the rest of the angles (relient on array size)
			//scan_filtered.intensities.push_back( 0 );
			//continue;
		//}
		
		//filter the outer edge of the laser range out of the results
		if( scan->ranges.at(i) > (scan->range_max - 0.1) || scan->ranges.at(i) < scan->range_min)	{	
			scan_filtered.ranges.push_back( 0 );	// add 0 if outside limit or it ruins the rest of the angles (relient on array size)
			scan_filtered.intensities.push_back( 0 );
			continue;
		}

		scan_filtered.ranges.push_back( scan->ranges.at(i) );
		scan_filtered.intensities.push_back( scan->intensities.at(i) );
	}
    
    pubLaserFiltered.publish(scan_filtered);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "data_verifier");

    ros::NodeHandle n("/control");
    ros::NodeHandle n_robot("/x80sv");
    ros::NodeHandle n_SLAM("/slam");


    //pubs
    pubOdometry = n.advertise<geometry_msgs::Pose>("odometry", 32);
    pubSensors = n.advertise<skynav_msgs::RangeDefinedArray>("sensors", 1024);
    pubLaser = n.advertise<sensor_msgs::LaserScan>("laser_scan", 1024);
    
	pubLaserFiltered = n_robot.advertise<LaserScan>("laser/scan_filtered", 1000);
    //subs
    ros::Subscriber subSensors = n_robot.subscribe("sensors", 1024, subSensorsCallback);
    
    ros::Subscriber subRelativePose = n_robot.subscribe("odometry", 32, subRelativePoseCallback);
    ros::Subscriber subLaserRobot = n_robot.subscribe("laser/scan_filtered", 1000, subLaserScanCallback);	
	ros::Subscriber subLaser = n_robot.subscribe("laser/scan", 1000, subFilterLaserCallback);
	
	//services
    servClientCurrentPose = n_SLAM.serviceClient<skynav_msgs::current_pose>("current_pose");
    
    ros::spin();

    return 0;
}

