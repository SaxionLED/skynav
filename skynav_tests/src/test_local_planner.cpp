#include <ros/ros.h>
#include <gtest/gtest.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <skynav_msgs/PointCloudVector.h>


using namespace std;

ros::Subscriber subSensorPointcloud, subPCVector, subOutlinePointcloud ;

pcl::PCLPointCloud2 mOriginal;
vector<pcl::PCLPointCloud2> mClusters;


void subSensorPointCloudDataCallback(const pcl::PCLPointCloud2ConstPtr& msg)
{
	mOriginal = (*msg);
}


void subPCVectorCallback(const skynav_msgs::PointCloudVector::ConstPtr& msg)
{
	mClusters.clear();
	
	for(vector<sensor_msgs::PointCloud2>::const_iterator it = msg->clouds.begin();it!=msg->clouds.end();++it){
		pcl::PCLPointCloud2 pc;
		pcl_conversions::toPCL((*it),pc);
		 
		mClusters.push_back(pc);
	}
}


//return the size of a vector of pointclouds
int countClusters(const vector<pcl::PCLPointCloud2>& input)
{
	return input.size();
}


TEST(LocalnavTests, countClusters)
{
	EXPECT_EQ(27, countClusters(mClusters));
}


TEST(LocalnavTests, DISABLED_waypointCheck)
{
	//pass
}


TEST(LocalnavTests, DISABLED_recursiveBugCheck)
{
	//pass
}


TEST(LocalnavTests, DISABLED_path_waypointCheckRecursivebug)
{
	//pass
	
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_obstacle_detector");
	ROS_INFO("started test_obstacle_detector");

	ros::NodeHandle n_localnav("/localnav");
	ros::NodeHandle n_control("/control");
	
	subSensorPointcloud = n_control.subscribe("cloud", 0, subSensorPointCloudDataCallback);
	subPCVector = n_localnav.subscribe("pointcloudVector",0, subPCVectorCallback);
	subOutlinePointcloud = n_localnav.subscribe("pointcloudVector",0, subPCVectorCallback);

	ros::Duration(10).sleep();
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		RUN_ALL_TESTS();
		
		ros::shutdown();
	}
	return 0;
}
