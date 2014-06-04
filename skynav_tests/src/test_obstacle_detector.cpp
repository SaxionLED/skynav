#include <ros/ros.h>
#include <gtest/gtest.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <skynav_msgs/PointCloudVector.h>

using namespace std;

ros::Subscriber subODpointcloud,subPCVector,subSensorpointcloud,subODRawpointcloud ;

vector<pcl::PCLPointCloud2> mClusters;	
pcl::PCLPointCloud2 mOriginal;
pcl::PCLPointCloud2 mReceived;
pcl::PCLPointCloud2 mReceivedRaw;



void subODPointCloudDataCallback(const pcl::PCLPointCloud2ConstPtr& msg)
{
	mReceived = (*msg);
}


void subSensorPointCloudDataCallback(const pcl::PCLPointCloud2ConstPtr& msg)
{
	mOriginal = (*msg);
}


void subODRawPointCloudDataCallback(const pcl::PCLPointCloud2ConstPtr& msg)
{
	mReceivedRaw = (*msg);
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


//compare two pointclouds based on size
bool comparePointClouds(const pcl::PCLPointCloud2& inputCloudA, const pcl::PCLPointCloud2& inputCloudB)
{
	int sizeA = inputCloudA.width * inputCloudA.height;
	int sizeB = inputCloudB.width * inputCloudB.height;

	if(sizeA == 0 || sizeB == 0 || sizeA % sizeB != 0)
	{
		return false;
	} 
	
	return true;
}


//return the size of a vector of pointclouds
int countClusters(const vector<pcl::PCLPointCloud2>& input)
{
	return input.size();
}


//test if output of obstacle_detector is (a modulo of) the original pointcloud in size
TEST(ObstacleDetectorTests, comparePointClouds)
{
	EXPECT_EQ(true, comparePointClouds(mReceivedRaw,mOriginal));
}


//test if the nr of clusters detected is what it should be according to the provided pointcloud
TEST(ObstacleDetectorTests, countClusters)
{
	EXPECT_EQ(27, countClusters(mClusters));		
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_obstacle_detector");
	ROS_INFO("started test_obstacle_detector");

	ros::NodeHandle n_localnav("/localnav");
	ros::NodeHandle n_control("/control");
	
	subPCVector = n_localnav.subscribe("pointcloudVector",0, subPCVectorCallback);
	subODpointcloud = n_localnav.subscribe("pointCloudData",0,subODPointCloudDataCallback);
	subODRawpointcloud = n_localnav.subscribe("pointCloudDataRaw",0,subODRawPointCloudDataCallback);

	subSensorpointcloud = n_control.subscribe("cloud", 0, subSensorPointCloudDataCallback);

	ros::Duration(10).sleep();

	while (ros::ok())
	{
		ros::spinOnce();	
			
		RUN_ALL_TESTS();
				
		ros::shutdown();
	}
	return 0;
}
