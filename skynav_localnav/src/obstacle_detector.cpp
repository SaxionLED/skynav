#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread/mutex.hpp>

#include "obstacle_detector_lib.h"

using namespace std;

ros::Publisher pubSensorData, pubCloud, pubCloudRaw, pubPCVector, pubClusters;
ros::ServiceClient servClientCurrentPose;

tf::TransformListener* mTransformListener;

Pcl2Vector mCloudSet;		//the set of pointclouds received in one loop

boost::mutex mMutex;

//output a pointcloud to a PCD file
void writeOutputPCD(const pcl::PCLPointCloud2& inputCloud)
{	
	//dont do anything and return when empty input
	if((inputCloud.width * inputCloud.height)==0)
	{
		//ROS_WARN("empty inputcloud, nothing to be done");
		return;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(inputCloud, *cloud);
	try{
		pcl::io::savePCDFileASCII (ros::package::getPath("skynav_localnav")+"/export/EXPORT.pcd", *cloud);
	}catch(std::exception& e){
		ROS_ERROR("obstacle_detector %s",e.what());
	}	
}


//discard objects outside of certain range, or when a new scan has been done
void forgetObjects()
{
	mCloudSet.clear();
}


//publish all known objects
void publishObjects()
{		
	boost::mutex::scoped_lock lock(mMutex);
	
	//if the cloudset is empty, dont publish anything
	if(mCloudSet.empty()){
		return;
	}	
	
	//declare pointclouds
	pcl::PCLPointCloud2 cloud;	
	pcl::PCLPointCloud2 cloud_filtered;
	Pcl2Vector cloud_clusters;
	skynav_msgs::PointCloudVector msg;

	//construct full pointcloud and apply filter(s)
	cloud = constructEnvironmentCloud(mCloudSet);	
	cloud_filtered = voxelfilter(cloud);	

	//publish pointclouds
	cloud.header.frame_id = "/map";		
	cloud_filtered.header.frame_id = "/map";		
	pubCloudRaw.publish(cloud);
	pubCloud.publish(cloud_filtered);

	//extract and publish an message with clusters, for local planner purpose
	cloud_clusters = extractClusters(cloud_filtered);
	
	//TODO these converstions are dirty and unwanted!
	for(Pcl2Vector::iterator it = cloud_clusters.begin();it!=cloud_clusters.end();++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPC (new pcl::PointCloud<pcl::PointXYZ>);
		sensor_msgs::PointCloud2 cluster;
		pcl::fromPCLPointCloud2((*it), *pclPC);
		pcl::toROSMsg(*pclPC, cluster);
		
		msg.clouds.push_back(cluster);
		pubClusters.publish(cluster);
	}
	pubPCVector.publish(msg);
		
	//clean up data
	forgetObjects();	

//----FOR TESTING PURPOSES-----
	//write cloud to pcd file		
	//writeOutputPCD(cloud);
//-----------------------------	
}


//add the inputCloud to the global vector of pointclouds
void addToEnvironmentCloudSet(const pcl::PCLPointCloud2& inputCloud){
	boost::mutex::scoped_lock lock(mMutex);
	mCloudSet.push_back(inputCloud);
}


/* subscribe on the sensor_msgs::PointCloud2 publisher, upon recieve convert to pcl::PCLPointCLoud2.
 * determine and devide the cloud in multiple clusters for further use.
 *  
 * note on pcl conversion
 * "When subscribing to a pcl::PointCloud<T> topic with a sensor_msgs::PointCloud2 subscriber or viceversa, 
 * the conversion (deserialization) between the two types sensor_msgs::PointCloud2 and pcl::PointCloud<T> 
 * is done on the fly by the subscriber."
 * source: http://wiki.ros.org/pcl/Overview
 */
void subPointCloudDataCallback(const pcl::PCLPointCloud2ConstPtr& msg) 
{
	addToEnvironmentCloudSet(*msg);	
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");
 
    //pubs
    pubPCVector = n.advertise<skynav_msgs::PointCloudVector>("pointcloudVector",1);
    
    pubCloud 	= 	n.advertise<pcl::PCLPointCloud2>("pointCloudData",10);
    pubCloudRaw = 	n.advertise<pcl::PCLPointCloud2>("pointCloudDataRaw",10);
    pubClusters = 	n.advertise<pcl::PCLPointCloud2>("pointCloudDataClusters",10);

    //subs
	ros::Subscriber subSensorCloud = n_control.subscribe("cloud", 10, subPointCloudDataCallback);
    // TODO: subscribe to pointcloud in base_link frame from sensors and translate to "map" frame for obstacle detection.
    
    mTransformListener = new tf::TransformListener();
	
    ros::Rate loop_rate(1);
	
	while (ros::ok()) {
            
		ros::spinOnce();
		
		publishObjects();
		
		loop_rate.sleep();		
	}
            
    delete mTransformListener;

    return 0;
}
