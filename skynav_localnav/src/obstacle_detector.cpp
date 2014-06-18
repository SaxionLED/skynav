#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <skynav_msgs/PointCloudVector.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>

#include "localnav_types.h"

using namespace std;

ros::Publisher pubSensorData, pubCloud, pubCloudRaw, pubPCVector, pubClusters;
ros::ServiceClient servClientCurrentPose;

tf::TransformListener* mTransformListener;

Pcl2Vector mCloudSet;		//the set of pointclouds received in one loop

boost::mutex mMutex;


//applies a voxelgrid filter for the pointcloud input and returns a filtered cloud
pcl::PCLPointCloud2 voxelfilter(const pcl::PCLPointCloud2& inputCloud)
{
	//dont filter anything and return when empty input
	if((inputCloud.width * inputCloud.height)==0)
	{
		ROS_WARN("empty inputcloud, no filter applied");
		return inputCloud;
	}
	
	pcl::PCLPointCloud2ConstPtr InputCloudPtr(new pcl::PCLPointCloud2(inputCloud)); //ugly copy constructor, but seemed neccesary for voxelgrid.setInputCloud() =(
	pcl::PCLPointCloud2 outputCloud;
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
	voxel.setInputCloud (InputCloudPtr);
	voxel.setLeafSize (0.01f, 0.01f, 0.01f);
	voxel.filter (outputCloud);
	
	return outputCloud;
}

//TODO allign the vector because of eigen component!!!
//http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
//determine and extract clusters from the input cloud and return them
Pcl2Vector extractClusters(const pcl::PCLPointCloud2& inputCloud)
{
	Pcl2Vector outputClusters;
	
	//dont do anything and return when empty input
	if((inputCloud.width * inputCloud.height)==0)
	{
		ROS_WARN("empty inputcloud, no clusters determined");
		return outputClusters;
	}
	
	pcl::PCLPointCloud2 clusterCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(inputCloud, *cloud);
	
	//Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
  
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.20); 		// in meters
	ec.setMinClusterSize (3); 			//default = 1
	//ec.setMaxClusterSize (250000); 	//default = MAXINT
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;	
		
		pcl::toPCLPointCloud2(*cloud_cluster, clusterCloud );
		
		clusterCloud.header.frame_id = "/map";

		outputClusters.push_back(clusterCloud);	
	}

	//if no seperate clusters could be extracted, return a vector containing the full cloud
	if(outputClusters.empty())
	{
		outputClusters.push_back(inputCloud);
	}	
	return outputClusters;
}


//concatinate the two pointclouds and return the concatinated pointcloud
pcl::PCLPointCloud2 concatinateClouds(const pcl::PCLPointCloud2& inputCloudA, const pcl::PCLPointCloud2& inputCloudB)
{	
	pcl::PCLPointCloud2 returnCloud;	
	
	//TODO pcl::PCLPointCloud2 does not seem to have operator '+=' even though it is documented by pcl.. needs further investigation
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(inputCloudA, *cloud1);
	pcl::fromPCLPointCloud2(inputCloudB, *cloud2);
	
	try
	{	
		*cloud1 += *cloud2;		
	}
	catch(std::exception& e)
	{
		ROS_ERROR("obstacle_detector:: concatinate  %s",e.what());
	}
	
	pcl::toPCLPointCloud2(*cloud1, returnCloud );		

	return returnCloud;	
}


//TODO CHECK // project the pointcloud (2D or 3D) onto the XYplane (Z=0)
pcl::PCLPointCloud2 projectOnXYPlane(const pcl::PCLPointCloud2& inputCloud)
{
	if((inputCloud.width * inputCloud.height)==0)
	{
		//ROS_WARN("empty inputcloud, no projection performed determined");
		return inputCloud;
	}
	pcl::PCLPointCloud2 outputCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud_projected;	
	
	pcl::fromPCLPointCloud2(inputCloud, *cloud);
	
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	
	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (cloud_projected);
	
	pcl::toPCLPointCloud2(cloud_projected, outputCloud);
	
	return outputCloud;
}


//concatinate the available clouds gathered in one loop into one pointcloud
pcl::PCLPointCloud2 constructEnvironmentCloud(const Pcl2Vector& cloudSet)
{
	pcl::PCLPointCloud2 cloud;

	for(Pcl2Vector::const_iterator it = cloudSet.begin(); it!= cloudSet.end(); ++it)
	{
		if (cloud.width * cloud.height == 0)
		{
			cloud = (*it); //the first iteration (so cloud is empty). Cloudset[0] is the basecloud
		}else
		{
			cloud = concatinateClouds(cloud, (*it));
		}
	}
	return cloud;
}


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

	//create pointclouds
	cloud = constructEnvironmentCloud(mCloudSet);	
	cloud_filtered = voxelfilter(cloud);	

	//publish pointclouds
	cloud.header.frame_id = "/map";		
	cloud_filtered.header.frame_id = "/map";		
	pubCloudRaw.publish(cloud);
	pubCloud.publish(cloud_filtered);

	//publish clusters	
	cloud_clusters = extractClusters(cloud_filtered);
	
	//publish message with vector of pointcloud2 for local planner. 
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
