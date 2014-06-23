#include "obstacle_detector_lib.h"

using namespace std;

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
	voxel.setLeafSize (0.01f, 0.01f, 0.01f); //in meters, -> 1 cm3 leafs
	voxel.filter (outputCloud);
	
	return outputCloud;
}


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
	
	//Fill in the ModelCoefficients values. In this case, we use a plane model, with ax+by+cz+d=0, where a=b=d=0, and c=1, 
	//or said differently, the X-Y plane.
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
