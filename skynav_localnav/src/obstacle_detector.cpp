#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>

#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/Object.h>
#include <skynav_msgs/Objects.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/thread/mutex.hpp>
#include <set>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#define ROBOTRADIUS 					0.5 	//the radius of the robot in meters. TODO get this from somewhere robot dependent
#define MAX_SENSORDIST 					4		//the outer range of the sensors in meters

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

struct compPoint 
{
    bool operator() (const Point32& first, const Point32& second) const {
        if (first.x < second.x)
            return true;
        else if (first.x > second.x)
            return false;
        else // if x the same
            if (first.y < second.y)
            return true;
        else
            return false; // if x the same and y the same or second y greater
    }
};

int mObjectsFound = 0;

ros::Publisher pubObjects, /*pubClusters,*/ pubObstacles, pubSensorData, pubSensorData_old, pubCloud;
ros::ServiceClient servClientCurrentPose;

laser_geometry::LaserProjection mLaserProjector;
tf::TransformListener* mTransformListener;

set<Point32, compPoint> mSensorData;
vector<PointCloud> mObjects;				//objects in memory (old)

//vector<pcl::PCLPointCloud2> mClusters;		//the set of clusters

vector<pcl::PCLPointCloud2> mCloudSet;		//the set of pointclouds received in one loop

boost::mutex mMutex;

Pose getCurrentPose() 
{
    Pose currentPose;

    skynav_msgs::current_pose poseService;

    if (servClientCurrentPose.call(poseService)) {

        currentPose = poseService.response.pose;

        //        ROS_INFO("Current pose (x %f) (y %f) (theta %f)", poseService.response.pose.x, poseService.response.pose.y, poseService.response.pose.theta);
    } else {
        ROS_ERROR("Failed to call current_pose service from obstacle detector");
    }

    return currentPose;
}


//determine if a point already exist in a list of points.
static bool pointExists(const Point32* a, vector<Point32>* vec) {
	const double xyRange = 0.02; //within 2 cm, it is concidered the same coordinate
    vector<Point32>::iterator it;

    for (it = vec->begin(); it != vec->end(); ++it) {
		
		
		if(((*it).x == a->x && (*it).y == a->y)){	//point coordinates are exactly the same
            return true;
		}
		
		double xyDistance = sqrt(pow((*it).x - a->x,2) + (pow((*it).y - a->y,2)));
		if (xyDistance <= xyRange){					//point exist by close proximity ( taken accuracy of sensor in consideration)
			return true;							
		}
    }	

    return false;									//point does not already exist
}


//determine if two points are within a certain distance to eachother
static bool pointInRange(const Point32* a, const Point32* b, const double searchDistance) 
{
		double xyDistance = sqrt(pow((a->x -b->x),2) + pow((a->y - b->y),2));	//use pythagoras to determine if coordinates of a are within certain range of b
		if(xyDistance <= searchDistance)
		{														
			return true;
		}  		  
    return false;
}


//truncate values (in meters) to mm precision
float truncateValue(const float value)
{
	return floorf(value*1000)/1000; //mm
}


//concatinate the two pointclouds and return this pointcloud
pcl::PCLPointCloud2 concatinateClouds(const pcl::PCLPointCloud2& inputCloudA, const pcl::PCLPointCloud2& inputCloudB)
{	
	pcl::PCLPointCloud2 returnCloud;	
	
	//TODO pcl::PCLPointCloud2 does not seem to have operator '+=' even though it is documented by pcl.. needs further investigation
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
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


//applies a voxelgrid filter for the pointcloud input and returns a filtered cloud
pcl::PCLPointCloud2 voxelfilter(const pcl::PCLPointCloud2& inputCloud)
{
	pcl::PCLPointCloud2ConstPtr InputCloudPtr(new pcl::PCLPointCloud2(inputCloud)); //ugly copy constructor, but seemed neccesary for voxelgrid.setInputCloud() =(
	pcl::PCLPointCloud2 outputCloud;
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
	voxel.setInputCloud (InputCloudPtr);
	voxel.setLeafSize (0.01f, 0.01f, 0.01f);
	voxel.filter (outputCloud);
	
	//ROS_INFO("Filtered pointCloud from %d to %d",inputCloud.width * inputCloud.height,outputCloud.width * outputCloud.height);
	return outputCloud;
}


//project the pointcloud (2D or 3D) onto the XYplane (Z=0)
pcl::PCLPointCloud2 projectOnXYPlane(const pcl::PCLPointCloud2& inputCloud)
{
	pcl::PCLPointCloud2 outputCloud;
	//TODO
	return outputCloud;
}


//split the input pointcloud into multiple clusters and return them
vector<pcl::PCLPointCloud2> defineClusters(const pcl::PCLPointCloud2& inputCloud)
{
	vector<pcl::PCLPointCloud2> outputClusters;
	pcl::PCLPointCloud2 clusterCloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromPCLPointCloud2(inputCloud, *cloud);
	
	//Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
  
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance (0.1); // in meters
	ec.setMinClusterSize (4);
	ec.setMaxClusterSize (250000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;	
		
		pcl::toPCLPointCloud2(*cloud_cluster, clusterCloud );
		
		clusterCloud.header.frame_id = "/map";

		outputClusters.push_back(clusterCloud);	
	}
	//ROS_INFO("nr of points: %d", (inputCloud.width * inputCloud.height));
	//ROS_INFO("nr of clusters: %d", mClusters.size());
	
	return outputClusters;
}


//add the inputCloud to the global vector of pointclouds
void addToEnvironmentCloudSet(const pcl::PCLPointCloud2& inputCloud){
	boost::mutex::scoped_lock lock(mMutex);
	mCloudSet.push_back(inputCloud);
}


//concatinate the available clouds gathered in one loop into one pointcloud
pcl::PCLPointCloud2 constructEnvironmentCloud(const vector<pcl::PCLPointCloud2>& cloudSet)
{
	pcl::PCLPointCloud2 cloud;

	for(vector<pcl::PCLPointCloud2>::const_iterator it = cloudSet.begin(); it!= cloudSet.end(); ++it)
	{
		if (cloud.width * cloud.height == 0)
		{
			cloud = (*it); //the first iteration (so cloud is empty). cloudset[0] is dus basis voor cloud
		}else
		{
			cloud = concatinateClouds(cloud, (*it));
		}
	}
	ROS_INFO("cloud size %d",(cloud.width * cloud.height));

	return cloud;
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
void subClusterDeterminationCallback(const pcl::PCLPointCloud2ConstPtr& msg) 
{
	addToEnvironmentCloudSet(*msg);	
	//mClusters = defineClusters(*msg);
}


//soon to be DEPRECATED
//detect objects within sensor range, compare to known objects and add/merge in object list
void subObjectDetectionCallback(const sensor_msgs::PointCloud::ConstPtr& msg) 
{
    // some vars
    const double xySearchDistance = 0.1; // 10cm        //TODO should be settable somewhere 
	
    // now we can check our newest points with the ones already in memory, find other points that are nearby and points near the found points
    for (uint i = 0; i < msg->points.size(); ++i) {
    
        PointCloud foundPoints;
		set<Point32>::iterator sensor_it;

        // check if objects were found nearby
        vector<PointCloud>::iterator firstObjectIt;

        for (firstObjectIt = mObjects.begin(); firstObjectIt != mObjects.end(); ++firstObjectIt) {

            for (uint firstPoints = 0; firstPoints < (*firstObjectIt).points.size(); ++firstPoints) {  
                
                if( pointInRange(&((*firstObjectIt).points.at(firstPoints)), &(msg->points.at(i)), xySearchDistance))   { // if point within range of another point
                
                    // if true, the coordinate is within xySearchDistance of any of the coordinates contained in the object and we should add it to that object
                    bool addOriginalPoint = true;

                    if (pointExists(&(msg->points.at(i)), &((*firstObjectIt).points))) { // do not add duplicates
                        addOriginalPoint = false; // point is already in the object, prevent adding it twice 
                    }

                    // point is added to object, now check if the point is also in range of another object
                    vector<PointCloud>::iterator secondObjectIt;

                    for (secondObjectIt = firstObjectIt + 1; secondObjectIt != mObjects.end();) {
                        bool erasedObject = false;

                        for (uint secondPoints = 0; secondPoints < (*secondObjectIt).points.size(); ++secondPoints) {
                            if( pointInRange(&((*secondObjectIt).points.at(secondPoints)), &(msg->points.at(i)), xySearchDistance))   {
                            
                                // another object in range was found

                                // to prevent the original point being added twice
                                if (pointExists( &(msg->points.at(i)), &((*secondObjectIt).points))) { // do not add duplicates
                                    addOriginalPoint = false;
                                }

                                // merge the two objects
                                (*firstObjectIt).points.insert((*firstObjectIt).points.end(), (*secondObjectIt).points.begin(), (*secondObjectIt).points.end()); // add all points from 2nd vector to the first
                                (*secondObjectIt).points.clear(); // empty vector

                                erasedObject = true;
                                secondObjectIt = mObjects.erase(secondObjectIt); // remove locally, can re-use the index for it
                                //ROS_INFO("merged objects");

                                break; // if point is in range, this needs not be executed again for that object

                                // repeat until all objects have been checked
                            }
                        }

                        if (!erasedObject) {
                            ++secondObjectIt;
                        }
                    }

                    if (addOriginalPoint) {
                        (*firstObjectIt).points.push_back(msg->points.at(i));
                    }

                    goto nextNewPoint;	//point has been processed. jump to next point in pointcloud
                }
            }
        }

        // no objects nearby were found, check if there are any single points nearby to create a new object out of

        for (sensor_it = mSensorData.begin(); sensor_it != mSensorData.end(); ++sensor_it) {
			
			if( pointInRange( &(*sensor_it), &(msg->points.at(i)), xySearchDistance))   {       //if point near other point
                foundPoints.points.push_back((*sensor_it));
                mSensorData.erase(sensor_it--); // is this safe?
                }
        }

        if (foundPoints.points.size() > 0) { // if points were found nearby

            foundPoints.points.push_back(msg->points.at(i)); // add the reference point, since it was not added anywhere yet
            
            foundPoints.header.stamp = ros::Time(0);
            foundPoints.header.frame_id = "/map";
            
            mObjects.push_back(foundPoints);

        } else { // no nearby points found
            // if no object found either, save the point for future searching
            mSensorData.insert(msg->points.at(i)); // note that a set is used because this prevents duplicates
        }
        
        // this skips everything and restarts the first for loop. should only be called when a new point (from msg) has been added to an object.
        nextNewPoint:
        asm("NOP");
        
    }    
}


//discard objects outside of certain range, or when a new scan has been done
void forgetObjects()
{
	mObjects.clear();
//	mClusters.clear();
	mCloudSet.clear();
}


//publish deprecated pointcloud type and object msg for backward compatibility during development
void publishOldData()
{
	//publish message for localnav calculations
	skynav_msgs::Objects msg;
	msg.objects = mObjects;
	pubObstacles.publish(msg);	
	
	//publish multiple clouds for visual representation
	for(vector<PointCloud>::iterator it = mObjects.begin(); it != mObjects.end(); ++it)	
	{
		pubObjects.publish( (*it) );
	}
}


//publish all known objects
void publishObjects()
{		
	boost::mutex::scoped_lock lock(mMutex);
	
	publishOldData(); //for backwards compatibility during development of the working software

	pcl::PCLPointCloud2 cloud;	
	pcl::PCLPointCloud2 cloud_filtered;
	
	cloud = constructEnvironmentCloud(mCloudSet);	
	cloud_filtered = voxelfilter(cloud);	
	
	cloud_filtered.header.frame_id = "/map";		
	
	pubCloud.publish(cloud_filtered);

	forgetObjects();	
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");
    ros::NodeHandle n_SLAM("/slam");

    //pubs
    pubObjects = n.advertise<PointCloud>("objects", 1024);
    pubObstacles = n.advertise<skynav_msgs::Objects>("obstacles", 1024);
    //pubClusters = n.advertise<pcl::PCLPointCloud2>("clusters",1024);
    pubCloud = n.advertise<pcl::PCLPointCloud2>("pointCloudData",10);

    //subs
    // TODO: subscribe to pointcloud in base_link frame from sensors and translate to "map" frame for obstacle detection.
    // ros::Subscriber subSensors = n_control.subscribe("sensors", 10, subSensorCallback); // raw unprocessed sensor values
    ros::Subscriber subSensorCloud = n_control.subscribe("cloud", 10, subClusterDeterminationCallback);
    //ros::Subscriber subSensorData = n_control.subscribe("sensor_data", 10, subObjectDetectionCallback); //old pointcloud type based laser data

    //services
    servClientCurrentPose = n_SLAM.serviceClient<skynav_msgs::current_pose>("current_pose");
    
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
