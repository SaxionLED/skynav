
#ifndef LOCAL_PLANNER_LIB_H
#define LOCAL_PLANNER_LIB_H

//TODO FIX
//this causes segfaults in pcl::ConcaveHull!!
// be carefull of this define; 
// see http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
// and http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
//#define EIGEN_DONT_ALIGN_STATICALLY 
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <math.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/thread/mutex.hpp>
#include "localnav_types.h"

#define ROBOTRADIUS 		1 		//the radius of the robot. TODO get this from somewhere robot dependent
#define MAX_SENSORDIST 		4		//the outer range of the sensors TODO get this from laser sensor dependent


//truncate values (in meters) to certain precision
float truncateValue(const float value);

double pclCalcDistance(pcl::PointXYZ a, pcl::PointXYZ b);

bool pclCompare_Point(pcl::PointXYZ p, pcl::PointXYZ q);

//determine pcl convex hull
pcl::PCLPointCloud2 pclConvex_hull(pcl::PCLPointCloud2& inputCloud);

//determine pcl concave hull
pcl::PCLPointCloud2 pclConcave_hull(pcl::PCLPointCloud2& inputCloud);

//recursive bug algorithm for object avoidance 
pclOptionPoint pclRecursiveBug(const pcl::PointXYZ currentPos, const pcl::PointXYZ targetPos, const pcl::PointXYZ collisionPoint, const pcl::PCLPointCloud2 objectPC_input);

//function to determine if there is a colission, where, and (if needed) call for a new waypoint calculation.
//if recursiveBugNeeded is true, the return value is the new waypoint calculated with the recursivebug algorithm, if false: the return is the collisionpoint itself
pclOptionPoint pclWaypointCheck(const pcl::PointXYZ pPath1, const pcl::PointXYZ pPath2, Pcl2Vector outlinesInput, bool recursiveBugNeeded);


//-------------------------------------------------------------------------------------------------//
//old style pointcloud, to be deprecated

//compare both points to each other
bool compare_Point(geometry_msgs::Point p, geometry_msgs::Point q);

//calculate distance between two point with use of pythagoras
double calcDistance(geometry_msgs::Point a, geometry_msgs::Point b);

//determine the convex outer shape of the 2d pointcloud
sensor_msgs::PointCloud convex_hull(sensor_msgs::PointCloud data);

//determine the concave outer shape of the 2d pointcloud
sensor_msgs::PointCloud concave_hull(sensor_msgs::PointCloud data);

//recursive bug algorithm for object avoidance
optionPoint recursiveBug(const geometry_msgs::Point currentPos,const geometry_msgs::Point targetPos, const geometry_msgs::Point collisionPoint, const sensor_msgs::PointCloud objectPC);

//function to determine if there is a colission, where, and (if needed) call for a new waypoint calculation.
//if recursiveBugNeeded is true, the return value is the new waypoint calculated with the recursivebug algorithm, if false: the return is the collisionpoint itself
optionPoint waypointCheck(const geometry_msgs::Point pPath1, const geometry_msgs::Point pPath2, std::vector<sensor_msgs::PointCloud> outlines, bool recursiveBugNeeded);

#endif

