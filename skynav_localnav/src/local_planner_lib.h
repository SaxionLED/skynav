
#ifndef LOCAL_PLANNER_LIB_H
#define LOCAL_PLANNER_LIB_H
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

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

#endif

