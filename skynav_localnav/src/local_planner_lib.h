
#ifndef LOCAL_PLANNER_LIB_H
#define LOCAL_PLANNER_LIB_H

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

#include <boost/optional.hpp>
#include <boost/thread/mutex.hpp>


#define ROBOTRADIUS 		1 		//the radius of the robot. TODO get this from somewhere robot dependent
#define MAX_SENSORDIST 		4		//the outer range of the sensors TODO get this from laser sensor dependent

typedef boost::optional<geometry_msgs::Point> optionPoint;
typedef boost::optional<pcl::PointXYZ> pclOptionPoint;		

//calculate distance between two point with use of pythagoras
double calcDistance(geometry_msgs::Point a, geometry_msgs::Point b);

//truncate values (in meters) to certain precision
float truncateValue(const float value);

//compare both points to each other
bool compare_Point(geometry_msgs::Point p, geometry_msgs::Point q);

//determine pcl convex hull
pcl::PCLPointCloud2 pclConvex_hull(pcl::PCLPointCloud2 inputCloud);

//determine pcl concave hull
pcl::PCLPointCloud2 pclConcave_hull(pcl::PCLPointCloud2 inputCloud);

//recursive bug algorithm for object avoidance 
pclOptionPoint recursiveBug(const pcl::PointXYZ currentPos, const pcl::PointXYZ targetPos, const pcl::PointXYZ collisionPoint, const pcl::PCLPointCloud2 objectPC_input);

//function to determine if there is a colission, where, and (if needed) call for a new waypoint calculation.
//if recursiveBugNeeded is true, the return value is the new waypoint calculated with the recursivebug algorithm, if false: the return is the collisionpoint itself
pclOptionPoint waypointCheck(const pcl::PointXYZ pPath1, const pcl::PointXYZ pPath2, std::vector<pcl::PCLPointCloud2> outlinesInput, bool recursiveBugNeeded);


//-------------------------------------------------------------------------------------------------//
//old style pointcloud, to be deprecated


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

