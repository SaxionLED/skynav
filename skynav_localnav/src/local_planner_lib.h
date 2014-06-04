
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

typedef sensor_msgs::PointCloud PointCloud;
typedef geometry_msgs::Point Point;
typedef geometry_msgs::Point32 Point32;
typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::PoseStamped PoseStamped;

typedef boost::optional<Point> optionPoint;		

//calculate distance between two point with use of pythagoras
double calcDistance(Point a, Point b);

//truncate values (in meters) to certain precision
float truncateValue(const float value);

//compare both points to each other
bool compare_Point(Point p, Point q);

//determine the convex outer shape of the 2d pointcloud
PointCloud convex_hull(PointCloud data);

//determine the concave outer shape of the 2d pointcloud
PointCloud concave_hull(PointCloud data);

//recursive bug algorithm for object avoidance
optionPoint recursiveBug(const Point currentPos,const Point targetPos, const Point collisionPoint, const PointCloud objectPC);

//function to determine if there is a colission, where, and (if needed) call for a new waypoint calculation.
//if recursiveBugNeeded is true, the return value is the new waypoint calculated with the recursivebug algorithm, if false: the return is the collisionpoint itself
optionPoint waypointCheck(const Point pPath1, const Point pPath2, std::vector<PointCloud> outlines, bool recursiveBugNeeded);

#endif

