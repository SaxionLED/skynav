#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <skynav_msgs/waypoint_check.h>
#include <skynav_msgs/Object.h>
#include <skynav_msgs/Objects.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>


using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std;

ros::Publisher pubObjectOutlines;

vector<PointCloud> mObstacles;
vector<PointCloud> mObjectOutlines;

bool compare(const Point32 &p, const Point32 &q){
	return p.x < q.x || (p.x == q.x && p.y < q.y);
}


// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double cross(const Point32 &O, const Point32 &A, const Point32 &B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}
 
// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Returns a pointcloud containing a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
PointCloud convex_hull(vector<Point32> P)
{
	PointCloud PC;
	
	int n = P.size(), k = 0;
	vector<Point32> H(2*n);
 
	// Sort points lexicographically
	sort(P.begin(), P.end(), compare);
 
	// Build lower hull
	for (int i = 0; i < n; ++i) {
		while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}
 
	// Build upper hull
	for (int i = n-2, t = k+1; i >= 0; i--) {
		while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}
 
	H.resize(k);
	for(int i = 0; i<H.size(); ++i){
	PC.points.push_back(H.at(i));	
	}
	PC.header.stamp = ros::Time::now();
    PC.header.frame_id = "/map";
	return PC;
}
//function to call convex hull determination for each object
void convexhullFunction(){	
	for(vector<PointCloud>::iterator it = mObstacles.begin(); it!= mObstacles.end(); ++it){
		mObjectOutlines.push_back(convex_hull((*it).points));
	}	
}
//receive a custom message, containing all up to date objects within sensor range, from obstacle detector
void subObstaclesCallback(const skynav_msgs::Objects::ConstPtr& msg) {
	mObstacles.clear();
	mObjectOutlines.clear();

	for(vector<PointCloud>::const_iterator objectIt = msg->objects.begin(); objectIt!= msg->objects.end(); ++objectIt){
        mObstacles.push_back((*objectIt));
	}		
	convexhullFunction();	

	ROS_INFO("outlines %d", mObjectOutlines.size());
	
	for(vector<PointCloud>::iterator outlineIt = mObjectOutlines.begin(); outlineIt!= mObjectOutlines.end(); ++outlineIt){
		ROS_INFO("size %d",(*outlineIt).points.size());
		pubObjectOutlines.publish((*outlineIt) );
	}
}

bool servServerWaypointCheckCallback(skynav_msgs::waypoint_check::Request &req, skynav_msgs::waypoint_check::Response &response) {

    // take req.waypoints and check if the path intersects with obstacles, if not: return true else publish new path and return false

	
    // obstacles consist of 2 coordinates, the line between them is the obstacle as seen by the sensors 
    //TODO better design. use convex hull of object (pointcloud) and determine collisison with outer lines

//    Point pPath1 = req.currentPos;
//    Point pPath2 = req.targetPos;
//
//    Point pathVector;
//    pathVector.x = pPath1.x - pPath2.x;
//    pathVector.y = pPath1.y - pPath2.y;
//
//    for (int i = 0; i < mObstacles.size(); i++) {
//
//        Point pObstacle1 = mObstacles.at(i).p1;
//        Point pObstacle2 = mObstacles.at(i).p2;
//
//        // if path line collides with any obstacle line, report it //TODO margin, eg robot size
//        
//        Point obstacleVector;
//        obstacleVector.x = pObstacle2.x - pObstacle1.x;
//        obstacleVector.y = pObstacle2.y - pObstacle1.y;
//
//        float lengthCrossProduct = pathVector.x * obstacleVector.y - obstacleVector.x * pathVector.y;
//        
//        if(lengthCrossProduct != 0)     {
//            // intersection
//            
//            
//        } else {
////            ROS_INFO("No intersection found");
//        }
//
//    }

    return true;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "local_planner");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");

    //pubs
    ros::Publisher pubWaypoints = n_control.advertise<nav_msgs::Path>("checked_waypoints", 32);
    pubObjectOutlines = n.advertise<PointCloud>("objectOutlines", 1024);

    //subs
    ros::Subscriber subObstacles = n.subscribe("obstacles", 1024, subObstaclesCallback);

    //services
    ros::ServiceServer servServerWaypointCheck = n.advertiseService("path_check", servServerWaypointCheckCallback);

    ros::spin();

    return 0;
}
