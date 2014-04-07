#include <ros/ros.h>
#include <math.h>
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

Point mNewWaypoint;			//new waypoint calculated by recursiveBug

//compare both point32s to each other
bool compare(const Point32 &p, const Point32 &q){
	return p.x < q.x || (p.x == q.x && p.y < q.y);
}


//compare both points to each other
bool compare_Point(const Point &p, const Point &q){
	return p.x < q.x || (p.x == q.x && p.y < q.y);
}


//calculate distance between two point with use of pythagoras
double calcDistance(Point a, Point b){
	return sqrt(pow((a.x -b.x),2) + pow((a.y -b.y),2));
}

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double cross(const Point32 &O, const Point32 &A, const Point32 &B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

//TODO Convex hull is not correct when encountering concave objects like walls!!
 
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
	mObjectOutlines.clear();	//clear the outlines and replace with new data
	
	for(vector<PointCloud>::iterator it = mObstacles.begin(); it!= mObstacles.end(); ++it){
		mObjectOutlines.push_back(convex_hull((*it).points));
	}
	//ROS_INFO("outlines %d", mObjectOutlines.size());	
	for(vector<PointCloud>::iterator outlineIt = mObjectOutlines.begin(); outlineIt!= mObjectOutlines.end(); ++outlineIt){
		pubObjectOutlines.publish((*outlineIt) );
	}	
}


//receive a custom message, containing all up to date objects within sensor range, from obstacle detector
void subObstaclesCallback(const skynav_msgs::Objects::ConstPtr& msg) {
	mObstacles.clear();

	for(vector<PointCloud>::const_iterator objectIt = msg->objects.begin(); objectIt!= msg->objects.end(); ++objectIt){
        mObstacles.push_back((*objectIt));
	}
}


//recursive bug algorithm for object avoidance
bool recursiveBug(const Point currentPos,const Point targetPos, const Point collisionPoint, const PointCloud objectPC){
	//ROS_INFO("recursive_bug");
	
	bool foundNew = false;
		
	Point nwTarget;
	Point obstacleExtremeLeft;
	Point obstacleExtremeRight;
	Point shortestExtremeToTarget;
	Point intersection = collisionPoint;
	Point laser_coord;
	
	double offsetDegree=10; //TODO use robot radius to calculate new coordinate
	double laserDist = 4;	
	double objectDist = calcDistance(currentPos,collisionPoint);
	double angleTowardTarget = atan2((targetPos.y - currentPos.y),(targetPos.x - currentPos.x));

	double A1,B1,C1; 	
	double A2,B2,C2;
	double determant;
	bool intersectFound;

	//ROS_INFO("left");
	for(int i = 0; i<=180; ++i){
		float angle = (M_PI*i)/180;
		intersectFound = false;

		laser_coord.x = (currentPos.x + (cos(angleTowardTarget + angle) * laserDist));
		laser_coord.y = (currentPos.y + (sin(angleTowardTarget + angle) * laserDist));
        
		//calculate intersection point					
		//line function for path Ax+By=C
		A1 = laser_coord.y-currentPos.y;
		B1 = currentPos.x - laser_coord.x;
		C1 = A1*currentPos.x+B1*currentPos.y;
		
		for(int i = 0, j = 1; j<objectPC.points.size(); ++i,++j){
			Point32 pObstacle1 = objectPC.points.at(i);
			Point32 pObstacle2 = objectPC.points.at(j);

			//line function for object edge Ax+By=C
			A2 = pObstacle2.y - pObstacle1.y;
			B2 = pObstacle1.x-pObstacle2.x;
			C2 = A2*pObstacle1.x+B2*pObstacle1.y;				
						
			determant = A1*B2 - A2*B1;
			if(determant == 0){
				continue;						//point has been processed. jump to next edge of the object
			}
			intersection.x = (B2*C1 - B1*C2)/determant;
			intersection.y = (A1*C2 - A2*C1)/determant;
			
			//check if intersection of lines occur within obstacle boundaries;
			if(	min(pObstacle1.x,pObstacle2.x) <= intersection.x && intersection.x <= max(pObstacle1.x, pObstacle2.x) 
			 && min(pObstacle1.y,pObstacle2.y) <= intersection.y && intersection.y <= max(pObstacle1.y, pObstacle2.y)){	
				//check if intersection of lines occur within path boundaries;
				if(	min(currentPos.x,laser_coord.x) <= intersection.x && intersection.x <= max(currentPos.x, laser_coord.x) 
				 && min(currentPos.y,laser_coord.y) <= intersection.y && intersection.y <= max(currentPos.y, laser_coord.y)){
					//path collides with object						
					intersectFound = true;
					break;					
				}						
			}	
		}
		
		if (!intersectFound){
			//ROS_INFO("left  extreme at (%f, %f)", obstacleExtremeLeft.x, obstacleExtremeLeft.y);
			break; // stop at first empty, previous hit was the extreme
		}	
		obstacleExtremeLeft = intersection;
		
	}
	
	//ROS_INFO("Right");
	for(int i = 0; i<=180; ++i){
		float angle = (M_PI*i)/180;
		intersectFound = false;

		laser_coord.x = (currentPos.x + (cos(angleTowardTarget - angle) * laserDist));
		laser_coord.y = (currentPos.y + (sin(angleTowardTarget - angle) * laserDist));
        
		//calculate intersection point					
		//line function for path Ax+By=C
		A1 = laser_coord.y-currentPos.y;
		B1 = currentPos.x - laser_coord.x;
		C1 = A1*currentPos.x+B1*currentPos.y;
		
		for(int i = 0, j = 1; j<objectPC.points.size(); ++i,++j){
			Point32 pObstacle1 = objectPC.points.at(i);
			Point32 pObstacle2 = objectPC.points.at(j);

			//line function for object edge Ax+By=C
			A2 = pObstacle2.y - pObstacle1.y;
			B2 = pObstacle1.x-pObstacle2.x;
			C2 = A2*pObstacle1.x+B2*pObstacle1.y;				
						
			determant = A1*B2 - A2*B1;
			if(determant == 0){
				continue;						//point has been processed. jump to next edge of the object
			}
			intersection.x = (B2*C1 - B1*C2)/determant;
			intersection.y = (A1*C2 - A2*C1)/determant;
			
			//check if intersection of lines occur within obstacle boundaries;
			if(	min(pObstacle1.x,pObstacle2.x) <= intersection.x && intersection.x <= max(pObstacle1.x, pObstacle2.x) 
			 && min(pObstacle1.y,pObstacle2.y) <= intersection.y && intersection.y <= max(pObstacle1.y, pObstacle2.y)){	
				//check if intersection of lines occur within path boundaries;
				if(	min(currentPos.x,laser_coord.x) <= intersection.x && intersection.x <= max(currentPos.x, laser_coord.x) 
				 && min(currentPos.y,laser_coord.y) <= intersection.y && intersection.y <= max(currentPos.y, laser_coord.y)){
					//path collides with object						
					intersectFound = true;
					break;					
				}						
			}	
		}
		
		if (!intersectFound){
			//ROS_INFO("right  extreme at (%f, %f)", obstacleExtremeRight.x, obstacleExtremeRight.y);
			break; // stop at first empty, previous hit was the extreme
		}	
		obstacleExtremeRight = intersection;		
	}
	
	//check for errors with calculated extremes //TODO more checks
	if(compare_Point(obstacleExtremeLeft,collisionPoint) && compare_Point(obstacleExtremeRight,collisionPoint)){
		//ROS_ERROR("no extremes found besides collisionpoint itself");
		return false;
	}
	
	//calc left extreme dist;   
    double extremeLeftPathLength  = calcDistance(targetPos, obstacleExtremeLeft);
	//calc right extreme dist    
	double extremeRightPathLength = calcDistance(targetPos, obstacleExtremeRight);
	
	//determine extreme closest to target, 
	if (extremeLeftPathLength < extremeRightPathLength){
		shortestExtremeToTarget=obstacleExtremeLeft;
	}else{
		shortestExtremeToTarget = obstacleExtremeRight;
		//apply offset sign
        offsetDegree = offsetDegree * -1; //inverse the offset degree
	}

	//calc new target	
	double angleTowardShortestExtreme = atan2((shortestExtremeToTarget.y - currentPos.y), (shortestExtremeToTarget.x - currentPos.x));
	nwTarget.x = 	currentPos.x + cos(angleTowardShortestExtreme + (offsetDegree * M_PI / 180)) * calcDistance(currentPos,shortestExtremeToTarget);
	nwTarget.y =	currentPos.y + sin(angleTowardShortestExtreme + (offsetDegree * M_PI / 180)) * calcDistance(currentPos, shortestExtremeToTarget);
	
	//TODO check nwTarget for errors
	
	mNewWaypoint = nwTarget;
	return true;

}
    
    
// receive the two coordinates that make up the current path and check for colission with known objects.
// objects consist of a set of coordinates that determine the CONVEX outline of the object.
// the line (edge) between two consecutive coordinates can be checked for colission with the path  
// if the path is free, set pathChanged on true and return, otherwise call the recursive bug algorithm 
// and return a new coordinate for the reroute and and return true 	// TODO margin, eg robot size
bool servServerWaypointCheckCallback(skynav_msgs::waypoint_check::Request &req, skynav_msgs::waypoint_check::Response &resp) {

	convexhullFunction();	//Only call convex hull when asking for colission. TODO replace with concave hull function!?
	
	if(mObjectOutlines.empty()){
		//ROS_INFO("No objects to check");
		return true;	//if there are no objects currently known, dont calculate anything and return
	}
	
    Point pPath1 = req.currentPos;
    Point pPath2 = req.targetPos;
    					
	//line function for path Ax+By=C
	double A1 = pPath2.y-pPath1.y;
	double B1 = pPath1.x - pPath2.x;
	double C1 = A1*pPath1.x+B1*pPath1.y;
	
	double A2;
	double B2;
	double C2;
	double determant;
	Point intersection;
	vector<Point> colissions;
	vector<PointCloud> intersect_obstacles;
	bool collisionsFound=false;
	
    for (vector<PointCloud>::iterator outlineIt = mObjectOutlines.begin(); outlineIt != mObjectOutlines.end(); ++outlineIt) {
		if((*outlineIt).points.size() >= 2){
			for(int i = 0, j = 1; j<(*outlineIt).points.size(); ++i,++j){

				Point32 pObstacle1 = (*outlineIt).points.at(i);
				Point32 pObstacle2 = (*outlineIt).points.at(j);
		
				//line function for object edge Ax+By=C
				A2 = pObstacle2.y - pObstacle1.y;
				B2 = pObstacle1.x-pObstacle2.x;
				C2 = A2*pObstacle1.x+B2*pObstacle1.y;				
							
				determant = A1*B2 - A2*B1;
				if(determant == 0){
					//ROS_INFO("no intersections");	//Lines are parallel
					continue;						//point has been processed. jump to next edge of the object
				}
				intersection.x = (B2*C1 - B1*C2)/determant;
				intersection.y = (A1*C2 - A2*C1)/determant;					
				
				//check if intersection of lines occur within obstacle boundaries;
				if(	min(pObstacle1.x,pObstacle2.x) <= intersection.x && intersection.x <= max(pObstacle1.x, pObstacle2.x) 
				 && min(pObstacle1.y,pObstacle2.y) <= intersection.y && intersection.y <= max(pObstacle1.y, pObstacle2.y)){	
					//check if intersection of lines occur within path boundaries;
					if(	min(pPath1.x,pPath2.x) <= intersection.x && intersection.x <= max(pPath1.x, pPath2.x) 
					 && min(pPath1.y,pPath2.y) <= intersection.y && intersection.y <= max(pPath1.y, pPath2.y)){
						//path collides with object						
						collisionsFound = true;
						colissions.push_back(intersection);
						intersect_obstacles.push_back((*outlineIt));
						continue;	//check for other colission points
					}						
				}							
			}
		}
	}
	if(collisionsFound){
		//find relevant colission from set of colissions with pythagoras
		Point relColission;
		PointCloud relObject;
		double colDistance = 4; //TODO maximum laser range
		double newDist;

		for(int i =0; i<colissions.size(); ++i){
			newDist = calcDistance(pPath1,colissions.at(i));
			if( newDist < colDistance){		
				colDistance = newDist;		
				relColission = colissions.at(i);
				relObject = intersect_obstacles.at(i);
			}
		}		
		//ROS_INFO("colission at (%f, %f)", relColission.x, relColission.y);
		
		//calculate new Point newPoint with recursive bug algorithm
		if(recursiveBug(pPath1,pPath2,relColission, relObject)){
			resp.pathChanged = 1;
			ROS_INFO("New waypoint at(%f,%f)",mNewWaypoint.x, mNewWaypoint.y);
			resp.newPos = mNewWaypoint;
			return true;
		}			
		resp.pathChanged = 0;
		ROS_ERROR("Collision detected, but no new waypoint could be calculated");
		return true;
	}	
	//ROS_INFO("No colission");
	resp.pathChanged = 0;
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
