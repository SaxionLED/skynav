#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <skynav_msgs/waypoint_check.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/Object.h>
#include <skynav_msgs/Objects.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <boost/optional.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/surface/concave_hull.h>



#define ROBOTRADIUS 		0.5 	//the radius of the robot. TODO get this from somewhere robot dependent
#define MAX_SENSORDIST 		4		//the outer range of the sensors

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std;

ros::Publisher pubObjectOutlines, pubNavigationState, pubObjectExtremes;
ros::Subscriber subObstacles, subNavigationState, subAbsoluteTargetPose;

ros::ServiceServer servServerWaypointCheck;
ros::ServiceClient servClientCurrentPose;

vector<PointCloud> mObstacles;					//pointclouds that make up the objects
vector<PointCloud> mObjectOutlines;				//outline of known objects, calculated by the convexhull function

PoseStamped mCurrentAbsoluteTargetPose;			//the current next target pose, as published by motion_control
uint8_t mControl_NavigationState;				//the state which motion_control is in.

typedef boost::optional<Point> optionPoint;		//use boost::optional for boolean return when no viable return value can be returned.
//typedef pcl::PointCloud<pcl::PointXYZ> PclPointCloud;
//typedef pcl::ConcaveHull<pcl::PointXYZ> PclConcaveHull;
//typedef pcl::PointXYZ PclPoint;

//compare function for sorting algorithm
bool compare(const Point32 &p, const Point32 &q){
	return p.x < q.x || (p.x == q.x && p.y < q.y);
}

//compare both points to each other
bool compare_Point(Point p, Point q){
	if((p.x == q.x) && (p.y = q.y)){
		return true;
	}
	return false;
}

//request the current pose
Pose getCurrentPose() {
	try{
		Pose currentPose;

		skynav_msgs::current_pose poseService;

		if (servClientCurrentPose.call(poseService)) {

			currentPose = poseService.response.pose;

		} else {
			ROS_ERROR("Failed to call current_pose service from local_planner");
			ros::shutdown();
		}
		return currentPose;
	}catch(exception& e){
		ROS_ERROR("exception caught: %s",e.what());
		ros::shutdown();
	}
}

//calculate distance between two point with use of pythagoras
double calcDistance(Point a, Point b){
	return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
}

//truncate values (in meters) to certain precision
float truncateValue(const float value){
	return floorf(value*1000)/1000; //mm
	//return floorf(value*100)/100; //cm 
}

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double cross(const Point32 &O, const Point32 &A, const Point32 &B){
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

//TODO Convex hull is not correct when encountering concave objects like walls!!
 
// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Returns a pointcloud containing a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
PointCloud convex_hull(PointCloud data){
	vector<Point32> P = data.points;
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

PointCloud concave_hull(PointCloud data){
	PointCloud PC;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	
	for(vector<Point32>::iterator it = data.points.begin(); it!= data.points.end(); ++it){
		cloud_in->push_back(pcl::PointXYZ((*it).x,(*it).y,0));
	}
	
	chull.setInputCloud (cloud_in);
	chull.setAlpha (0.5);
	chull.reconstruct (*cloud_out);
	
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_out->begin(); it!= cloud_out->end(); ++it){
		Point32 p;
		p.x=(*it).x;
		p.y=(*it).y;
		PC.points.push_back(p);		
	}
	PC.header.stamp = ros::Time::now();
    PC.header.frame_id = "/map";
	return PC;
}

//function to call convex hull determination for each object
void convexhullFunction(){	
	mObjectOutlines.clear();	//clear the outlines and replace with new data
	
	for(vector<PointCloud>::iterator it = mObstacles.begin(); it!= mObstacles.end(); ++it){
		mObjectOutlines.push_back(concave_hull((*it)));
	}
	
	for(vector<PointCloud>::iterator outlineIt = mObjectOutlines.begin(); outlineIt!= mObjectOutlines.end(); ++outlineIt){
		pubObjectOutlines.publish((*outlineIt) );
	}	
}

void subNavigationStateCallback(const std_msgs::UInt8& msg ){
	mControl_NavigationState = msg.data;
	//ROS_INFO("localnav NavigationState: %d", msg.data);	
}

void subAbsoluteTargetPoseCallback(const PoseStamped& msg){
	mCurrentAbsoluteTargetPose = msg;
	//ROS_INFO("Current Localnav target pose: (%f,%f)", msg.pose.position.x, msg.pose.position.y);
}

//receive a custom message, containing all up to date objects within sensor range, from obstacle detector
void subObstaclesCallback(const skynav_msgs::Objects::ConstPtr& msg) {
	mObstacles.clear();

	for(vector<PointCloud>::const_iterator objectIt = msg->objects.begin(); objectIt!= msg->objects.end(); ++objectIt){
        if((*objectIt).points.size()>5){
			mObstacles.push_back((*objectIt));
		}
	}
}


//recursive bug algorithm for object avoidance
optionPoint recursiveBug(const Point currentPos,const Point targetPos, const Point collisionPoint, const PointCloud objectPC){

	if(objectPC.points.empty()){
		ROS_WARN("Object size is zero, stopping recursive bug");
		return optionPoint();	//return false;
	}
	bool foundNew = false;
		
	Point nwTarget;
	Point obstacleExtremeLeft;
	Point obstacleExtremeRight;
	Point shortestExtremeToTarget;
	Point intersection = collisionPoint;
	Point laser_coord;
	
	//double laserDist = 4;
	double laserDist = calcDistance(currentPos, targetPos);	//TODO hack for the first colission check when path is received by waypoint_filter.

	double angleTowardTarget = atan2((targetPos.y - currentPos.y),(targetPos.x - currentPos.x));

	double A1,B1,C1; 	
	double A2,B2,C2;
	double determant;
	bool intersectFound;

	//left side of scan radius
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
	
	//right side of scan radius
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
	if((compare_Point(obstacleExtremeLeft,collisionPoint)) && (compare_Point(obstacleExtremeRight,collisionPoint))){
		ROS_ERROR("no extremes found besides collisionpoint itself");
		return optionPoint();	//return false;
	}
	
	{
		//DEBUG publish extremes for visual debugging purposes
		PointCloud obExtremes;
		obExtremes.header.stamp = ros::Time::now();
		obExtremes.header.frame_id = "/map";
		Point32 left;
		Point32 right;
		left.x = obstacleExtremeLeft.x;
		left.y = obstacleExtremeLeft.y;
		left.z = 0.20;
		right.x = obstacleExtremeRight.x;
		right.y = obstacleExtremeRight.y;
		right.z = 0.20;
		obExtremes.points.push_back(left);
		obExtremes.points.push_back(right);
		pubObjectExtremes.publish(obExtremes);
	}
	
	//calc path via left extreme   
    double extremeLeftPathLength  = calcDistance(targetPos, obstacleExtremeLeft) + calcDistance(currentPos, obstacleExtremeLeft);
	//calc path via right extreme    
	double extremeRightPathLength = calcDistance(targetPos, obstacleExtremeRight) + calcDistance(currentPos, obstacleExtremeRight);;
	
	int angleSign;
	//determine extreme closest to target, 
	if (extremeLeftPathLength < extremeRightPathLength){
		shortestExtremeToTarget=obstacleExtremeLeft;
		angleSign = 1;
	}else{
		shortestExtremeToTarget = obstacleExtremeRight;
		//apply offset sign
		angleSign = -1;
	}
	
	//calculate new waypoint based on robot size and trigenomitry functions in 7 steps
	double offsetRadius = 0.5 * ROBOTRADIUS; //half the robot radius
	double distNewP;
	double distNewP2;
	double angleCollision;
	double angleNewP;
	double angleEx;
	double angleExNewP;
	
	//I determine distances between known and detected points
	double distCollision = calcDistance(currentPos,collisionPoint);
	double distExtreme = calcDistance(currentPos, shortestExtremeToTarget);
	double distExtrCollis = calcDistance(collisionPoint, shortestExtremeToTarget);
	double distNwPCollis = (distExtrCollis + offsetRadius);
	
	double distCollision2 = pow(distCollision,2);
	double distExtreme2 = pow(distExtreme,2);
	double distExtrCollis2 = pow(distExtrCollis,2);
	double distNwPCollis2 = pow(distNwPCollis,2); 

	//II determine the angle between the edges at the colission
	if(distExtrCollis == 0){ //prevent nan values
		angleCollision = 1;
	}else{
		angleCollision = acos( (distExtrCollis2 + distCollision2 - distExtreme2 ) / (2 * (distExtrCollis * distExtreme) ) );
	}
	
	//III determine the distance to the new waypoint
	distNewP = sqrt(distNwPCollis2 + distCollision2 - (2 * distNwPCollis * distCollision * cos(angleCollision)) );
	distNewP2 = pow(distNewP,2);
	
	//IV determine the angle between the current position and the extreme of the object
	angleEx = atan2( (shortestExtremeToTarget.y - currentPos.y), (shortestExtremeToTarget.x - currentPos.x) );
	
	//V determine the angle between the edges to extreme and the new waypoint
	angleExNewP = acos( (distExtreme2 + distNewP2 - offsetRadius)/(2 * (distExtreme * distNewP) ) ) * angleSign;
	
	//VI calculate the angle to the new waypoint
	if(distExtrCollis == 0){ //prevent nan values
		angleNewP = angleExNewP;
	}else{
		angleNewP = angleEx + angleExNewP;
	}
	
	//VII calculate x and y coordinates of the new waypoint, based on distance and angle from current pos
	nwTarget.x = truncateValue(	currentPos.x + cos(angleNewP) * distNewP	);
	nwTarget.y = truncateValue(	currentPos.y + sin(angleNewP) * distNewP	);

	if(isnan(nwTarget.x) || isnan(nwTarget.y)){
		return boost::optional<Point>();		
		ROS_ERROR("Targetpose is not valid, skipping targetPose!"); 	
	}
	return boost::optional<Point>(nwTarget);	//return true with new target
}

//function to determine if there is a colission, where, and (if needed) call for a new waypoint calculation.
//if recursiveBugNeeded is true, the return value is the new waypoint calculated with the recursivebug algorithm, if false: the return is the collisionpoint itself
optionPoint waypointCheck(Point pPath1, Point pPath2, bool recursiveBugNeeded){
	
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
		//find relevant colission from set of colissions
		Point relColission;
		PointCloud relObject;
		double colDistance = 9999; // initiate distance at "far from robot" TODO set certain distance
		double newDist;

		for(int i =0; i<colissions.size(); ++i){
			newDist = calcDistance(pPath1,colissions.at(i));
			if( newDist < colDistance ){		
				colDistance = newDist;		
				relColission = colissions.at(i);
				relObject = intersect_obstacles.at(i);
			}
		}		
		//ROS_INFO("colission at (%f, %f), %d", relColission.x, relColission.y, relObject.points.size());
		
		if(recursiveBugNeeded){
			//calculate new Point newPoint with recursive bug algorithm
			optionPoint newPoint;
			if((newPoint = recursiveBug(pPath1, pPath2, relColission, relObject))){
				//ROS_INFO("newpoint at (%f, %f)", (*newPoint).x, (*newPoint).y);
				return optionPoint(newPoint);  //return true, with new waypoint 		
			}			
			ROS_ERROR("Collision detected, but no new waypoint could be calculated");
			return optionPoint();  //return false 
		}
		//recursive bug is not neccesary, only collisionpoint is asked
		return optionPoint(relColission); //return true with colissionpoint		
	}
	//no colissions found
	//ROS_INFO("No collisions found on current track");
	return optionPoint();  //return false 
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
		resp.pathChanged = 0;
		return true;	//if there are no objects currently known, dont calculate anything and return
	}
	//call the colissioncheck algorithm. if colissioncheck is true, new waypoint is returned, else there is no colission	
	optionPoint newPoint;
	if((newPoint = waypointCheck(req.currentPos, req.targetPos, true))){		//call for colissioncheck with recursiveBug active
		resp.pathChanged = 1;
		resp.newPos = *newPoint;
		ROS_INFO("New waypoint at(%f,%f)",resp.newPos.x, resp.newPos.y);
		return true;
	}
	resp.pathChanged = 0;
	return true;
}

//send a new NAVIGATION_STATE as interrupt to motion_control
void interruptNavigationState(const std_msgs::UInt8 pubmsg){
	mControl_NavigationState = pubmsg.data;
	pubNavigationState.publish(pubmsg);
	
	return;	
}

// call colissioncheck function. if colission occurs, publish interrupt navigationstate for motion_control 
void collisionCheck(){
	
	convexhullFunction();	//Only call convex hull when asking for colissioncheck. TODO replace with concave hull function!?	
	
	if(mObjectOutlines.empty()){
		return;				// dont calculate anything and return
	}
	
	Pose currentPose = getCurrentPose();
	Pose targetPose = mCurrentAbsoluteTargetPose.pose;

	optionPoint collision;
	if((collision = waypointCheck(currentPose.position, targetPose.position, false))){
		
		if(calcDistance(currentPose.position, *collision) <= (0.5*MAX_SENSORDIST) ){
			std_msgs::UInt8 msg;		
			msg.data = 6;		//TODO replace with NAVIGATION_STATE state;
			interruptNavigationState(msg);
			//ROS_INFO("Collision at: %f,%f", (*collision).x, (*collision).y );
		}
	}	 
	return;
}
 
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");
    ros::NodeHandle n_slam("/slam");

    //pubs
    pubObjectOutlines = n.advertise<PointCloud>("objectOutlines", 10);
    pubNavigationState = n_control.advertise<std_msgs::UInt8>("navigation_state_interrupt", 0);
    pubObjectExtremes = n.advertise<PointCloud>("obstacleExtremes",2);
    

    //subs
    subObstacles = n.subscribe("obstacles", 10, subObstaclesCallback);
	subNavigationState= n_control.subscribe("navigation_state",0,subNavigationStateCallback);
	subAbsoluteTargetPose = n_control.subscribe("target_pose",0,subAbsoluteTargetPoseCallback);
	
	//services
    servServerWaypointCheck = n.advertiseService("path_check", servServerWaypointCheckCallback);
    
    
	servClientCurrentPose = n_slam.serviceClient<skynav_msgs::current_pose>("current_pose");

	ros::Rate loop_rate(1); // loop every 1s

	while(ros::ok){
		
		ros::spinOnce();

		if(mControl_NavigationState == 1){
			collisionCheck();
			loop_rate.sleep();
		}	
	}

    return 0;
}
