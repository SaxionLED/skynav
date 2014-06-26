#include <ros/ros.h>

#include <skynav_msgs/waypoint_check.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/PointCloudVector.h>

#include <std_msgs/UInt8.h>

#include "local_planner_lib.h"

using namespace std;
using namespace geometry_msgs;

ros::Publisher pubObjectOutlines, pubNavigationState, pubClusterOutlines;
ros::Subscriber subNavigationState, subAbsoluteTargetPose, subPointCloudSensorData, subPCVector;

ros::ServiceServer servServerPclWaypointCheck;
ros::ServiceClient servClientCurrentPose;

Pcl2Vector mClusters;			//set of clusters(objects)
Pcl2Vector mClusterOutlines;	//set of cluster outlines

PoseStamped mCurrentAbsoluteTargetPose;			//the current next target pose, as published by motion_control
uint8_t mControl_NavigationState;				//the state which motion_control is in.

boost::mutex mMutex;


//request the current pose
Pose getCurrentPose() 
{
	try{
		Pose currentPose;

		skynav_msgs::current_pose poseService;

		if (servClientCurrentPose.call(poseService)) 
		{

			currentPose = poseService.response.pose;

		} else {
			ROS_ERROR("Failed to call current_pose service from local_planner");
			ros::shutdown();
		}
		return currentPose;
	}
	catch(exception& e)
	{
		ROS_ERROR("exception caught: %s",e.what());
		ros::shutdown();
	}
}


//receive a pcl::PCLPointCloud2 and perform some actions on it
void subPointCloudDataCallback(const pcl::PCLPointCloud2ConstPtr& msg)
{
	//TODO
}


void subPCVectorCallback(const skynav_msgs::PointCloudVector::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(mMutex);

	mClusters.clear();
	
	for(vector<sensor_msgs::PointCloud2>::const_iterator it = msg->clouds.begin();it!=msg->clouds.end();++it){
		 pcl::PCLPointCloud2 pclPC;
		 pcl_conversions::toPCL((*it),pclPC);
		 
		 mClusters.push_back(pclPC);
	}	
}


void pclPublishHull()
{
	boost::mutex::scoped_lock lock(mMutex);

	if(!mClusterOutlines.empty())
	{
		for(Pcl2Vector::iterator outlineIt = mClusterOutlines.begin(); outlineIt!= mClusterOutlines.end(); ++outlineIt)
		{
			pubClusterOutlines.publish((*outlineIt) );
		}	
	}
}


void pclHullFunction()
{
	boost::mutex::scoped_lock lock(mMutex);

	mClusterOutlines.clear();

	for(Pcl2Vector::iterator it = mClusters.begin(); it!= mClusters.end(); ++it)
	{
		if((*it).width * (*it).height > 40)
		{
			mClusterOutlines.push_back(pclConcave_hull((*it)));
		} 
		else 
		if((*it).width * (*it).height <= 40 && (*it).width * (*it).height > 10 )
		{
			mClusterOutlines.push_back(pclConvex_hull((*it)));
		}
		else 
		if((*it).width * (*it).height >=2)
		{
			mClusterOutlines.push_back((*it));
		}
	}
	//ROS_INFO("cluster_outlines; %d", mClusterOutlines.size());
	lock.unlock();

	pclPublishHull();	
}


void subNavigationStateCallback(const std_msgs::UInt8& msg ){
	mControl_NavigationState = msg.data;
}


void subAbsoluteTargetPoseCallback(const PoseStamped& msg){
	mCurrentAbsoluteTargetPose = msg;
}


// receive the two coordinates that make up the current path and check for colission with known objects.
// objects consist of a set of coordinates that determine the CONVEX outline of the object.
// the line (edge) between two consecutive coordinates can be checked for colission with the path  
// if the path is free, set pathChanged on true and return, otherwise call the recursive bug algorithm 
// and return a new coordinate for the reroute and and return true 	// TODO margin, eg robot size
bool servServerPclWaypointCheckCallback(skynav_msgs::waypoint_check::Request &req, skynav_msgs::waypoint_check::Response &resp) {

	//Only call when asking for colission.
	pclHullFunction();
	
	if(mClusterOutlines.empty()){
		resp.pathChanged = 0;
		return true;	//dont calculate anything and return
	}
	pcl::PointXYZ currentPCLPoint;
	pcl::PointXYZ targetPCLPoint;
	
	currentPCLPoint.x	= req.currentPos.x;
	currentPCLPoint.y 	= req.currentPos.y;
	targetPCLPoint.x 	= req.targetPos.x;
	targetPCLPoint.y 	= req.targetPos.y;
	
	//if colissioncheck is true, new waypoint is returned, else there is no colission	
	pclOptionPoint newPoint;
	if((newPoint = pclWaypointCheck(currentPCLPoint, targetPCLPoint, mClusterOutlines, true))){		//call for colissioncheck with recursiveBug active
		resp.pathChanged = 1;
		
		geometry_msgs::Point nwTargetPoint;
		nwTargetPoint.x = (*newPoint).x;
		nwTargetPoint.y = (*newPoint).y;
		resp.newPos = nwTargetPoint;
		
		ROS_INFO("New waypoint at(%f,%f)",resp.newPos.x, resp.newPos.y);
		return true;
	}
	resp.pathChanged = 0;
	return true;
}


//send a new NAVIGATION_STATE as interrupt to motion_control
void publishInterruptNavigationState(const std_msgs::UInt8 pubmsg){
	mControl_NavigationState = pubmsg.data;
	pubNavigationState.publish(pubmsg);

	return;	
}


// call colissioncheck function. if colission occurs, publish interrupt navigationstate for motion_control 
void pclCollisionCheck(){
	
	pclHullFunction();	//Only call when asking for colissioncheck. 

	if(mClusterOutlines.empty()){
		ROS_INFO("no cluster outline");
		return;				// dont calculate anything and return
	}

	Pose currentPose 	= getCurrentPose();
	Pose targetPose 	= mCurrentAbsoluteTargetPose.pose;
	
	pcl::PointXYZ currentPCLPoint;
	pcl::PointXYZ targetPCLPoint;
	
	currentPCLPoint.x	= currentPose.position.x;
	currentPCLPoint.y 	= currentPose.position.y;
	targetPCLPoint.x 	= targetPose.position.x;
	targetPCLPoint.y 	= targetPose.position.y;
	
	pclOptionPoint pclCollision;
	if((pclCollision = pclWaypointCheck(currentPCLPoint, targetPCLPoint, mClusterOutlines, false)))
	{
		if(pclCalcDistance(currentPCLPoint, *pclCollision) <= (0.5*MAX_SENSORDIST) )
		{
			std_msgs::UInt8 msg;		
			msg.data = 6;						//TODO replace with NAVIGATION_STATE state;
			publishInterruptNavigationState(msg);
			ROS_INFO("Collision at: %f,%f", (*pclCollision).x, (*pclCollision).y );
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
	pubClusterOutlines = n.advertise<pcl::PCLPointCloud2>("clusterOutlines", 10);
    pubNavigationState = n_control.advertise<std_msgs::UInt8>("navigation_state_interrupt", 0);    
    
    //subs
	subNavigationState= n_control.subscribe("navigation_state",0,subNavigationStateCallback);
	subAbsoluteTargetPose = n_control.subscribe("target_pose",0,subAbsoluteTargetPoseCallback);
	subPCVector = n.subscribe("pointcloudVector",1, subPCVectorCallback);
    
	//subPointCloudSensorData = n.subscribe("pointCloudData",1,subPointCloudDataCallback);

	//services
	servServerPclWaypointCheck = n.advertiseService("path_check", servServerPclWaypointCheckCallback);

    
	servClientCurrentPose = n_slam.serviceClient<skynav_msgs::current_pose>("current_pose");

	ros::Rate loop_rate(1);

	while(ros::ok){
		
		ros::spinOnce();

		if(mControl_NavigationState == 1){
			pclCollisionCheck();

			loop_rate.sleep();
		}	
	}

    return 0;
}
