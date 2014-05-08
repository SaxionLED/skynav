#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <stdbool.h>
#include <nav_msgs/Path.h>
#include <skynav_msgs/waypoint_check.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace geometry_msgs;

ros::Publisher pubWaypoints;
ros::Publisher pubDummyPath;
ros::ServiceClient servClientWaypointCheck;

void subNavigationStateCallback(const std_msgs::UInt8::ConstPtr& msg) {

    //    ROS_INFO("recv: %d", msg->data);
}

void subGlobalPlannerWaypointsCallback(const nav_msgs::Path::ConstPtr& msg) {
	ROS_INFO("received new path from GlobalNav, check for known obstacles");
	skynav_msgs::waypoint_check srv;
	bool changedPath = false;
	
	nav_msgs::Path nwPath;
	nwPath.header = msg->header;
	vector<PoseStamped> nwWaypoints = msg->poses;
	if(nwWaypoints.size()>1){
		for(int wp0 = 0, wp1 = 1; wp1<nwWaypoints.size(); ++wp0,++wp1){
			srv.request.currentPos = nwWaypoints.at(wp0).pose.position;
			srv.request.targetPos  = nwWaypoints.at(wp1).pose.position;

			if(servClientWaypointCheck.call(srv)){
				if(srv.response.pathChanged){
					changedPath=true;
					vector<PoseStamped>::iterator pathIt = nwWaypoints.begin();	

					PoseStamped ps;
					ps.header.frame_id = "/map";
					ps.header.stamp = ros::Time::now();				
					ps.pose.position = srv.response.newPos;
					
					//insert new waypoint into the list of waypoints
					nwWaypoints.insert(pathIt+(wp0+1),ps);		 		
				}
			}else{
				ROS_ERROR("Failed to call waypoint check service from waypoint_filter.?");
				return;
				//ros::shutdown();
			}						 
		}
	}
	if(changedPath){
		nwPath.poses = nwWaypoints;
		ROS_INFO("Colission(s) detected and detour calculated");
		pubWaypoints.publish(nwPath);
	}else{		
		pubWaypoints.publish(msg);
	}
}

//temp function to supply placeholder waypoints for testing without available globalnav modules
//output a hardcoded absolute waypoint array
void placeholderGlobalPlannerWaypoints() {
	

    ROS_INFO("Placeholder path generated");
    nav_msgs::Path path;

    path.header.frame_id = "/map";
    path.header.stamp = ros::Time::now();
    
    PoseStamped ps;
    ps.header.frame_id = "/map";
    ps.header.stamp = ros::Time::now();

     //start pose, has orientation 
	{
        ps.pose.position.x = 0;
        ps.pose.position.y = 0;
        path.poses.push_back(ps);
    }
    { // end pose, needs orientation
        ps.pose.position.x = 0;
        ps.pose.position.y = 0;
        ps.pose.orientation.z = M_PI;   //180 deg
        path.poses.push_back(ps);
    }

    pubDummyPath.publish(path); //publish dummy path data on waypoints topic
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "waypoint_filter");

    ros::NodeHandle n("/control");
    ros::NodeHandle n_localnav("/localnav");
    ros::NodeHandle n_globalnav("/globalnav");

    //pubs
    ros::Publisher pubNavigationState = n.advertise<std_msgs::UInt8>("navigation_state", 0);
    pubWaypoints = n.advertise<nav_msgs::Path>("checked_waypoints", 32);
	pubDummyPath = n_globalnav.advertise<nav_msgs::Path>("waypoints", 32);

    //subs
    //ros::Subscriber subNavigationState = n.subscribe("navigation_state", 0, subNavigationStateCallback);
    ros::Subscriber subGlobalPlannerWaypoints = n_globalnav.subscribe("waypoints", 32, subGlobalPlannerWaypointsCallback);

    //service
    servClientWaypointCheck = n_localnav.serviceClient<skynav_msgs::waypoint_check>("path_check");

   	//ros::Duration(15).sleep();
      
	//placeholderGlobalPlannerWaypoints();
	
	ros::spin();
	
    return 0;
}


