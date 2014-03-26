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
	
	ROS_INFO("received new path from GlobalNav");
	//todo add current location to path check
	for(int wp1 = 0, wp2 = 1; wp2<msg->poses.size(); ++wp1,++wp2){
		skynav_msgs::waypoint_check srv;
		srv.request.currentPos = msg->poses.at(wp1).pose.position;
		srv.request.targetPos  = msg->poses.at(wp2).pose.position;

		if(!servClientWaypointCheck.call(srv)){
			 //insert_after(srv.path,waypoint1);
			 //service to Globalnav for feedback
			 ROS_INFO("colission detected on path from Globalnav");
		}			 
	}
    pubWaypoints.publish(msg);
}

void placeholderGlobalPlannerWaypoints() {

    //temp function to supply placeholder waypoints while global_planner does not exist yet


    // receive waypoints (in this case, hardcoded)
    // should already be received in proper formatted array
    // call waypoint_check service
    // if false as response, publish array to pubWaypoints
    // if true, do nothing

    // temp hardcoded absolute waypoint array
    nav_msgs::Path path;

    path.header.frame_id = "/map";
    path.header.stamp = ros::Time::now();

    double rectSize = 10;

    PoseStamped ps;
    ps.header.frame_id = "/map";
    ps.header.stamp = ros::Time::now();

     //start pose, has orientation (should be the same as robots physical orientation)
    {
        ps.pose.position.x = 0;
        ps.pose.position.y = 0;
        path.poses.push_back(ps);
    }

    { // no orientation needed
        ps.pose.position.x = 0;
        ps.pose.position.y = 0.2*rectSize;
        path.poses.push_back(ps);
    }

    { // no orientation needed
        ps.pose.position.x = rectSize;
        ps.pose.position.y = 0.5*rectSize;
        path.poses.push_back(ps);
    }

    { // no orientation needed
        ps.pose.position.x = rectSize;
        ps.pose.position.y = -0.5*rectSize;
        path.poses.push_back(ps);
    }
    
    { // no orientation needed
        ps.pose.position.x = 0;
        ps.pose.position.y = -0.5*rectSize;
        path.poses.push_back(ps);
    }

    { // end pose, needs orientation
        ps.pose.position.x = 0;
        ps.pose.position.y = 0;
        ps.pose.orientation.z = M_PI;   //180 deg
        path.poses.push_back(ps);
    }

    //pubWaypoints.publish(path);
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
    ros::Subscriber subNavigationState = n.subscribe("navigation_state", 0, subNavigationStateCallback);
    ros::Subscriber subGlobalPlannerWaypoints = n_globalnav.subscribe("waypoints", 32, subGlobalPlannerWaypointsCallback);

    //service
    servClientWaypointCheck = n_localnav.serviceClient<skynav_msgs::waypoint_check>("path_check");

    ros::Rate loop_rate(1);    
    
	while(ros::ok()){
	//ros::Duration(10).sleep();
      
	//placeholderGlobalPlannerWaypoints();
	
    loop_rate.sleep();
    
    ros::spin();
	}


    return 0;
}


