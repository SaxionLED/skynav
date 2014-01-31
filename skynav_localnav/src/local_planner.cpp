#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <skynav_msgs/waypoint_check.h>
#include <skynav_msgs/Object.h>
#include <geometry_msgs/Point.h>

using namespace geometry_msgs;
using namespace std;

vector<skynav_msgs::Object> mObstacles;

void subObstaclesCallback(const skynav_msgs::Object::ConstPtr& msg) {

    //    mObstacles.push_back(msg);
}

bool servServerWaypointCheckCallback(skynav_msgs::waypoint_check::Request &req, skynav_msgs::waypoint_check::Response &response) {

    // take req.waypoints and check if the path intersects with obstacles, if not: return true else publish new path and return false

    // obstacles consist of 2 coordinates, the line between them is the obstacle as seen by the sensors //TODO better design: maybe an array of coordinates that are near eachother, creating an ellipse? 



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

    /// TEMP
    // add a test obstacle

    /// END TEMP

    ros::init(argc, argv, "local_planner");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");

    //pubs
    ros::Publisher pubWaypoints = n_control.advertise<nav_msgs::Path>("checked_waypoints", 32);

    //subs
    ros::Subscriber subObstacles = n.subscribe("obstacles", 256, subObstaclesCallback);

    //services
    ros::ServiceServer servServerWaypointCheck = n.advertiseService("path_check", servServerWaypointCheckCallback);

    ros::spin();

    return 0;
}