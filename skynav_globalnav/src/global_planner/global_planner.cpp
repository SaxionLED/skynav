/*
 * global_planner.cpp
 *
 * november 2013
 *
 * Jurriaan Voskes
 */
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <string>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "graph.h"

//custom msgs
#include <skynav_msgs/navigation_state.h>
#include <skynav_msgs/environment_info.h>
#include <skynav_msgs/environment_srv.h>
#include <skynav_msgs/user_init.h>
#include <skynav_msgs/path_query_srv.h>
#include <skynav_msgs/edit_fixedWPs_srv.h>

namespace navigation_states
{
enum navigation_state
{
  IDLE, RUN, REACHED, UNREACHABLE, ERROR
};
}
namespace planner_state
{
enum state_
{
  Init, Query, Idle, Stop, Error
};
}

const int NO_LOOP = 0;

/*
 * Global planner main class
 */
class GlobalPlanner
{
private:
  std::string node_name_;
  ros::NodeHandle* node_;
  ros::NodeHandle* node_control_;
  int loop_rate_;

  navigation_states::navigation_state navigation_state_;
  planner_state::state_ planner_state_;

  ros::Subscriber navigation_state_sub_;
  ros::Subscriber environment_sub_;
  ros::Subscriber user_init_sub_;

  ros::Publisher waypoints_pub_;

  ros::ServiceClient getEnvironmentInfo_;
  ros::ServiceServer pathQuery_srv_;
  ros::ServiceServer fixedWaypoints_srv_;

//Graph* p_mGlobalGraph;
  Graph* p_mFullGraph;
  MapData* p_mMapData;

  bool initDone_;

  void Init();
  bool Query(unsigned int xStart, unsigned int yStart, float thStart, unsigned int xTarget, unsigned int yTarget,
             float thTarget);
  void Stop();
  void Error();
  void ReInit();

  bool respond_pathQuery(skynav_msgs::path_query_srv::Request &req,
                         skynav_msgs::path_query_srv::Response &res);
  bool respond_fixedWaypoints(skynav_msgs::edit_fixedWPs_srv::Request &req,
                              skynav_msgs::edit_fixedWPs_srv::Response &res);

public:
  GlobalPlanner(std::string node_name, int loop_rate);
  virtual ~GlobalPlanner()
  {
    delete node_;
    delete node_control_;
   
	if (p_mMapData)
    {
      delete p_mMapData;
    }
    if (p_mFullGraph)
    {
      delete p_mFullGraph;
    }

  }
  ;
  void navigation_stateCallback(const skynav_msgs::navigation_state::ConstPtr& msg);
  void user_InitCallback(const skynav_msgs::user_init::ConstPtr& msg);

  bool outputWaypoints(std::vector<Node*> &v_pPath);
  bool getEnvironmentData();
  void loop();
};

GlobalPlanner::GlobalPlanner(std::string node_name, int loop_rate) :
    node_name_(node_name), loop_rate_(loop_rate)
{
  node_ = new ros::NodeHandle("/globalnav");
  node_control_ = new ros::NodeHandle("/control");

//service servers
  pathQuery_srv_ = node_->advertiseService("path_query", &GlobalPlanner::respond_pathQuery, this);
  fixedWaypoints_srv_ = node_->advertiseService("update_fixed_waypoints", &GlobalPlanner::respond_fixedWaypoints, this);
//service client
  getEnvironmentInfo_ = node_->serviceClient<skynav_msgs::environment_srv>("environment_req");
//publisher
  waypoints_pub_ = node_->advertise<nav_msgs::Path>("waypoints", 10);
//subscriber  
  navigation_state_sub_ = node_control_->subscribe("navigation_state", 10, &GlobalPlanner::navigation_stateCallback, this);
  user_init_sub_ = node_->subscribe("user_init", 10, &GlobalPlanner::user_InitCallback, this);

  planner_state_ = planner_state::Idle;
  navigation_state_ = navigation_states::RUN;

  p_mFullGraph = NULL;
  p_mMapData = NULL;

  initDone_ = false;
}
/*
 * receive a start and target location, query the known graph
 */
bool GlobalPlanner::respond_pathQuery(skynav_msgs::path_query_srv::Request &req,
                                      skynav_msgs::path_query_srv::Response &res)
{
  if (req.request)
  {
    ROS_INFO("query request");

    if (Query(req.startPose.x, req.startPose.y, req.startPose.theta, req.targetPose.x, req.targetPose.y,
              req.targetPose.theta))
    {
      res.response = 1;
      return true;
    }
  }
  ROS_ERROR("Error with received query");
  return false;
}
/*
 * receive an updated list with fixed waypoints
 */
bool GlobalPlanner::respond_fixedWaypoints(skynav_msgs::edit_fixedWPs_srv::Request &req,
                                           skynav_msgs::edit_fixedWPs_srv::Response &res)
{
  if (req.request)
  {
    if (req.waypoints.size() > 0)
    {

      std::vector<Node*> tmp_fxWps;
      for (int i = 0; i < req.waypoints.size(); i++)
      {
        Node* pNode = new Node(req.waypoints[i].x, req.waypoints[i].y, i);
        tmp_fxWps.push_back(pNode);

      }
      this->p_mMapData->updateFixedWPs(tmp_fxWps);
      tmp_fxWps.clear();

      if (p_mFullGraph)
      {
        p_mFullGraph->updateFixedWaypoints();
      }
    }
    return true;
  }
  else
  {
    ROS_ERROR("Upate of Fixed Waypoints failed");
    return false;
  }
}

/*
 * change the navigation_state
 */
void GlobalPlanner::navigation_stateCallback(const skynav_msgs::navigation_state::ConstPtr& msg)
{
  //TODO navigation_state_ = msg->state;
  ROS_INFO("Navigation state changed");
}

/*
 * re-init the full graph when the user sends a re-init msg
 */
void GlobalPlanner::user_InitCallback(const skynav_msgs::user_init::ConstPtr& msg)
{
  if (msg->state)
  {
    ROS_INFO("Re-init");
    ReInit();
  }
}
/*
 * output the waypoints that make up the path on the roadmap
 */
bool GlobalPlanner::outputWaypoints(std::vector<Node*> &v_pPath)
{
  /*
   * todo conversion function from units to real world coordinates based on scale.
   * '/scale' in functions need to be replaced with actual scaling factor
   * output is in meters.
   */

  float scale = 100;

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/map";

  /*
   * use a reverse iterator to traverse the list of waypoints from last to first entry.
   * list is reversed because the algorithm returns a backward path from target to start node
   */
  for (std::vector<Node*>::reverse_iterator rit = v_pPath.rbegin(); rit != v_pPath.rend(); ++rit)
  {
    geometry_msgs::PoseStamped ps;

    ps.pose.position.x = (float((*rit)->getXpos()) / scale);
    ps.pose.position.y = (float((*rit)->getYpos()) / scale);
    ps.pose.orientation.z = float((*rit)->getTheta()); //orientation of the robot

    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "/map";

    msg.poses.push_back(ps);

  }
  waypoints_pub_.publish(msg);
  return true;
}

//service call to get environment info from environment ROSnode
bool GlobalPlanner::getEnvironmentData()
{
  skynav_msgs::environment_srv srv;
  srv.request.request = 1;
  if (getEnvironmentInfo_.call(srv))
  {
    ROS_INFO("received environment data");

    //create new mapdata
    if (p_mMapData)
    {
      delete p_mMapData;
      //if mapdata already exists, remove old one, create new
    }
    p_mMapData = new MapData(srv.response.environment.map.info.width, srv.response.environment.map.info.height,
                             srv.response.environment.map.info.resolution);

    std::vector<int> tmp_data;
    tmp_data.resize(srv.response.environment.map.info.width * srv.response.environment.map.info.height);
    for (int i = 0; i < srv.response.environment.map.data.size(); i++)
    {
      tmp_data[i] = srv.response.environment.map.data[i];
    }
    p_mMapData->parseOccupancyList(tmp_data);
    tmp_data.clear();

    /*
     * add fixed waypoints to mapdata
     */
    std::vector<Node*> tmp_fxWps;
    for (int i = 0; i < srv.response.environment.fixed_waypoints.size(); i++)
    {
      Node* pNode = new Node(srv.response.environment.fixed_waypoints[i].x,
                             srv.response.environment.fixed_waypoints[i].y, i);
      tmp_fxWps.push_back(pNode);
    }
    this->p_mMapData->addFixedWPs(tmp_fxWps);
    tmp_fxWps.clear();
    return true;
  }
  return false;
}

//re-init the map and graph
void GlobalPlanner::ReInit()
{
  this->initDone_ = false;
  Init();
}

/*
 * retrieve map-data from environment and create a graph based on this data.
 */
void GlobalPlanner::Init()
{
  if (!initDone_)
  {
    planner_state_ = planner_state::Init;
    ROS_INFO("initializing");
    if (getEnvironmentData())
    {
      /*
       * TODO create global Graph based on connected areas in the environment. this can be rooms, hallways, difficult traverseable areas or special places like the coffeemachine for instance.
       * p_mGlobalGraph = new Graph
       */

      //create local level graph, based on the known mapdata and a randomized graph generator
      p_mFullGraph = new Graph(p_mMapData);
      ROS_INFO("init done");
      this->initDone_ = true;
    }
    else
    {
      ROS_ERROR("no environment info available");
      planner_state_ = planner_state::Idle;
      return;
    }
  }
  else
  {
    ROS_INFO("init already done");
  }
  planner_state_ = planner_state::Idle;

}
/*
 * query a graph based on start and target coordinates in carthesian space
 */
bool GlobalPlanner::Query(unsigned int xStart, unsigned int yStart, float thStart, unsigned int xTarget,
                          unsigned int yTarget, float thTarget)
{
  if (xStart == xTarget && yStart == yTarget)
  {
    ROS_ERROR("start is target location, nothing to be done");
    return false;
  }
  if (!initDone_)
  {
    Init();
  }
  if (initDone_)
  {
    ROS_INFO("query");
    this->planner_state_ = planner_state::Query;
    /*
     *TODO query global graph
     *TODO determine local graph based on global graph
     */
    //query the local level roadmap
    if (p_mFullGraph->findPath(xStart, yStart, thStart, xTarget, yTarget, thTarget))
    {
      std::vector<Node*> path = p_mFullGraph->getPath();
      //p_mFullGraph->print(true); //print a .dotfile with the grap and highlights the waypoint nodes
      outputWaypoints(path);
      return true;
    }
    else
    {
      ROS_ERROR("no path can be found");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Query could not be commenced because environment has not been initialized. Are all nodes active?");
  }
  return false;
}
//TODO function to shut down the program clean
void GlobalPlanner::Stop()
{
  ROS_INFO("stop");
  //TODO stop function
  ros::shutdown();
}

//TODO action when error
void GlobalPlanner::Error()
{
  ROS_ERROR("some error occured");
}

/*
 * main function
 * waiting for msg and srv callbacks
 */
void GlobalPlanner::loop(void)
{
  ros::spin();
  return;
}

int main(int argc, char ** argv)
{
  //init ROS environment
  std::string node_name = "global_planner";
  ros::init(argc, argv, node_name);

  ROS_INFO("started global_planner");
  GlobalPlanner x(node_name, NO_LOOP);
  x.loop();
  ROS_INFO("global_planner exit");
  return 0;
}

