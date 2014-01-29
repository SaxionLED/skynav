/*
 * environment.cpp
 *
 * november 2013
 *
 * Jurriaan Voskes
 */
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>

#include <nav_msgs/OccupancyGrid.h>

//custom msgs
#include <skynav_msgs/user_init.h>
#include <skynav_msgs/environment_info.h>
#include <skynav_msgs/environment_srv.h>
#include <skynav_msgs/mapreader_srv.h>
#include <skynav_msgs/edit_fixedWPs_srv.h>

const int LOOP_RATE = 1;
const int NO_LOOP = 0;

struct Map_Info
{
  float mapHight;
  float mapWidth;
  float resolution;
  std::vector<int8_t> mOccupancydata;

  Map_Info()
  {
    this->mapHight = 0;
    this->mapWidth = 0;
    this->resolution = 1;
  }
  ~Map_Info()
  {
    mOccupancydata.clear();
  }
};

struct Node
{
  unsigned int xPos;
  unsigned int yPos;
  float theta;
  std::string type;
  Node(unsigned int x, unsigned int y, std::string type)
  {
    this->xPos = x;
    this->yPos = y;
    this->type = type;
  }
};

class Environment
{
private:
  std::string node_name_;
  ros::NodeHandle* node_;
  ros::NodeHandle* node_slam_;
  int loop_rate_;
  Map_Info* p_mMap_Info;
  std::string s_mMap_Filepath;

  //publishers
  //ros::Publisher environment_pub_;
  ros::Publisher pubOccupancyGrid;
  ros::ServiceServer environment_srv_;
  ros::ServiceServer receive_newMap_srv_;
  ros::ServiceServer receive_fixedWPs_srv_;
  ros::Publisher re_init_pub_;

  // subscribers
  ros::ServiceClient getMapRead_;
  ros::ServiceClient send_fixedWPS_srv_;

  std::vector<Node*> v_pFixedWPs;

public:
  Environment(std::string node_name, int loop_rate);
  virtual ~Environment()
  {
    delete node_slam_;
    delete node_;

    delete p_mMap_Info;
    for (std::vector<Node*>::iterator it = v_pFixedWPs.begin(); it != v_pFixedWPs.end(); it++)
    {
      delete (*it);
    }
    v_pFixedWPs.clear();
  }
  ;

  void loop();
  bool respond_environment(skynav_msgs::environment_srv::Request &req,
                           skynav_msgs::environment_srv::Response &res);
  bool respond_newMap(skynav_msgs::mapreader_srv::Request &req, skynav_msgs::mapreader_srv::Response &res);
  bool respond_fixedWPs(skynav_msgs::edit_fixedWPs_srv::Request &req,
                        skynav_msgs::edit_fixedWPs_srv::Response &res);
  bool getMapRead(std::string filePath);
};

Environment::Environment(std::string node_name, int loop_rate) :
    node_name_(node_name), loop_rate_(loop_rate)
{
  //create nodehandles
  node_ = new ros::NodeHandle("/globalnav");
  node_slam_ = new ros::NodeHandle("/SLAM");

  //environment_pub_ = node_->advertise<skynav_msgs::environment_info>("environment_info", 10);
  re_init_pub_ = node_->advertise<skynav_msgs::user_init>("user_init", 10);
  
  pubOccupancyGrid = node_->advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);

  environment_srv_ = node_->advertiseService("environment_req", &Environment::respond_environment, this);
  receive_newMap_srv_ = node_->advertiseService("update_map_req", &Environment::respond_newMap, this);
  receive_fixedWPs_srv_ = node_->advertiseService("edit_fixedWPs", &Environment::respond_fixedWPs, this);

  send_fixedWPS_srv_ = node_->serviceClient<skynav_msgs::edit_fixedWPs_srv>("update_fixed_waypoints");

  getMapRead_ = node_slam_->serviceClient<skynav_msgs::mapreader_srv>("map_read_req");

  this->p_mMap_Info = new Map_Info();
  s_mMap_Filepath = "";
}

/*
 * respond on environment info request
 * from init() in global_planner
 */
bool Environment::respond_environment(skynav_msgs::environment_srv::Request &req,
                                      skynav_msgs::environment_srv::Response &res)
{
  if (req.request)
  {
    skynav_msgs::environment_info msg;
    ROS_INFO("environment received request");
    if (!s_mMap_Filepath.empty()) //check if the filepath has been set, if not.. return false because no map has been loaded yet
    {
      ROS_INFO(" path: %s", s_mMap_Filepath.c_str());
      msg.state = 1;
      msg.map.info.width = this->p_mMap_Info->mapWidth;
      msg.map.info.height = this->p_mMap_Info->mapHight;
      msg.map.info.resolution = this->p_mMap_Info->resolution;
      msg.map.data.resize(this->p_mMap_Info->mapWidth * this->p_mMap_Info->mapHight);
      std::vector<int8_t> data = this->p_mMap_Info->mOccupancydata;
      for (uint i = 0; i < data.size(); i++)
      {
        msg.map.data[i] = data[i];
      }
      for (std::vector<Node*>::iterator it = this->v_pFixedWPs.begin(); it != this->v_pFixedWPs.end(); it++)
      {
        geometry_msgs::Pose2D tmp_pose;
        tmp_pose.x = (*it)->xPos;
        tmp_pose.y = (*it)->yPos;
        tmp_pose.theta = (*it)->theta;
        msg.fixed_waypoints.push_back(tmp_pose);
      }
      res.environment = msg;
      return true;
    }
    else
    {
      ROS_ERROR("no map initialized yet");
      return false;
    }
  }
  ROS_ERROR("no viable request received");
  return false;

}

/*
 * respond to an update request of the map and environment
 * from gui
 * 
 * this initiates a getMapread and awaits response
 * 
 */
bool Environment::respond_newMap(skynav_msgs::mapreader_srv::Request &req,
                                 skynav_msgs::mapreader_srv::Response &res)
{
  if (req.request)
  {
    ROS_INFO("new map update requested");
    if (getMapRead(req.file_path)) //call the mapreader for parsing the given map
    {
      this->s_mMap_Filepath = req.file_path;

      for (std::vector<Node*>::iterator it = v_pFixedWPs.begin(); it != v_pFixedWPs.end(); it++)
      {
        delete (*it);
      }
      this->v_pFixedWPs.clear();

      res.response = 1;
      res.map.info.height = this->p_mMap_Info->mapHight;
      res.map.info.width = this->p_mMap_Info->mapWidth;
      //todo
      res.map.info.resolution = this->p_mMap_Info->resolution;

      res.map.data.resize(this->p_mMap_Info->mapWidth * this->p_mMap_Info->mapHight);
      std::vector<int8_t> data = this->p_mMap_Info->mOccupancydata;
      for (uint i = 0; i < data.size(); i++)
      {
        res.map.data[i] = data[i];
      }
      data.clear(); //delete the temporary vector data<int>

      skynav_msgs::user_init msg;
      msg.state = 1;
      re_init_pub_.publish(msg); //call init in global planner to initialize the environment
      return true; //respond to the gui with environment information
    }
  }
  ROS_ERROR("update map failed");
  return false;
}

/*
 * request a new map from SLAM/mapreader
 * gets initialized from respond_newMap()
 * returns parsed mapdata from mapreader.
 */
bool Environment::getMapRead(std::string filePath)
{
  if (!filePath.empty())
  {
    skynav_msgs::mapreader_srv srv;
    srv.request.request = 1;
    srv.request.file_path = filePath;
    if (getMapRead_.call(srv))
    {
      //process map data
      this->p_mMap_Info->mapHight = srv.response.map.info.height;
      this->p_mMap_Info->mapWidth = srv.response.map.info.width;
      this->p_mMap_Info->resolution = srv.response.map.info.resolution;

      this->p_mMap_Info->mOccupancydata.clear();
      for (uint i = 0; i < srv.response.map.data.size(); i++)
      {
        this->p_mMap_Info->mOccupancydata.push_back(srv.response.map.data[i]);
      }

      ROS_INFO("map H %f", this->p_mMap_Info->mapHight);
      ROS_INFO("map W %f", this->p_mMap_Info->mapWidth);
      ROS_INFO("map R %f", this->p_mMap_Info->resolution);
	  
	  // EXPERIMENTAL occupancy grid for rviz
	  nav_msgs::OccupancyGrid og;
	  og.header.frame_id = "/map";
	  og.header.stamp = ros::Time::now();
	  
	  og.data = p_mMap_Info->mOccupancydata;
	  
	  og.info.resolution = p_mMap_Info->resolution / 100;	//TODO resolution fix
	  og.info.width = p_mMap_Info->mapWidth;
	  og.info.height = p_mMap_Info->mapHight;		//TODO FIX TYPO
	  
	  pubOccupancyGrid.publish(og);
	  
      return true;
    }
    ROS_ERROR("no viable response from SLAM/mapreader.. check if it is running");
  }
  else
  {
    ROS_ERROR("empty filepath, no map to read");
  }
  return false;
}

/*
 * function to respond on fixed waypoins update
 */
bool Environment::respond_fixedWPs(skynav_msgs::edit_fixedWPs_srv::Request &req,
                                   skynav_msgs::edit_fixedWPs_srv::Response &res)
{
  if (req.request == 1)
  {
    ROS_INFO("received new set of fixed waypoints, clear old one");

    for (std::vector<Node*>::iterator it = v_pFixedWPs.begin(); it != v_pFixedWPs.end(); it++)
    {
      delete (*it);
    }
    this->v_pFixedWPs.clear();

    std::vector<geometry_msgs::Pose2D> tmp_data = req.waypoints;
    for (std::vector<geometry_msgs::Pose2D>::iterator it = tmp_data.begin(); it != tmp_data.end(); it++)
    {
      Node* pNode = new Node((*it).x, (*it).y, "FIXED");
      this->v_pFixedWPs.push_back(pNode);
    }
    ROS_INFO("updated fixed waypoints in environment information");
    /*
     * send update to global planner
     */
    skynav_msgs::edit_fixedWPs_srv srv;
    srv.request.request = 1;
    srv.request.waypoints = req.waypoints;
    if(send_fixedWPS_srv_.call(srv)){
      ROS_INFO("New fxwaypoints to Global_planner");
      return true;
    }
    else{
      ROS_ERROR("NO Response from Global_planner");
      return true;;
    }
  }
  else
  {
    ROS_ERROR("NO viable new set of waypoints received");
    return false;
  }
}

void Environment::loop()
{
  ROS_INFO("started environment");
  ros::spin();
}

int main(int argc, char ** argv)
{
  std::string node_name = "environment";
  ros::init(argc, argv, node_name);

  Environment x(node_name, LOOP_RATE);
  x.loop();
  ROS_INFO("Environment exit");
  return 0;
}

