/*
 * map_data.cpp
 *
 *  Created on: Nov 22, 2013
 *      Author: jurriaan
 */
#include <ros/ros.h>
#include "graph.h"
#define std_maxconnect 10	//
#define std_maxdist 50
#define std_maxnodes 200

MapData::MapData(unsigned int xDimension, unsigned int yDimension, float resolution)
{
  this->mXdim = xDimension;
  this->mYdim = yDimension;
  this->mResolution = resolution;

  this->mMaxNConnect = std_maxconnect;

  //todo, create formula for the optimal roadmap density.

  this->mMax_NDist = (xDimension + yDimension) / 4;
  this->mMax_RNodes = xDimension + yDimension;

  this->init();
}
MapData::MapData(unsigned int xDimension, unsigned int yDimension, float resolution, unsigned int maxNodes,
                 unsigned int maxConnect, float maxDist)
{
  this->mXdim = xDimension;
  this->mYdim = yDimension;
  this->mResolution = resolution;
  this->mMaxNConnect = maxConnect;
  this->mMax_NDist = maxDist;
  this->mMax_RNodes = maxNodes;

  this->init();
}

MapData::~MapData()
{
  v2dMap.clear();
  for (std::vector<Node*>::iterator it = v_mFixedWPs.begin(); it != v_mFixedWPs.end(); it++)
  {
    delete (*it);
  }
  v_mFixedWPs.clear();
//TODO delete  fixed waypoints vector<Node*>
}

void MapData::setMaxRandNodes(unsigned int maxRNodes)
{
  this->mMax_RNodes = maxRNodes;
}

void MapData::setMaxNDist(float maxDist)
{
  this->mMax_NDist = maxDist;
}

void MapData::setMaxNConnect(unsigned int maxNconn)
{
  this->mMaxNConnect = maxNconn;
}

/*
 * create the occupance grid of the map and fill with Cfree
 */
void MapData::init()
{
//resize the 2d vector to match the size of the map
  v2dMap.resize(mYdim + 1, std::vector<cSpace>(mXdim + 1));

  for (unsigned int y = 0; y < mYdim + 1; y++)
  {
    for (unsigned int x = 0; x < mXdim + 1; x++)
    {
      v2dMap[y][x] = spaceType::Cfree;
    }
  }

}
//return distance between coordinates in map.
float MapData::getPDistance(Point* p_A, Point* p_B)
{
  float d;
  int xd, yd;

  xd = p_B->mXpos - p_A->mXpos;
  yd = p_B->mYpos - p_A->mYpos;
  //Euclidian Distance
  d = (sqrt((xd * xd) + (yd * yd)));
  return d;
}

//Check if node coordinates collide with environment or already existing node
bool MapData::checkCCollision(unsigned int x, unsigned int y)
{
  if ((v2dMap[y][x]) == spaceType::Object || (v2dMap[y][x]) == spaceType::Node)
  {
    //collision detected
    return true;
  }
  else
  {
    return false;
  }
}
//check if node collides with environment or already existing node
bool MapData::checkPCollision(Node* p_node)
{
  unsigned int p_x = p_node->getXpos();
  unsigned int p_y = p_node->getYpos();
  if ((v2dMap[p_y][p_x]) == spaceType::Object || (v2dMap[p_y][p_x]) == spaceType::Node)
  {
    //collision detected
    return true;
  }
  return false;

}

//check for all the points in the vector<point*> in line if there is collision with the known map.
bool MapData::checkLineCollission(Line* p_line)
{
  for (std::vector<Point*>::iterator it = p_line->mCoordinates.begin(); it != p_line->mCoordinates.end(); it++)
  {
    if ((v2dMap[(*it)->mYpos][(*it)->mXpos]) == spaceType::Object)
    {
      // line collides with known object on the map
      return true;
    }
  }
  return false;
}

//create a line (vector<Point*>) for the edge between coordinate a and coordinate b
Line* MapData::Bresenham(Point* A, Point* B)
{
  int x1 = A->mXpos;
  int y1 = A->mYpos;

  int const x2 = B->mXpos;
  int const y2 = B->mYpos;
  std::vector<Point*> points;
  int delta_x(x2 - x1);
  // if x1 == x2, then it does not matter what we set here
  signed char const ix((delta_x > 0) - (delta_x < 0));
  delta_x = std::abs(delta_x) << 1;

  int delta_y(y2 - y1);
  // if y1 == y2, then it does not matter what we set here
  signed char const iy((delta_y > 0) - (delta_y < 0));
  delta_y = std::abs(delta_y) << 1;

  //push first point into list
  Point* p_point = new Point(x1, y1);
  points.push_back(p_point);

  if (delta_x >= delta_y)
  {
    // error may go below zero
    int error(delta_y - (delta_x >> 1));

    while (x1 != x2)
    {
      if ((error >= 0) && (error || (ix > 0)))
      {
        error -= delta_x;
        y1 += iy;
      }
      // else do nothing

      error += delta_y;
      x1 += ix;
      Point* p_point = new Point(x1, y1);
      points.push_back(p_point);
    }
  }
  else
  {
    // error may go below zero
    int error(delta_x - (delta_y >> 1));

    while (y1 != y2)
    {
      if ((error >= 0) && (error || (iy > 0)))
      {
        error -= delta_y;
        x1 += ix;
      }
      // else do nothing

      error += delta_x;
      y1 += iy;
      Point* p_point = new Point(x1, y1);
      points.push_back(p_point);
    }
  }
  Line* p_line = new Line(points);
  return p_line;
}

unsigned int MapData::getMaxRandNodes() const
{
  return mMax_RNodes;
}
float MapData::getMaxNDist() const
{
  return mMax_NDist;
}
unsigned int MapData::getMaxNConnect() const
{
  return mMaxNConnect;
}
unsigned int MapData::getXdimension() const
{
  return mXdim;
}
unsigned int MapData::getYdimension() const
{
  return mYdim;
}

//mark a cell on the map as free or occupied
bool MapData::markNode(Node* p_point, cSpace e_cSpace)
{
  v2dMap[p_point->getYpos()][p_point->getXpos()] = e_cSpace;
  return true;
}

//parse the data[] to a 2d grid represenation of the map
void MapData::parseOccupancyList(std::vector<int> &occupancyList)
{
  std::cout << "parsing data\n";
  int count = 0;
  for (int y = 0; y < this->mYdim; y++)
  {
    for (int x = 0; x < this->mXdim; x++)
    {
      if (occupancyList[count] == 100)
      {
        this->v2dMap[y][x] = spaceType::Object;
      }
      else if (occupancyList[count] == 1)
      {
        this->v2dMap[y][x] = spaceType::Cfree;
      }
      if (count != occupancyList.size())
      {
        count++;
      }
      else
      {
        return;
      }
    }
  }

}
//check if coordinates are within the bounds of the stated environment
bool MapData::checkCoordinates(unsigned int xPos, unsigned int yPos)
{
  if ((xPos < 0 || yPos < 0) || (xPos > this->mXdim || yPos > this->mYdim))
  {
    return false;
  }
  return true;
}
std::vector<std::vector<cSpace> > MapData::getMapData() const
{
  return v2dMap;
}

std::vector<Node*> MapData::getFixedWPs() const
{
  return v_mFixedWPs;
}

bool MapData::addFixedWPs(std::vector<Node*> p_list)
{
  ROS_INFO("add new fixed waypoints");
  for (std::vector<Node*>::iterator it = p_list.begin(); it != p_list.end(); it++)
  {
    std::cout << (*it)->getId() << std::endl;
  }
  return true;
}

//receive a new updated list of fixed waypoints.
bool MapData::updateFixedWPs(std::vector<Node*> p_list)
{
  ROS_INFO("Update fixed waypoints");

  for (std::vector<Node*>::iterator it = v_mFixedWPs.begin(); it != v_mFixedWPs.end(); it++)
  {
    delete (*it);
  }
  v_mFixedWPs.clear();

  for (std::vector<Node*>::iterator it = p_list.begin(); it != p_list.end(); it++)
  {
    v_mFixedWPs.push_back((*it));
  }

  return true;
}

//TODO update the map with new information
void MapData::update()
{
  //TODO
  //change dimensions,
  //change occupancy grid
}

