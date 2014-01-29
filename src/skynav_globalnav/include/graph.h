/*
 * graph.h
 *
 *  Created on: Nov 14, 2013
 *      Author: jurriaan
 */

#ifndef GRAPH_H_
#define GRAPH_H_
#include <vector>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <algorithm>


class Node;
class Edge;
class Graph;
class PathFinder;
class MapData;

namespace nodeTypes
{
enum nodeType
{
  Fixed_room, Fixed_door, Random_node, Not_Defined, Fixed_General
};
}
typedef nodeTypes::nodeType nodeType;

namespace spaceType
{
enum cSpace
{
  Cfree, Object, Node
};
}
typedef spaceType::cSpace cSpace;

struct Point
{
  unsigned int mXpos;
  unsigned int mYpos;
  Point(unsigned int x, unsigned int y)
  {
    this->mXpos = x;
    this->mYpos = y;
  }
};
struct Line
{
  std::vector<Point*> mCoordinates;
  Line(std::vector<Point*> coords)
  {
    this->mCoordinates = coords;
  }
  ~Line()
  {
    for (std::vector<Point*>::iterator it = mCoordinates.begin(); it != mCoordinates.end(); it++)
    {
      delete (*it);
    }
    mCoordinates.clear();
  }
};
struct NeighbourDist
{
  Node* node;
  float dist;
  NeighbourDist(Node* n, float d)
  {
    this->node = n;
    this->dist = d;
  }
};
bool distSort(NeighbourDist* p_A, NeighbourDist* p_B);

class Node
{
public:
  Node(unsigned int x, unsigned int y, unsigned int id);
  //Node(Graph*, unsigned int x, unsigned int y, unsigned int id);
  Node(Graph*, unsigned int x, unsigned int y, unsigned int id, nodeType type);
  virtual ~Node();
  float estimateDist(unsigned int, unsigned int);
  bool compare(Node* p_node);
  void addConnection(Node* p_adjacent);

  std::vector<Node*> getAdjacencyList();
  void setAdjacencyList(std::vector<Node*> adjacencyList);
  float getF();
  void setF(float f);
  float getG();
  void setG(float g);
  float getH(Node* p_target);
  unsigned int getId();
  nodeType getType();
  void setType(nodeType e_type);
  unsigned int getXpos();
  void setXpos(unsigned int xpos);
  unsigned int getYpos();
  void setYpos(unsigned int ypos);
  Graph* getMGraph();
  void setMGraph(Graph* p_graph);
  Node* getMParent();
  void setMParent(Node* p_parent);
  void setTheta(float theta);
  float getTheta();

private:
  Graph* p_mGraph;
  Node* p_mParent;
  std::vector<Node*> mAdjacencyList;
  unsigned int mID;
  unsigned int mXpos;
  unsigned int mYpos;
  float mTheta;
  float mG;
  float mF;
  float mH;
  nodeType mType;
};

class Edge
{
public:
  Edge(Node*, Node*, float);
  Edge(Node*, Node*);
  virtual ~Edge();
  float getLength();
  float calcLenght();
  bool compare(Edge* p_edge);

  Node* getA();
  Node* getB();

private:
  Node* p_mA;
  Node* p_mB;
  float mLenght;
};

class Graph
{
public:
  Graph(MapData* p_mapData);
  virtual ~Graph();

  bool addNode(Node* p_node);
  bool nodeExist(Node* p_node);
  Node* returnNodeExist(Node* p_node);
  bool addEdge(Node* p_A, Node* p_B);
  bool edgeExist(Edge* p_edge);

  Edge* getEdgeBetween(Node* p_A, Node* p_B);
  std::vector<Node*> getPath()const;
  bool findPath(unsigned int xStart, unsigned int yStart, float thStart, unsigned int xTarget, unsigned int yTarget, float thTarget);
  std::vector<Node*> getAllNodes();
  std::vector<Edge*> getAllEdges();

  bool tryCreateEdge(Node* A, Node* B);
  void createRandomRoadmap();
  Node* tryAddToRoadmap(unsigned int xPos, unsigned int yPos,float theta, nodeType type);
  bool updateFixedWaypoints();
  bool exportGraph(std::string filePath);
  bool importGraph(std::string filePath);
  void print(bool path);
private:
  std::vector<Node*> v_mNodes; //al nodes that make up the roadmap
  std::vector<Edge*> v_mEdges; //all edges between the nodes on the roadmap
  std::vector<Node*> v_mPath; //the nodes that make up te path from start to end
  PathFinder* p_mAlgorithm; //the algorithm to find the path on the roadmap
  MapData* p_mMapData; //all usefull data known about the map
};

class MapData
{
public:
  MapData(unsigned int xDimension, unsigned int yDimension, float resolution);
  MapData(unsigned int xDimension, unsigned int yDimension, float resolution,unsigned int maNodes, unsigned int maxConnect, float maxDist);
  virtual ~MapData();
  float getPDistance(Point* p_A, Point* p_B);
  bool checkCCollision(unsigned int x, unsigned int y);
  bool checkPCollision(Node* p_node);
  bool checkLineCollission(Line* p_line);
  Line* Bresenham(Point* A, Point* B);
  unsigned int getMaxRandNodes() const;
  void setMaxRandNodes(unsigned int maxRNodes);
  float getMaxNDist() const;
  void setMaxNDist(float maxDist);
  unsigned int getMaxNConnect() const;
  void setMaxNConnect(unsigned int maxNconn);
  unsigned int getXdimension() const;
  unsigned int getYdimension() const;
  bool markNode(Node* point, cSpace e_cSpace);
  void parseOccupancyList(std::vector<int> &occupancyList);
  bool checkCoordinates(unsigned int xPos, unsigned int yPos);
  std::vector<std::vector<cSpace> > getMapData() const;
  std::vector<Node*> getFixedWPs() const;
  bool addFixedWPs(std::vector<Node*>);
  bool updateFixedWPs(std::vector<Node*>);
  void init();
  void update();

  bool readFromFile();//TODO TEMP FUNCTION


private:
  unsigned int mMax_RNodes; //maximum number of random nodes placed for creating the roadmap
  float mMax_NDist; //maximum distance between placed nodes to create an edge between them for creating the roadmap
  unsigned int mMaxNConnect; //maximum number of edges a new placed node can form to neighbours.
  std::vector<std::vector<cSpace> > v2dMap;
  std::vector<Node*> v_mFixedWPs;
  unsigned int mXdim;
  unsigned int mYdim;
  float mResolution;
};
#endif /* GRAPH_H_ */

