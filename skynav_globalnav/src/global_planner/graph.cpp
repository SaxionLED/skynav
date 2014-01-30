/*
 * graph.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: jurriaan
 */

#include "graph.h"
#include "path_finder.h"

Graph::Graph(MapData* p_mapData)
{
  p_mAlgorithm = new PathFinder(this);
  this->p_mMapData = p_mapData;

  //create a randomized roadmap based on the map and variables given in p_mapdata.
  createRandomRoadmap(); 
}

Graph::~Graph()
{
  for (std::vector<Edge*>::iterator it = v_mEdges.begin(); it != v_mEdges.end(); it++)
  {
    delete (*it);
  }
  for (std::vector<Node*>::iterator it = v_mNodes.begin(); it != v_mNodes.end(); it++)
  {
    delete (*it);
  }
  v_mEdges.clear();
  v_mNodes.clear();
  delete this->p_mAlgorithm;
}
/*
 * add a node to the list of nodes
 */
bool Graph::addNode(Node* p_node)
{
  v_mNodes.push_back(p_node);
  return true;
}
/*
 * check if a node exist in the list with nodes
 */
bool Graph::nodeExist(Node* p_node)
{
  for (std::vector<Node*>::iterator it = v_mNodes.begin(); it != v_mNodes.end(); it++)
  {
    if (p_node->compare(*it))
    {
      return true;
    }
  }
  return false;
}
/*
 * return the existing node if the node given as argument already exist
 */
Node* Graph::returnNodeExist(Node* p_node)
{
  for (std::vector<Node*>::iterator it = v_mNodes.begin(); it != v_mNodes.end(); it++)
  {
    if (p_node->compare(*it))
    {
      return (*it);
    }
  }
  return NULL;
}
/*
 * create a new edge between node a an b
 */
bool Graph::addEdge(Node* p_A, Node* p_B)
{
  if (!(p_A->compare(p_B))) //check if the nodes are the same. if same, return false
  {
    Edge* p_Temp = new Edge(p_A, p_B); //create new edge, and check for existence in the graph. if exists, delete, otherwise add.
    if (!edgeExist(p_Temp))
    {
      v_mEdges.push_back(p_Temp);
      p_A->addConnection(p_B);
      p_B->addConnection(p_A);
      return true;
    }
    delete p_Temp;
  }
  return false;
}
/*
 * check if an edge exist
 */
bool Graph::edgeExist(Edge* p_edge)
{
  for (std::vector<Edge*>::iterator it = v_mEdges.begin(); it != v_mEdges.end(); it++)
  {
    if (p_edge->compare(*it))
    {
      return true;
    }
  }
  return false;
}
/*
 * return the edge between two nodes
 */
Edge* Graph::getEdgeBetween(Node* p_A, Node* p_B)
{
  Edge* p_Temp = new Edge(p_A, p_B);
  for (std::vector<Edge*>::iterator it = v_mEdges.begin(); it != v_mEdges.end(); it++)
  {
    if (p_Temp->compare(*it))
    {
      delete p_Temp;
      return (*it);
    }
  }
  delete p_Temp;
  return NULL;
}

std::vector<Node*> Graph::getAllNodes()
{
  return v_mNodes;
}

std::vector<Edge*> Graph::getAllEdges()
{
  return v_mEdges;
}

/*
 * try to create an Edge between node A and B. if an edge can be created it will be made and added to the graph
 * if not, the function will return false
 */
bool Graph::tryCreateEdge(Node* p_A, Node* p_B)
{
  Point* pTemp_A = new Point(p_A->getXpos(), p_A->getYpos());
  Point* pTemp_B = new Point(p_B->getXpos(), p_B->getYpos());
  Line* p_edgeLine = p_mMapData->Bresenham(pTemp_A, pTemp_B);
  if (!p_mMapData->checkLineCollission(p_edgeLine))
  {
    addEdge(p_A, p_B);

    delete pTemp_A;
    delete pTemp_B;
    delete p_edgeLine;
    return true;
  }
  delete pTemp_A;
  delete pTemp_B;
  delete p_edgeLine;
  return false;
}
/*
 * create the roadmap, based on random sampling the environment
 */
void Graph::createRandomRoadmap()
{
  unsigned int ui_nodeID = v_mNodes.size() + 1; //TODO node id determination based on something
  unsigned int ui_nodesPlaced = 0; //start number of nodes created
  unsigned int ui_maxNodes = this->p_mMapData->getMaxRandNodes();
  unsigned int ui_maxConnect = this->p_mMapData->getMaxNConnect();
  float f_maxDist = this->p_mMapData->getMaxNDist();
  srand(time(NULL));

  while (ui_nodesPlaced <= ui_maxNodes)
  {
    //create random coordinates
    unsigned int tempX = rand() % this->p_mMapData->getXdimension();
    unsigned int tempY = rand() % this->p_mMapData->getYdimension();

    //check collision of potential new point
    if (!p_mMapData->checkCCollision(tempX,tempY))
    {
      Node* p_node = new Node(this, tempX, tempY, ui_nodeID, nodeTypes::Random_node);
      addNode(p_node); //put it in the graph::nodelist

      //p_mMapData->markNode(p_tempPoint, spaceType::Node);

      //create list of candidate neighbours
      std::vector<NeighbourDist*> candidateNodes;
      for (std::vector<Node*>::iterator it = v_mNodes.begin(); it != v_mNodes.end(); it++)
      {
        if (!p_node->compare(*it))
        {
          float f_tempDist = (*it)->estimateDist(tempX, tempY);
          if (f_tempDist <= f_maxDist)
          {
            NeighbourDist* p_neighbour = new NeighbourDist((*it), f_tempDist);
            candidateNodes.push_back(p_neighbour);
          }
        }
      }
      //sort neighbour nodes based on distance from current
      std::sort(candidateNodes.begin(), candidateNodes.end(), distSort);

      //connect neighbour nodes untill max_connections
      unsigned int count = 0;
      for (std::vector<NeighbourDist*>::iterator it = candidateNodes.begin(); it != candidateNodes.end(); it++)
      {
        if (count < ui_maxConnect)
        {
          if (tryCreateEdge(p_node, (*it)->node))
          {
            count += 1;
          }
        }
        else
          break;
      }

      ui_nodeID++;
      ui_nodesPlaced++;
      //delete the neighbour nodes list from memory after it is not needed anymore
      for (std::vector<NeighbourDist*>::iterator it = candidateNodes.begin(); it != candidateNodes.end(); it++)
      {
        delete (*it);
      }
    }
  }
}

/*
 * creates a new node on the roadmap and returns it, if it does not already exists.
 * if it already exists, it returns the original node
 */
Node* Graph::tryAddToRoadmap(unsigned int xPos, unsigned int yPos, float theta, nodeType type)
{
  unsigned int ui_nodeID = v_mNodes.size() + 1; //TODO node id determination
  unsigned int ui_maxConnect = this->p_mMapData->getMaxNConnect();
  float f_maxDist = this->p_mMapData->getMaxNDist();

  Node* p_tempNode = new Node(this, xPos, yPos, ui_nodeID, type);
  p_tempNode->setTheta(theta);
  Node* p_existingNode = returnNodeExist(p_tempNode);
  if (p_existingNode != NULL)
  {
    delete p_tempNode;
    p_existingNode->setTheta(theta);
    return p_existingNode;
  }
  else if (!p_mMapData->checkPCollision(p_tempNode)){
    addNode(p_tempNode);
    p_mMapData->markNode(p_tempNode, spaceType::Node);
    //create list of candidate neighbours
    std::vector<NeighbourDist*> candidateNodes;
    for (std::vector<Node*>::iterator it = v_mNodes.begin(); it != v_mNodes.end(); it++)
    {
      if (!p_tempNode->compare(*it))
      {
        float f_tempDist = (*it)->estimateDist(xPos, yPos);
        if (f_tempDist <= f_maxDist)
        {
          NeighbourDist* p_neighbour = new NeighbourDist((*it), f_tempDist);
          candidateNodes.push_back(p_neighbour);
        }
      }
    }
    //sort neighbour nodes based on distance from current
    std::sort(candidateNodes.begin(), candidateNodes.end(), distSort);

    //connect neighbour nodes untill max_connections
    unsigned int count = 0;
    for (std::vector<NeighbourDist*>::iterator it = candidateNodes.begin(); it != candidateNodes.end(); it++)
    {
      if (count < ui_maxConnect)
      {
        if (tryCreateEdge(p_tempNode, (*it)->node))
        {
          count += 1;
        }
      }
      else
        break;
    }
    //delete the neighbour nodes list from memory after it is not needed anymore
    for (std::vector<NeighbourDist*>::iterator it = candidateNodes.begin(); it != candidateNodes.end(); it++)
    {
      delete (*it);
    }
    return p_tempNode;
  }else{
	//node collides with environment
    delete p_tempNode;
    return NULL;
  }
}

//--Query the graph for a path with start and target coordinates
bool Graph::findPath(unsigned int xStart, unsigned int yStart,float thStart, unsigned int xTarget, unsigned int yTarget,float thTarget)
{
  //check if coordinates lie within map region
  if (p_mMapData->checkCoordinates(xStart, yStart) && p_mMapData->checkCoordinates(xTarget, yTarget))
  {
    Node* p_start = tryAddToRoadmap(xStart, yStart, thStart, nodeTypes::Fixed_General); //try to generate new Node(x,y) on graph, or get already existing node
    Node* p_target = tryAddToRoadmap(xTarget, yTarget,thTarget, nodeTypes::Fixed_General);// ""

    if(p_start && p_target){ //check if p_start and p_target != NULL, which means they collide with the environment

    if (p_mAlgorithm->findPath(p_start, p_target))//query the graph to find path from start to target
    {
      v_mPath = p_mAlgorithm->getPath();
      ROS_INFO("found Path");
      return true;
    }
    else
    {
      ROS_ERROR("no path found");
      return false;
    }
  }else{
    ROS_ERROR("start or target collide with environment");
    return false;
  }
  }
  ROS_ERROR("start or target dont lie on the map");
  return false;
}



bool Graph::updateFixedWaypoints(){
  /*
   *todo method to delete old fixed waypoints
   *todo memleak here!?
   */
  std::vector<Node*> p_list = p_mMapData->getFixedWPs();
for(std::vector<Node*>::iterator it = p_list.begin(); it!=p_list.end();it++){
  tryAddToRoadmap((*it)->getXpos(),(*it)->getYpos(),(*it)->getTheta(),nodeTypes::Fixed_General);
}
ROS_INFO("added new waypoints to graph");
return true;
}

//return the found path
std::vector<Node*> Graph::getPath() const
{
  if(v_mPath.empty()){
    ROS_ERROR("path is empty");
    //return NULL
  }
  return v_mPath;
}

//TODO export the current graph to a file
bool Graph::exportGraph(std::string filePath)
{
  ROS_INFO("EXPORT GRAPH");
  return true;
}

//TODO import a previously saved graph
bool Graph::importGraph(std::string filePath)
{
  ROS_INFO("IMPORT GRAPH");

  return true;
}

/*
 * print function to print the graph to a .dot file that can be processed by graphviz neato to produce a png image of t the graph
 * $ neato -Tpng -s4 graph.dot -O
 */
void Graph::print(bool path)
{
  std::ofstream dotfile;
  if (dotfile)
  {
    dotfile.open("/bin/graph.dot", std::ofstream::out);
    dotfile << "graph G { \n";
    for (std::vector<Node*>::iterator it = v_mNodes.begin(); it != v_mNodes.end(); it++)
    {
      dotfile << (*it)->getId() << "[";
      if (path)
      {
        for (std::vector<Node*>::iterator a = v_mPath.begin(); a != v_mPath.end(); a++)
        {
          if ((*it)->compare(*a))
          {
            dotfile << "color=\"red\",style=\"filled\",";
          }
        }
      }
      dotfile << "pos=\"" << (*it)->getXpos() << "," << (*it)->getYpos() << "!\"]\n";
    }

    for (std::vector<Edge*>::iterator it = v_mEdges.begin(); it != v_mEdges.end(); it++)
    {
      dotfile << (*it)->getA()->getId() << "--" << (*it)->getB()->getId();
      dotfile << ";\n";
    }

    dotfile << "}";
    dotfile.close();

  }
  else
  {
    std::cerr << "error, no graph printed possible \n";
  }
//  if (path)
//  {
//    for (std::vector<Node*>::iterator it = v_mPath.begin(); it != v_mPath.end(); it++)
//    {
//      std::cout << (*it)->getId() << "\n";
//    }
//  }
//  system("neato -Tpng -s4 graph.dot -O ");
}



/*-------------------------------------------------------------------
 * sorting function for sorting the neighbour nodes based on distance
 */
bool distSort(NeighbourDist* p_A, NeighbourDist* p_B)
{
  return (p_A->dist < p_B->dist);
}

