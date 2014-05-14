/*
 * node.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: jurriaan
 */

#include "graph.h"
Node::Node(unsigned int x, unsigned int y, unsigned int id)
{
  this->mXpos = x;
  this->mYpos = y;
  this->mID = id;
  this->p_mParent = NULL;
  this->p_mGraph = NULL;
  this->mF = 0;
  this->mG = 0;
  this->mH = 0;
  this->mType = nodeTypes::Not_Defined;
  this->mTheta = 0;

}

Node::Node(Graph* graph, unsigned int x, unsigned int y, unsigned int id, nodeType type)
{
  this->p_mGraph = graph;
  this->mXpos = x;
  this->mYpos = y;
  this->mID = id;
  this->p_mParent = NULL;
  this->mF = 0;
  this->mG = 0;
  this->mH = 0;
  this->mType = type;
  this->mTheta = 0;
}

Node::~Node()
{
//
}

float Node::estimateDist(unsigned int xDest, unsigned int yDest)
{
  int xd, yd;
  float d;
  xd = xDest - this->getXpos();
  yd = yDest - this->getYpos();
  // Euclidian Distance
  d = (sqrt((xd * xd) + (yd * yd))); // Manhattan distance: d=abs(xd)+abs(yd) // Chebyshev distance: d=max(abs(xd), abs(yd));
  return d;
}

bool Node::compare(Node* p_node)
{
  if (p_node->getXpos() == this->getXpos() && p_node->getYpos() == this->getYpos())
  {
    return true;
  }
  return false;
}

void Node::addConnection(Node* p_adjacent)
{
  this->mAdjacencyList.push_back(p_adjacent);
}

//--getters & setters--//

std::vector<Node*> Node::getAdjacencyList()
{
  return mAdjacencyList;
}

void Node::setAdjacencyList(std::vector<Node*> adjacencyList)
{
  mAdjacencyList = adjacencyList;
}

float Node::getF()
{
  mF = mH + mG;
  return mF;
}

void Node::setF(float f)
{
  mF = f;
}

float Node::getG()
{
  return mG;
}

void Node::setG(float g)
{
  mG = g;
}

float Node::getH(Node* p_target)
{
  mH = estimateDist(p_target->getXpos(), p_target->getYpos());
  return mH;
}

unsigned int Node::getId()
{
  return mID;
}

nodeType Node::getType()
{
  return mType;
}

void Node::setType(nodeType e_type)
{
  mType = e_type;
}

unsigned int Node::getXpos()
{
  return mXpos;
}

void Node::setXpos(unsigned int xpos)
{
  mXpos = xpos;
}

unsigned int Node::getYpos()
{
  return mYpos;
}

void Node::setYpos(unsigned int ypos)
{
  mYpos = ypos;
}

Graph* Node::getMGraph()
{
  return p_mGraph;
}

void Node::setMGraph(Graph* p_graph)
{
  p_mGraph = p_graph;
}

Node* Node::getMParent()
{
  return p_mParent;
}

void Node::setMParent(Node* p_parent)
{
  p_mParent = p_parent;
}

void Node::setTheta(float theta)
{
  this->mTheta = theta;
}
float Node::getTheta()
{
  return this->mTheta;
}
