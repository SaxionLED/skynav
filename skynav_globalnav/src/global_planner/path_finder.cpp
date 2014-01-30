/*
 * path_finder.cpp
 *
 *  Created on: Nov 20, 2013
 *      Author: jurriaan
 */

#include "path_finder.h"

PathFinder::PathFinder(Graph* p_graph)
{
  p_mGraph = p_graph;
}

PathFinder::~PathFinder()
{
  mPath.clear();
}
/*
 * determine if a certain node exist in a given list of nodes
 */

bool PathFinder::isInList(Node* p_node, std::vector<Node*> &v_pList)
{
  for (std::vector<Node*>::iterator it = v_pList.begin(); it != v_pList.end(); it++)
  {
    if ((*it)->compare(p_node))
    {
      return true;
    }
  }
  return false;
}
/*
 * delete a certain node from a list of nodes.
 * DOES NOT DELETE THE OBJECT, ONLY THE REFERENCE IN THIS LIST
 */
bool PathFinder::delFromList(Node* p_node, std::vector<Node*> &v_pList)
{
  for (std::vector<Node*>::iterator it = v_pList.begin(); it != v_pList.end(); it++)
  {
    if ((*it)->compare(p_node))
    {
      v_pList.erase(it);
      return true;
    }
  }
  return false;
}
/*
 * query a certain list and get the node with the lowest f score to target
 */
Node* PathFinder::getLowestF(std::vector<Node*> v_pList)
{
  Node* p_Temp = v_pList.front();
  for (std::vector<Node*>::iterator it = v_pList.begin(); it != v_pList.end(); it++)
  {
    if (p_Temp->getF() > (*it)->getF())
    {
      p_Temp = (*it);
    }
  }
  return p_Temp;

}
/*
 * after a path has been found to the target,
 * recursively reconstruct the path from target to begin to determine the waypoints
 */
std::vector<Node*> PathFinder::reconstructPath(std::vector<Node*> &v_pPath, Node* p_wp)
{
  v_pPath.push_back(p_wp);
  if (!p_wp->compare(p_mStart))
  {
    v_pPath = reconstructPath(v_pPath, p_wp->getMParent());
  }
  return v_pPath;
}

std::vector<Node*> PathFinder::getPath()
{
  return mPath;
}

/*
 * query the graph with start and target node to find a path from start to end.
 */
bool PathFinder::findPath(Node* p_start, Node* p_target)
{
  std::vector<Node*> v_pOpen;				//the open list of not yet tried nodes
  std::vector<Node*> v_pClosed;				//the closed list with discarded tried nodes
  std::vector<Node*> v_pPathThree;			//the nodes of the path that has been tried	
  std::vector<Node*> v_pWayPoints;			//list for the reconstruction of the path
  p_mStart = p_start;						//start node
  p_mTarget = p_target;						//target node
  
  
 mPath.clear();
  p_start->setG(0);
  p_start->getH(p_target);
  v_pOpen.push_back(p_start);

  while (!v_pOpen.empty())
  {
    Node* p_curNode = getLowestF(v_pOpen);
    if (p_curNode->compare(p_target))
    {
      v_pPathThree.push_back(p_curNode);
      v_pWayPoints = reconstructPath(v_pWayPoints, p_curNode);

      mPath = v_pWayPoints;
      v_pWayPoints.clear();
      v_pOpen.clear();
      v_pClosed.clear();
      v_pPathThree.clear();
      return true;
    }

    v_pClosed.push_back(p_curNode);
    delFromList(p_curNode, v_pOpen);

    std::vector<Node*> v_pAdjacencyList = p_curNode->getAdjacencyList();
    for (std::vector<Node*>::iterator it = v_pAdjacencyList.begin(); it != v_pAdjacencyList.end(); it++)
    {
      Node* p_childNode = (*it);

      float fp_globalG = p_curNode->getG() + p_mGraph->getEdgeBetween(p_curNode, p_childNode)->getLength();
      float fp_globalF = fp_globalG + p_childNode->getH(p_target);

      if (isInList(p_childNode, v_pClosed) && fp_globalF >= p_childNode->getF())
      {
        //nothing   --> continue
      }
      else if (!isInList(p_childNode, v_pOpen) || fp_globalF < p_childNode->getF())
      {
        v_pPathThree.push_back(p_curNode);
        p_childNode->setMParent(p_curNode);
        p_childNode->setG(fp_globalG);
        if (!isInList(p_childNode, v_pOpen))
        {
          v_pOpen.push_back(p_childNode);
        }
      }
    }
  }
  ROS_ERROR("error! no path could be found\n");
  mPath.push_back(p_mStart);
  return false;
}
