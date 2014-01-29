/*
 * path_finder.h
 *
 *  Created on: Nov 20, 2013
 *      Author: jurriaan
 */

#ifndef PATH_FINDER_H_
#define PATH_FINDER_H_
#include <ros/ros.h> // You must include this to do things with ROS.
#include "graph.h"
#include <iostream>

class PathFinder
{
public:
  PathFinder(Graph* p_graph);
  virtual ~PathFinder();

  bool findPath(Node* p_start, Node* p_target);
  bool isInList(Node* p_node, std::vector<Node*> &v_pList);
  bool delFromList(Node* p_node, std::vector<Node*> &v_pList);
  Node* getLowestF(std::vector<Node*> v_pList);
  std::vector<Node*> reconstructPath(std::vector<Node*> &v_pPath, Node* p_wp);
  std::vector<Node*> getPath();

private:
  Graph* p_mGraph;
  std::vector<Node*> mPath;
  Node* p_mStart;
  Node* p_mTarget;
};

#endif /* PATH_FINDER_H_ */
