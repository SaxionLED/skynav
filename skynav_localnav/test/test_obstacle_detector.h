#ifndef TEST_OBSTACLE_DETECTOR_H
#define TEST_OBSTACLE_DETECTOR_H

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/exceptions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/package.h>
#include "localnav_types.h"

pcl::PCLPointCloud2 read(const std::string& input);
bool pointCloudsEqual(const pcl::PCLPointCloud2 pc_A ,const pcl::PCLPointCloud2 pc_B);
bool checkVoxel(const pcl::PCLPointCloud2 pc_A ,const pcl::PCLPointCloud2 pc_B, const float threshold);



#endif

