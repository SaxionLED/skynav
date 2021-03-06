#ifndef TEST_LOCALPLANNER_H
#define TEST_LOCALPLANNER_H
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/exceptions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <ros/package.h>
#include "localnav_types.h"

pcl::PCLPointCloud2 read(const std::string& input);

bool pointCloudsEqual(const pcl::PCLPointCloud2 pc_A ,const pcl::PCLPointCloud2 pc_B);

bool wpcheck_validate(const pclOptionPoint& value, const pcl::PCLPointCloud2& ref);

bool rec_bug_validate(const pclOptionPoint& value, const pcl::PCLPointCloud2& ref);

#endif
