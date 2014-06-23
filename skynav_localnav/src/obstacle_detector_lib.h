
#ifndef OBSTACLE_DETECTOR_LIB_H
#define OBSTACLE_DETECTOR_LIB_H

#include <sensor_msgs/PointCloud2.h>
#include <skynav_msgs/PointCloudVector.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <tf/transform_listener.h>

#include "localnav_types.h"

pcl::PCLPointCloud2 voxelfilter(const pcl::PCLPointCloud2& inputCloud);

Pcl2Vector extractClusters(const pcl::PCLPointCloud2& inputCloud);

pcl::PCLPointCloud2 concatinateClouds(const pcl::PCLPointCloud2& inputCloudA, const pcl::PCLPointCloud2& inputCloudB);

pcl::PCLPointCloud2 projectOnXYPlane(const pcl::PCLPointCloud2& inputCloud);

pcl::PCLPointCloud2 constructEnvironmentCloud(const Pcl2Vector& cloudSet);


#endif
