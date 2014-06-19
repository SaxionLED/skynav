#ifndef LOCALNAV_TYPES_H
#define LOCALNAV_TYPES_H

#include <Eigen/StdVector>
#include <boost/optional.hpp>

typedef std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > PclXYZVector;
typedef std::vector<pcl::PCLPointCloud2, Eigen::aligned_allocator<pcl::PCLPointCloud2> > Pcl2Vector;

typedef boost::optional<pcl::PointXYZ> pclOptionPoint;

#endif
