#include <gtest/gtest.h>
#include <obstacle_detector_lib.h>
#include "test_obstacle_detector.h"

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

pcl::PCLPointCloud2 read(const std::string& input)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PCLPointCloud2 outputCloud;
	pcl::PCDReader reader;	
	
	reader.read(input, cloud);
	
	pcl::toPCLPointCloud2(cloud, outputCloud);
	
	return outputCloud;	
}


bool pointCloudsEqual(const pcl::PCLPointCloud2 pc_A ,const pcl::PCLPointCloud2 pc_B)
{
	//TODO further check for similarities
	return((pc_A.width * pc_A.height) == (pc_B.width * pc_B.height));
}


bool checkVoxel(const pcl::PCLPointCloud2 pc_A ,const pcl::PCLPointCloud2 pc_B, const float threshold)
{
	//todo
}



TEST(ObstacleDetector_LibTestSuite, testDummy)
{    
    //pass
    //EXPECT_TRUE();
}

TEST(ObstacleDetector_LibTestSuite, voxelFilter)
{    
	pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/testcases/testcase2D.pcd");     
	
    //pass
    //EXPECT_TRUE();
}
TEST(ObstacleDetector_LibTestSuite, extract_clusters)
{    
	pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/EXPORT8.pcd");  
	Pcl2Vector clusters = extractClusters(input);  
    EXPECT_EQ(27, clusters.size() );
}


TEST(ObstacleDetector_LibTestSuite, environmentcloud_reconstruct)
{    
	pcl::PCLPointCloud2 input  = read(ros::package::getPath("skynav_tests")+"/include/EXPORT1.pcd");
	pcl::PCLPointCloud2 input2 = read(ros::package::getPath("skynav_tests")+"/include/EXPORT2.pcd"); 
	pcl::PCLPointCloud2 input3 = read(ros::package::getPath("skynav_tests")+"/include/EXPORT3.pcd");  
	pcl::PCLPointCloud2 input4 = read(ros::package::getPath("skynav_tests")+"/include/EXPORT4.pcd");  
	pcl::PCLPointCloud2 input5 = read(ros::package::getPath("skynav_tests")+"/include/EXPORT5.pcd");  

    //pass
    //EXPECT_TRUE();
}


TEST(ObstacleDetector_LibTestSuite, projectXY)
{    
    //pass
    //EXPECT_TRUE();
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
