
#include <gtest/gtest.h>
#include <local_planner_lib.h>
#include "test_localplanner.h"

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
	return((pc_A.width * pc_A.height) == (pc_B.width * pc_B.height));
}

TEST(LocalPlannerLibTestSuite, testDummy)
{    
    //pass
}

TEST(LocalPlannerLibTestSuite, testTruncatValue)
{
    float input = 1.234567890987;
    EXPECT_FLOAT_EQ(1.234, truncateValue(input));
}


TEST(LocalPlannerLibTestSuite, testConvexHull)
{
    pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/testcases/testcase2D.pcd");    
	pcl::PCLPointCloud2 ref	= read(ros::package::getPath("skynav_tests")+"/include/testcases/testcase2D_solution.pcd");  
	
	EXPECT_TRUE(pointCloudsEqual(pclConvex_hull(input), ref))
	 << "input = " << ::testing::PrintToString(input.width * input.height) <<"ref = "<< ::testing::PrintToString(ref.width * ref.height); 
}


TEST(LocalPlannerLibTestSuite, testConcaveHull)
{
    pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/testcases/testcase2D.pcd");    
	pcl::PCLPointCloud2 ref	= read(ros::package::getPath("skynav_tests")+"/include/testcases/testcase2D_solution.pcd");  
	
	EXPECT_FALSE(pointCloudsEqual(pclConcave_hull(input), ref))
	 << "input = " << ::testing::PrintToString(input.width * input.height) <<"ref = "<< ::testing::PrintToString(ref.width * ref.height); 
}


TEST(LocalPlannerLibTestSuite, testRecursiveBug)
{
    // pass
}


TEST(LocalPlannerLibTestSuite, testWayPointCheck)
{
    // pass
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

