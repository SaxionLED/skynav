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
	//TODO further check for similarities
	return((pc_A.width * pc_A.height) == (pc_B.width * pc_B.height));
}

bool wpcheck_validate(const pclOptionPoint& value, const pcl::PCLPointCloud2& ref){
	if(value){
		ROS_INFO("colission detected at %f , %f ", (*value).x, (*value).y);
		return true;
	}
	ROS_INFO("no colission detected");
	return false;
}

bool rec_bug_validate(const pclOptionPoint& value, const pcl::PCLPointCloud2& ref){
	if(value){
		ROS_INFO("rec bug outcome: %f , %f ", (*value).x, (*value).y);
		return true;
	}
	ROS_INFO("error no return value");
	return false;
}


TEST(LocalPlannerLibTestSuite, testDummy)
{    
    //pass
    //EXPECT_TRUE();
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


TEST(LocalPlannerLibTestSuite, testWayPointCheck)
{
    pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/EXPORT10.pcd");    
    ASSERT_TRUE((input.width * input.height) != 0);
    
    pcl::PCLPointCloud2 input_outline = pclConcave_hull(input);
    ASSERT_TRUE((input_outline.width * input_outline.height) != 0);
    
    Pcl2Vector vOutlines;
    vOutlines.push_back(input_outline);
    ASSERT_TRUE(!vOutlines.empty());
    
    pcl::PointXYZ start1(0,0,0); 		
    pcl::PointXYZ target1(4,0,0);
        
    pcl::PointXYZ start2(0,0,0); 		
    pcl::PointXYZ target2(3,2,0);
    
	pcl::PointXYZ start3(0,0,0); 		
    pcl::PointXYZ target3(3,-2,0);
    
    EXPECT_TRUE (wpcheck_validate( pclWaypointCheck( start1 , target1, vOutlines, false ) , input) ); 
	EXPECT_TRUE (wpcheck_validate( pclWaypointCheck( target1, start1,  vOutlines, false ) , input) ); 
    EXPECT_FALSE(wpcheck_validate( pclWaypointCheck( start2 , target2, vOutlines, false ) , input) ); 
	EXPECT_FALSE(wpcheck_validate( pclWaypointCheck( start3 , target3, vOutlines, false ) , input) ); 
}


TEST(LocalPlannerLibTestSuite, testRecursiveBug)
{
     pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/EXPORT10.pcd");    
    ASSERT_TRUE((input.width * input.height) != 0);
    
    pcl::PCLPointCloud2 input_outline = pclConcave_hull(input);
    ASSERT_TRUE((input_outline.width * input_outline.height) != 0);
    
    Pcl2Vector vOutlines;
    vOutlines.push_back(input_outline);
    ASSERT_TRUE(!vOutlines.empty());
    
    pcl::PointXYZ start(0,0,0); 		
    pcl::PointXYZ target(4,0,0);  
    
    pclOptionPoint collisionPoint = pclWaypointCheck( start , target, vOutlines, false );
    ASSERT_TRUE(collisionPoint);
    
    EXPECT_TRUE(rec_bug_validate( pclRecursiveBug(start,target,(*collisionPoint), input_outline), input)) ;
     
}

TEST(LocalPlannerLibTestSuite, testWayPointCheckRecursiveBug)
{
	pcl::PCLPointCloud2 input = read(ros::package::getPath("skynav_tests")+"/include/EXPORT10.pcd");    
    ASSERT_TRUE((input.width * input.height) != 0);
    
    pcl::PCLPointCloud2 input_outline = pclConcave_hull(input);
    ASSERT_TRUE((input_outline.width * input_outline.height) != 0);
    
    Pcl2Vector vOutlines;
    vOutlines.push_back(input_outline);
    ASSERT_TRUE(!vOutlines.empty());
    
    pcl::PointXYZ start1(0,0,0); 		
    pcl::PointXYZ target1(4,0,0);
        
    pcl::PointXYZ start2(0,0,0); 		
    pcl::PointXYZ target2(3,2,0);
    
	pcl::PointXYZ start3(0,0,0); 		
    pcl::PointXYZ target3(3,-2,0);
    
    EXPECT_TRUE (rec_bug_validate( pclWaypointCheck( start1 , target1, vOutlines, true ) , input) ); 
	EXPECT_TRUE (rec_bug_validate( pclWaypointCheck( target1, start1,  vOutlines, true ) , input) ); 
    EXPECT_FALSE(rec_bug_validate( pclWaypointCheck( start2 , target2, vOutlines, true ) , input) ); 
	EXPECT_FALSE(rec_bug_validate( pclWaypointCheck( start3 , target3, vOutlines, true ) , input) ); 	
	
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

