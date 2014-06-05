
#include <gtest/gtest.h>
#include <local_planner_lib.h>
#include "test_localplanner.h"

TEST(LocalPlannerLibTestSuite, testCalcDistance)
{
    
    Point a;
    a.x = 0;
    a.y = 0;
    Point b;
    b.x = 3;
    b.y = 4;
    EXPECT_EQ(5, calcDistance(a, b));
}

TEST(LocalPlannerLibTestSuite, testTruncatValue)
{
    float input = 1.234567890987;
    EXPECT_FLOAT_EQ(1.234, truncateValue(input));
}


TEST(LocalNavTestSuite, testConvexHull)
{
    // pass
    //sensor_msgs::PointCloud input = read(ros::package::getPath("skynav_tests")+"/include/testcases/testCase2D.pcd");    
    //sensor_msgs::PointCloud chull = convex_hull(input); 
    //sensor_msgs::PointCloud ref	= read(ros::package::getPath("skynav_tests")+"/include/testcases/testCase2D_solution.pcd");   
}


TEST(LocalNavTestSuite, testConcaveHull)
{
    // pass
}

TEST(LocalNavTestSuite, testRecursiveBug)
{
    // pass
}


TEST(LocalNavTestSuite, testWayPointCheck)
{
    // pass
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

