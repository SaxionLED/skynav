
#include <gtest/gtest.h>
#include <local_planner_lib.h>

using namespace geometry_msgs;

TEST(LocalNavTestSuite, testConvexHull)
{
    // pass
}

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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

