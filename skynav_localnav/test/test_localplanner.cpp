
#include <gtest/gtest.h>
#include <local_planner_lib.h>

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

