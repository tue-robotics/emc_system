#include <gtest/gtest.h>

#include <emc/io.h>

#include <vector>

TEST(Io, SendPath)
{
    emc::IO io;

    std::vector<double> point;
    point.push_back(0.0);
    point.push_back(0.0);

    std::vector<std::vector<double>> path;
    path.push_back(point);
    path.push_back(point);

    bool res = io.sendPath(path);
    EXPECT_TRUE(res);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
