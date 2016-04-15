#include "gtest/gtest.h"
#include <rrtstartree.h>
#include <2dplane/2dplane.h>
#include <2dplane/GridStateSpace.hpp>
#include <vector>
#include <memory>

using namespace RRTStar;
using namespace Eigen;
using namespace std;


TEST(rrtStarTreeTest, Example_2dplane) {
    rrtStarTree<Vector2f> *tree = TreeFor2dPlane(
        make_shared<GridStateSpace>(50, 50, 50, 50),
        Vector2f(40, 40),	//	goal point
        5);					//	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 10000;
    tree->setMaxIterations(maxIterations);
    tree->setGoalMaxDist(5);

    tree->setStartState(Vector2f(10, 10));
    bool success = tree->run();	//	run with the given starting point
    ASSERT_TRUE(success);
}

TEST(rrtStarTree1, FailOnImpossibleRequest) {
    rrtStarTree<Vector2f> *tree = TreeFor2dPlane(
        make_shared<GridStateSpace>(50, 50, 50, 50),
        Vector2f(60, 60),	//	goal point outside the bounds of the state space
        5);					//	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 2000;
    tree->setMaxIterations(maxIterations);
    tree->setGoalMaxDist(5);

    tree->setStartState(Vector2f(10, 10));
    bool success = tree->run();	//	run with the given starting point
    ASSERT_FALSE(success); // the rrt search should fail because the goal isn't reachable
}

TEST(rrtStarTree1, GetPath) {
    rrtStarTree<Vector2f> *tree = TreeFor2dPlane(
        make_shared<GridStateSpace>(50, 50, 50, 50),
        Vector2f(40, 40),	//	goal point
        5);					//	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 10000;
    tree->setMaxIterations(maxIterations);
    tree->setGoalMaxDist(5);

    tree->setStartState(Vector2f(10, 10));
    bool success = tree->run();	//	run with the given starting point
    ASSERT_TRUE(success);

    //	get path in reverse order (end -> root)
    vector<Vector2f> path;
    tree->getPath(path, tree->lastNode(), true);
    ASSERT_TRUE(path.size() > 1);

    //	get path in regular order (root -> end)
    path.clear();
    tree->getPath(path, tree->lastNode(), false);
    ASSERT_TRUE(path.size() > 1);
}
GTEST_API_ int main(int argc,char **argv)
{
    //printf("Running main() from gtest_main.cc\n");
    //testing::AddGlobalTestEnvironment(new QtAppEnvironment(argc, argv));

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
