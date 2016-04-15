#include "gtest/gtest.h"
#include <rrtstartree.h>
#include <2dplane/2dplane.h>
#include <2dplane/GridStateSpace.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

using namespace RRTStar;
using namespace Eigen;
using namespace std;

/*
 *  Test the basic function of the rrtstartree first is the earest function
 *
 *
 */
TEST(rrtStarTreeTest, Example_2dplane) {
    rrtStarTree<Vector2f> *tree = rrtStarTreeFor2dPlane(
        make_shared<GridStateSpace>(50, 50, 50, 50),
        Vector2f(40, 40),	//	goal point
        10);					//	step size

    //	give it plenty of iterations so it's not likely to fail
    const int maxIterations = 10000;
    tree->setMaxIterations(maxIterations);
    tree->setGoalMaxDist(5);

    tree->setStartState(Vector2f(10, 10));
    bool success = tree->run();	//	run with the given starting point
    ASSERT_TRUE(success);
}
/*
TEST(rrtStarTreeTest, FailOnImpossibleRequest) {
    rrtStarTree<Vector2f> *tree = rrtStarTreeFor2dPlane(
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
*/
TEST(rrtStarTreeTest, GetPath) {
    rrtStarTree<Vector2f> *tree = rrtStarTreeFor2dPlane(
        make_shared<GridStateSpace>(50, 50, 50, 50),
        Vector2f(40, 40),	//	goal point
        10);					//	step size

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
TEST(rrtStarTreeTest, RunwithObstacle) {
    std::shared_ptr<GridStateSpace> _stateSpace;
    _stateSpace=make_shared<GridStateSpace>(800, 600, 40, 30);

    ifstream in("obstacle2d.txt");
    if (! in.is_open())
           { cout << "Error opening file"; exit (1); }

    while(!in.eof())
    {
        int x=-1,y=-1;
        char z;
        in>>x>>z>>y;
        Vector2f pos = Vector2f(x,y);

        Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
        bool _erasingObstacles = _stateSpace->obstacleGrid().obstacleAt(gridLoc);
        //  toggle the obstacle state of clicked square
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = !_erasingObstacles;
    }

    //	give it plenty of iterations so it's not likely to fail
    rrtStarTree<Vector2f> *tree = rrtStarTreeFor2dPlane(_stateSpace,
        Vector2f(400, 300),	//	goal point
        10);					//	step size
    //start 5050 goal 400,300

    const int maxIterations = 10000;
    tree->setMaxIterations(maxIterations);
    tree->setGoalMaxDist(5);

    tree->setStartState(Vector2f(50, 50));
    bool success = tree->run();	//	run with the given starting point
    ASSERT_TRUE(success);
    cout<<"Number iteration:"<<tree->allNodes().size();

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
