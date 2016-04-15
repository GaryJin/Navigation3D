#include "gtest/gtest.h"
#include <birrt.h>
#include <2dplane/2dplane.h>
#include <2dplane/GridStateSpace.hpp>
#include <3dplane/3dplane.h>
#include <3dplane/GridStateSpace3d.h>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

using namespace RRT;
using namespace Eigen;
using namespace std;

/*
 *  Test the basic function of the birrttree first is the earest function
 *  withought obstacle in thet state space
 *  first test the run function
 *  second test the Impossible Request
 *  third test the getPath funcion
 */
TEST(birrtTreeTest, Example_2dplane) {
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
    BiRRT<Vector2f> *tree = new BiRRT<Vector2f>(_stateSpace);
    //start 5050 goal 400,300
    //  setup birrt
    tree->setStartState(Vector2f(50, 50));
    tree->setGoalState(Vector2f(400, 300));
    tree->setStepSize(30);
    tree->setMaxStepSize(30);
    tree->setGoalMaxDist(12);
    tree->setMaxIterations(10000);

    clock_t start,finish;
    start=clock();
    bool success = tree->run();	//	run with the given starting point
    finish=clock();

    ASSERT_TRUE(success);
    cout<<"Number iteration:"<<tree->iterationCount()<<endl;
    cout<<"solutionLength:"<<tree->getsolutionLength()<<endl;
    double time=(finish-start)*1.0/CLOCKS_PER_SEC;
    cout<<"actual time:"<<time<<"(s)"<<endl;

}
/*
TEST(birrtTreeTest, plane3d_withob) {

    std::shared_ptr<GridStateSpace3d> _stateSpace;
    _stateSpace=make_shared<GridStateSpace3d>(50, 50, 50, 50, 50, 50);

    ifstream in("obstacle3d.txt");
    if (! in.is_open())
           { cout << "Error opening file"; exit (1); }

    while(!in.eof())
    {
        int x=-1,y=-1,z=-1;
        char m;
        in>>x>>m>>y>>m>>z;
        Vector3f pos = Vector3f(x,y,z);
        Vector3i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
        //  toggle the obstacle state of clicked square
        _stateSpace->obstacleGrid().obstacleAt(gridLoc) = true;
    }

    //	give it plenty of iterations so it's not likely to fail
    BiRRT<Vector3f> *tree = new BiRRT<Vector3f>(_stateSpace);
    //  setup birrt
    tree->setStartState(Vector3f(10, 10, 10));
    tree->setGoalState(Vector3f(40, 40, 40));
    tree->setStepSize(7);
    tree->setMaxStepSize(20);
    tree->setGoalMaxDist(3);
    tree->setMaxIterations(5000);
    clock_t start,finish;
    start=clock();
    bool success = tree->run();	//	run with the given starting point
    finish=clock();
    ASSERT_TRUE(success);
    cout<<"Number iteration:"<<tree->iterationCount()<<endl;
    cout<<"solutionLength:"<<tree->getsolutionLength()<<endl;
    double time=(finish-start)*1.0/CLOCKS_PER_SEC;
    cout<<"actual time:"<<time<<"(s)"<<endl;
}*/
GTEST_API_ int main(int argc,char **argv)
{
    //printf("Running main() from gtest_main.cc\n");
    //testing::AddGlobalTestEnvironment(new QtAppEnvironment(argc, argv));

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
