//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

//standard include
#include <iostream>
#include <string>
#include <stdlib.h>
#include <ctime>

//msgs
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//tf
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

//rrt
#include <3dplane/GridStateSpace3d.h>
#include <birrtstar.h>
#include <3dplane/3dplane.h>
#include <planning/Path.hpp>

//bezier3d
#include <bezier3d.h>

using namespace Eigen;
using namespace std;
using namespace RRTStar;

/* Countertop
 * Bias(-0.43, 6.13, 0.15)
 * Start = Vector3f(52, 190, 30);
 * End = Vector3f(45, 370, 90);
 * Scaler = 0.02;
 * 260, 470, 160, 26, 47, 16
 */

/* Theater
 * Bias(30000, 26000, 500)
 * Start = Vector3f(57, 180, 50);
 * End = Vector3f(530, 250, 10);
 * Scaler = 100;
 * 540, 510, 200, 54, 51, 20
 */

/* Overpass
 * Bias(4850, 6550, 500)
 * Start = Vector3f(50, 50, 25);
 * End = Vector3f(340, 250, 25);
 * Scaler = 40;
 * 360, 300, 110, 36, 30, 11
 */


/* 
 * @brief This is the map param for Countertop & Theater & Overpass
 * If you want to use it in your map
 * I suggest you change the code using the .yaml to road the map
 */
struct Planningmap
{
    Vector3f Bias;
    Vector3f Start;
    Vector3f End;
    double Scaler;
    int x_scaler;
    int y_scaler;
    int z_scaler;
};

Planningmap Countertop = {
    Vector3f(-0.43, 6.13, 0.15),
    Vector3f(52, 190, 30),
    Vector3f(45, 370, 90),
    0.02,
    260,
    470,
    160
};
//,Theater,Overpass,mapnow;
Planningmap Theater = {
    Vector3f(30000, 26000, 500),
    Vector3f(57, 250, 50),
    Vector3f(530, 250, 10),
    100,
    540,
    510,
    200
};

Planningmap Overpass = {
    Vector3f(4850, 6550, 500),
    Vector3f(50, 50, 25),
    Vector3f(340, 250, 25),
    40,
    360,
    300,
    110
};

Planningmap mapnow = Theater;

class RRTPlanner
{
public:
    RRTPlanner()
    {
        //initialize the node handler
        ros::NodeHandle private_nh;

        //receive the map msg from the topic
        sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_output", ros::Duration(10));        
        
        sourcePoints = *msg;


        ROS_INFO("Road the map points number:%d",sourcePoints.width*sourcePoints.height);
        
        //initialize the publisher
        pub = private_nh.advertise<visualization_msgs::Marker>("FinalSolutionPoints", 10);                
        pubStartEnd = private_nh.advertise<visualization_msgs::Marker>("StartEndpoint", 10);        
        pubFinalLine = private_nh.advertise<visualization_msgs::Marker>("FinalSolutionLine", 1);        
        //pubFinalCurve = private_nh.advertise<visualization_msgs::Marker>("FinalSolutionCurve", 1);  

        //The scaler to the map
        Scaler = mapnow.Scaler;

        initialize();

        Vector3f Bias = mapnow.Bias;
        WorldToMap(Bias);  

        initializeRRTStar(Start, End, 50, 30);
        
        if(!(_stateSpace->stateValid(Start)))
        {
            ROS_INFO("Start point invalid!");
            exit(0);
        }
        if(!(_stateSpace->stateValid(End)))
        {
            ROS_INFO("End point invaild!");
            exit(0);
            //ros::shutdown();
        }

        StartEndpoint.points.push_back(PointToWorld(Start, Bias));      
        StartEndpoint.points.push_back(PointToWorld(End, Bias));

        
        clock_t start,finish;//count the algorithm time

        start = clock();
        step(10000);
        finish = clock();
        double time=(finish-start)*1.0/CLOCKS_PER_SEC;
        ROS_INFO("Algorithm run time:%f(s)",time);
        //rrtrun();

        
        //show the Length
        double Length = 0;
        for(std::vector<Eigen::Vector3f>::iterator iter = _previousSolution.begin(); iter < _previousSolution.end()-1; iter++)
        {
            Length += _stateSpace->distance(*iter,*(iter+1));
        }

        ROS_INFO("Final result length: %f",Length);

        RoadToWorld(Bias);
                
        ros::Rate r(0.1);
        PublishTheTree();
        CreateCurve(Bias);
        PublishTheResult(r);

    }

    /*
     * @brief Initialize  
     * 1. Start&End point
     * 2. Visualization marker
     * 3. The statespace
     */
    void initialize()
    {
        ROS_INFO("Initialize class.");

        Start = mapnow.Start;
        End = mapnow.End;
        //initialize Marker      
        StartEndpoint.header.frame_id = SolutionPoint.header.frame_id = "/odom";
        StartTree.header.frame_id = EndTree.header.frame_id = "/odom";
        SolutionCurve.header.frame_id = SolutionLine.header.frame_id = "/odom";
       
        StartEndpoint.header.stamp = SolutionPoint.header.stamp = ros::Time::now();
        StartTree.header.stamp = EndTree.header.stamp = ros::Time::now();
        SolutionCurve.header.stamp = SolutionLine.header.stamp = ros::Time::now();

        StartEndpoint.ns = SolutionPoint.ns = "SendPoints";
        StartTree.ns = EndTree.ns = "SendPoints";
        SolutionCurve.ns = SolutionLine.ns = "SendPoints1";

        StartEndpoint.action = SolutionPoint.action = visualization_msgs::Marker::ADD;
        StartTree.action = EndTree.action = visualization_msgs::Marker::ADD;
        SolutionCurve.action = SolutionLine.action = visualization_msgs::Marker::ADD;

        StartEndpoint.pose.orientation.w = SolutionPoint.pose.orientation.w = 1.0;
        StartTree.pose.orientation.w = EndTree.pose.orientation.w = 1.0;
        SolutionCurve.pose.orientation.w = SolutionLine.pose.orientation.w = 1.0;

        SolutionPoint.id = 0;
        StartEndpoint.id = 2;
        StartTree.id = 3;
        SolutionLine.id = 4;
        SolutionCurve.id = 5;
        EndTree.id = 16;

        
        StartEndpoint.type = SolutionPoint.type = visualization_msgs::Marker::SPHERE_LIST;
        StartTree.type = EndTree.type = visualization_msgs::Marker::LINE_LIST;
        SolutionCurve.type = SolutionLine.type = visualization_msgs::Marker::LINE_STRIP;
        
        // POINTS markers use x and y scale for width/height respectively
        StartEndpoint.scale.x = SolutionPoint.scale.x = Scaler*2;
        StartEndpoint.scale.y = SolutionPoint.scale.y = Scaler*2;
        
        SolutionLine.scale.x = SolutionCurve.scale.x = Scaler;
        SolutionCurve.scale.y = SolutionLine.scale.y = Scaler;
        
        StartTree.scale.x = EndTree.scale.x = Scaler;
        StartTree.scale.y = EndTree.scale.y = Scaler;


        //yellow
        SolutionLine.color.r = 1.0;
        SolutionLine.color.g = 1.0;
        SolutionLine.color.a = 1.0;

        //Red
        SolutionCurve.color.r = 1.0;
        SolutionCurve.color.a = 1.0;

        // Points are green
        SolutionPoint.color.g = 1.0f;
        SolutionPoint.color.a = 1.0;
        
        //Start and end point red
        StartEndpoint.color.r = 1.0;
        StartEndpoint.color.a = 1.0;
        
        // Start tree  is blue
        StartTree.color.b = 1.0;
        StartTree.color.a = 1.0;
        
        // End tree  is yellow
        EndTree.color.a = 1.0;
        EndTree.color.r = 1.0;
        EndTree.color.g = 0.5;

        //initialzie statespace
        _stateSpace = make_shared<GridStateSpace3d>(mapnow.x_scaler, 
                                        mapnow.y_scaler,
                                        mapnow.z_scaler,
                                        mapnow.x_scaler/10,
                                        mapnow.y_scaler/10,
                                        mapnow.z_scaler/10);
                                                   
    }

    /*
     * @brief initialize the MB-RRT* tree
     * @param 1.Start point, 2.End point, 3.Tree grow stepsize, 4.The max judge distance for connecting
     */
    void initializeRRTStar(const Vector3f startpoint, const Vector3f goalpoint, int stepsize, int MaxDist)
    {        
        ROS_INFO("Initialize MBRRTStar Tree.")  ;      

        _biRRTStar = new BiRRTStar<Vector3f>(_stateSpace);
        //  setup birrtStar
        _biRRTStar->setStartState(startpoint);
        _biRRTStar->setGoalState(goalpoint);
        _biRRTStar->setStepSize(stepsize);
        _biRRTStar->setMaxStepSize(1.5*stepsize);
        _biRRTStar->setGoalMaxDist(MaxDist);
        _biRRTStar->setASCEnabled(1);      
    }

    void step(int numTimes)
    {
        for(int i = 0; i<numTimes; i++)
        {
            _biRRTStar->grow();
        }
        _previousSolution.clear();
        if (_biRRTStar->startSolutionNode() != nullptr || _biRRTStar->goalSolutionNode() != nullptr) {
            _biRRTStar->getPath(_previousSolution);
            Planning::SmoothPath<Vector3f>(_previousSolution, *_stateSpace);  
        }
    }

    void rrtrun()
    {
        while(_biRRTStar->startSolutionNode() == nullptr) {
            step(1);
        }

    }

    void WorldToMap(Vector3f Bias)
    {
        sensor_msgs::PointCloud2ConstIterator<float> x_in(sourcePoints, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_in(sourcePoints, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_in(sourcePoints, "z");
        int obcount = 0;
        for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in)
        {
            Vector3f pos = Vector3f((*x_in),(*y_in),(*z_in));
            pos = pos + Bias;
            pos = pos / Scaler;
            Vector3i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
            if(!(_stateSpace->obstacleGrid().obstacleAt(gridLoc)))
                obcount++;
            _stateSpace->obstacleGrid().obstacleAt(gridLoc) = true;

        }
        ROS_INFO("Total obstacle number:%d",obcount);

    }

    void CreateCurve(Vector3f Bias)
    {
        ROS_INFO("Generate the curve");
        _finalCurve.clear();
        SolutionCurve.points.clear();
        Bezier3d test;
        test.ComputeCurve(*_stateSpace, _previousSolution, _finalCurve);
        for(const Vector3f &curr : _finalCurve)
        {
            geometry_msgs::Point p;
            Vector3f mapPoint = curr * Scaler;
            mapPoint = mapPoint - Bias;
            p.x = mapPoint.x();
            p.y = mapPoint.y();
            p.z = mapPoint.z();
            SolutionCurve.points.push_back(p);
        }
    }

    void RoadToWorld(Vector3f Bias)
    {
        ROS_INFO("Translate the road to world")  ;      

        SolutionPoint.points.clear();
        SolutionLine.points.clear();
        
        for(const Vector3f &curr : _previousSolution)
        {
            geometry_msgs::Point p;
            Vector3f mapPoint = curr * Scaler;
            mapPoint = mapPoint - Bias;
            p.x = mapPoint.x();
            p.y = mapPoint.y();
            p.z = mapPoint.z();
            SolutionPoint.points.push_back (p);
            SolutionLine.points.push_back(p);
        }
    }

    Vector3f PointTomap(const Vector3f point_in, Vector3f Bias)
    {
            Vector3f point_out = point_in + Bias;
            point_out = point_out / Scaler;
            return point_out; 
    }

    geometry_msgs::Point PointToWorld(const Vector3f point_in, Vector3f Bias)
    {
            geometry_msgs::Point point_out;
            Vector3f mapPoint = point_in * Scaler;
            mapPoint = mapPoint - Bias;
            point_out.x = mapPoint.x();
            point_out.y = mapPoint.y();
            point_out.z = mapPoint.z();
            return point_out;
    }

    void PublishTheResult(ros::Rate r)
    {
        ROS_INFO("Publish the Result");      
        while(ros::ok())
        {
            pub.publish(SolutionPoint);
            pubStartEnd.publish(StartEndpoint);
            pubStartEnd.publish(StartTree);
            pubStartEnd.publish(EndTree);
            pubFinalLine.publish(SolutionLine);
            pubFinalLine.publish(SolutionCurve);

            r.sleep();
        }

    }

    void PublishTheTree()
    {
        //Start tree
        rrtStarTree<Vector3f> rrtstart = _biRRTStar->startTree();

        //  draw all the nodes and connections
        for (const Node<Vector3f> *node : rrtstart.allNodes()) {
            geometry_msgs::Point loc = PointToWorld(
                Vector3f(node->state().x(), 
                    node->state().y(),
                    node->state().z())
                , mapnow.Bias);

            if (node->parent()) {

                //  draw edge
                geometry_msgs::Point parentLoc = PointToWorld(
                    Vector3f(node->parent()->state().x(),
                        node->parent()->state().y(),
                        node->parent()->state().z())
                        , mapnow.Bias);
                StartTree.points.push_back(loc);
                StartTree.points.push_back(parentLoc);
            }
        }
        
        rrtStarTree<Vector3f> rrtend = _biRRTStar->goalTree();
        //  draw all the nodes and connections
        for (const Node<Vector3f> *node : rrtend.allNodes()) {
            geometry_msgs::Point loc = PointToWorld(
                Vector3f(node->state().x(), 
                    node->state().y(),
                    node->state().z())
                , mapnow.Bias);

            if (node->parent()) {
                //  draw edge
                geometry_msgs::Point parentLoc = PointToWorld(
                    Vector3f(node->parent()->state().x(),
                        node->parent()->state().y(),
                        node->parent()->state().z())
                        , mapnow.Bias);
                EndTree.points.push_back(loc);
                EndTree.points.push_back(parentLoc);
            }
        }
        //End tree

    }


private:
    //ROS message
    ros::Publisher pub;
    ros::Publisher pubStartEnd;
    ros::Publisher pubFinalLine;
    ros::Publisher pubFinalCurve;

    sensor_msgs::PointCloud2 sourcePoints;
    
    //Birrtstar
    std::shared_ptr<GridStateSpace3d> _stateSpace;
    RRTStar::BiRRTStar<Eigen::Vector3f> *_biRRTStar;

    std::vector<Eigen::Vector3f> _previousSolution;
    std::vector<Eigen::Vector3f> _finalCurve;

    Vector3f Start,End;

    double Scaler;

    //Visualization Marker
    visualization_msgs::Marker SolutionPoint;
    visualization_msgs::Marker SolutionCurve;
    visualization_msgs::Marker SolutionLine;
    visualization_msgs::Marker StartEndpoint;
    visualization_msgs::Marker StartTree;
    visualization_msgs::Marker EndTree;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "RRTPlanner");
    ROS_INFO("Run RRT Planner");
    RRTPlanner _rrtplanner;

    return 0;
}
