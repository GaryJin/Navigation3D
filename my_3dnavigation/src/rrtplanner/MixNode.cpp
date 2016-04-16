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

using namespace Eigen;
using namespace std;
using namespace RRTStar;

class RRTPlanner
{
public:
    RRTPlanner()
    {
        ros::NodeHandle private_nh;
        sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_output", ros::Duration(10));        
        
        SourcePoints = *msg;

        ROS_INFO("Road the map points number:%d",SourcePoints.width*SourcePoints.height);
        pub = private_nh.advertise<visualization_msgs::Marker>("SolutionPoints", 100);                
        pubStartEnd = private_nh.advertise<visualization_msgs::Marker>("StartEndpoint", 100);        
        initialize();
        Vector3f Bias(-0.43, 6.13, 0.15);
        WorldToMap(Bias);  

        initializeRRTStar(Vector3f(0, 50, 80), Vector3f(130, 400, 100), 30, 30);

        StartEndpoint.points.push_back(PointToWorld(Vector3f(0, 50, 80), Bias));      
        StartEndpoint.points.push_back(PointToWorld(Vector3f(130, 400, 100), Bias));
        
        step(5000);
        rrtrun();

        RoadToWorld(Bias);
        
        ros::Rate r(0.1);
        PublishTheTree();
        PublishTheResult(r);

    }

    void initialize()
    {
        ROS_INFO("Initialize class.")  ;
        //initialize Marker      
        StartEndpoint.header.frame_id = SolutionPoint.header.frame_id = "/odom";
        StartTree.header.frame_id = EndTree.header.frame_id = "/odom";
        StartEndpoint.header.stamp = SolutionPoint.header.stamp = ros::Time::now();
        StartTree.header.stamp = EndTree.header.stamp = ros::Time::now();
        StartEndpoint.ns = SolutionPoint.ns = "SendPoints";
        StartTree.ns = EndTree.ns = "SendPoints";
        StartEndpoint.action = SolutionPoint.action = visualization_msgs::Marker::ADD;
        StartTree.action = EndTree.action = visualization_msgs::Marker::ADD;
        StartEndpoint.pose.orientation.w = SolutionPoint.pose.orientation.w = 1.0;
        StartTree.pose.orientation.w = EndTree.pose.orientation.w = 1.0;
        SolutionPoint.id = 0;
        StartEndpoint.id = 2;
        StartTree.id = 3;
        EndTree.id = 16;
        StartEndpoint.type = SolutionPoint.type = visualization_msgs::Marker::SPHERE_LIST;
        StartTree.type = EndTree.type = visualization_msgs::Marker::LINE_LIST;
        // POINTS markers use x and y scale for width/height respectively
        StartEndpoint.scale.x = SolutionPoint.scale.x = 0.2;
        StartEndpoint.scale.y = SolutionPoint.scale.y = 0.2;
        
        StartTree.scale.x = EndTree.scale.x = 0.01;
        StartTree.scale.y = EndTree.scale.y = 0.01;

        // Points are green
        SolutionPoint.color.g = 1.0f;
        SolutionPoint.color.a = 1.0;
        //Start and end point red
        StartEndpoint.color.r = 1.0;
        StartEndpoint.color.a = 1.0;
        // Start tree  is blue
        StartTree.color.b = 1.0;
        StartTree.color.a = 1.0;
        // End tree  is blue
        EndTree.color.a = 1.0;
        EndTree.color.r = 1.0;
        EndTree.color.g = 0.5;
        //initialzie statespace
        _stateSpace = make_shared<GridStateSpace3d>(260, 
                                        470,
                                        160,
                                        26,
                                        47,
                                        16);
                                                   
    }

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

        sensor_msgs::PointCloud2ConstIterator<float> x_in(SourcePoints, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_in(SourcePoints, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_in(SourcePoints, "z");
        for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in)
        {
            Vector3f pos = Vector3f((*x_in),(*y_in),(*z_in));
            pos = pos + Bias;
            pos = pos * 50;
            Vector3i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
            _stateSpace->obstacleGrid().obstacleAt(gridLoc) = true;

        }
        
    }

    void RoadToWorld(Vector3f Bias)
    {
        ROS_INFO("Translate the road to world")  ;      

        SolutionPoint.points.clear();
        for(const Vector3f &curr : _previousSolution)
        {
            geometry_msgs::Point p;
            Vector3f mapPoint = curr / 50;
            mapPoint = mapPoint - Bias;
            p.x = mapPoint.x();
            p.y = mapPoint.y();
            p.z = mapPoint.z();
            SolutionPoint.points.push_back (p);
        }
    }

    Vector3f PointTomap(const Vector3f point_in, Vector3f Bias)
    {
            Vector3f point_out = point_in + Bias;
            point_out = point_out * 50;
            return point_out; 
    }

    geometry_msgs::Point PointToWorld(const Vector3f point_in, Vector3f Bias)
    {
            geometry_msgs::Point point_out;
            Vector3f mapPoint = point_in / 50;
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
                , Vector3f(-0.43, 6.13, 0.15));

            if (node->parent()) {
                //  draw edge
                geometry_msgs::Point parentLoc = PointToWorld(
                    Vector3f(node->parent()->state().x(),
                        node->parent()->state().y(),
                        node->parent()->state().z())
                        , Vector3f(-0.43, 6.13, 0.15));
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
                , Vector3f(-0.43, 6.13, 0.15));

            if (node->parent()) {
                //  draw edge
                geometry_msgs::Point parentLoc = PointToWorld(
                    Vector3f(node->parent()->state().x(),
                        node->parent()->state().y(),
                        node->parent()->state().z())
                        , Vector3f(-0.43, 6.13, 0.15));
                EndTree.points.push_back(loc);
                EndTree.points.push_back(parentLoc);
            }
        }
        //End tree

    }


private:
    ros::Publisher pub;
    ros::Publisher pubStartEnd;
    sensor_msgs::PointCloud2 SourcePoints;
    std::shared_ptr<GridStateSpace3d> _stateSpace;
    RRTStar::BiRRTStar<Eigen::Vector3f> *_biRRTStar;
    std::vector<Eigen::Vector3f> _previousSolution;
    visualization_msgs::Marker SolutionPoint;
    visualization_msgs::Marker StartEndpoint;
    visualization_msgs::Marker StartTree;
    visualization_msgs::Marker EndTree;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "RRTPlanner");
    ROS_INFO("Run RRT Planner");
    RRTPlanner _rrtplanner;
    //ros::spin();
    return 0;
}
