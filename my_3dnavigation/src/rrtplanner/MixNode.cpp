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

        ROS_INFO("Road the map points number:%d",SourcePoints.width*SourcePoints.height)  ;
        //sub = private_nh.subscribe<sensor_msgs::PointCloud2>("cloud_output", 1, &RRTPlanner::RoadTheMap, this);
        //pub = private_nh.advertise<sensor_msgs::PointCloud2>("SendOK", 1);
        pub = private_nh.advertise<visualization_msgs::Marker>("SolutionPoints", 100);        
        //pub.publish(SourcePoints);
        //pub = private_nh.advertise<sensor_msgs::PointCloud2>("SendOK", 1);   
        
        
        initialize();
        Vector3f Bias(-0.43, 6.13, 0.15);
        WorldToMap(Bias);  

        initializeRRTStar(Vector3f(0, 50, 80), Vector3f(130, 400, 100), 30, 30);

        step(5000);
        rrtrun();

        RoadToWorld(Bias);
        
        ros::Rate r(0.1);
        while(ros::ok())
        {
            PublishTheResult();
            r.sleep();
        }
    }

    void initialize()
    {
        ROS_INFO("Initialize class.")  ;      
        SolutionPoint.header.frame_id = "/odom";
        SolutionPoint.header.stamp = ros::Time::now();
        SolutionPoint.ns = "SendPoints";
        SolutionPoint.action = visualization_msgs::Marker::ADD;
        SolutionPoint.pose.orientation.w = 1.0;

        SolutionPoint.id = 0;
        SolutionPoint.type = visualization_msgs::Marker::POINTS;

        // POINTS markers use x and y scale for width/height respectively
        SolutionPoint.scale.x = 0.2;
        SolutionPoint.scale.y = 0.2;

        // Points are green
        SolutionPoint.color.g = 1.0f;
        SolutionPoint.color.a = 1.0;



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

    void PublishTheResult()
    {
        ROS_INFO("Publish the Result")  ;      
        pub.publish(SolutionPoint);
    }
private:
    ros::Publisher pub;
    sensor_msgs::PointCloud2 SourcePoints;
    std::shared_ptr<GridStateSpace3d> _stateSpace;
    RRTStar::BiRRTStar<Eigen::Vector3f> *_biRRTStar;
    std::vector<Eigen::Vector3f> _previousSolution;
    visualization_msgs::Marker SolutionPoint;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "RRTPlanner");
    ROS_INFO("gaga");
    RRTPlanner haha;
    //ros::spin();
    return 0;
}
