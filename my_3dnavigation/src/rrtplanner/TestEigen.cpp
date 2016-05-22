#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>
#include <iostream>
#include <float.h>

#include <3dplane/3dplane.h>
#include <3dplane/GridStateSpace3d.h>

using namespace Eigen;
using namespace std;

Vector3f Bias(-0.43, 6.13, 0.15);
//Vector3f Bias(30000, 26000, 2310);
//Vector3f Bias(4850, 6550, 0);

vector<Vector3f> ObstacleCloud;

void outputCallback(const sensor_msgs::PointCloud2 &mapmsg)
{
    ObstacleCloud.clear();
    sensor_msgs::PointCloud2ConstIterator<float> x_in(mapmsg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_in(mapmsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_in(mapmsg, "z");
    Vector3f point;
    float x_inmax=-FLT_MAX,x_inmin=FLT_MAX,y_inmax=-FLT_MAX,
            y_inmin=FLT_MAX,z_inmax=-FLT_MAX,z_inmin=FLT_MAX;
    for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in) {
      point = Eigen::Vector3f((*x_in), (*y_in), (*z_in));
      point = point+Bias;
      point = point*50;
      if(point.x()>x_inmax)
          x_inmax=point.x();
      if(point.y()>y_inmax)
          y_inmax=point.y();
      if(point.z()>z_inmax)
          z_inmax=point.z();
      if(point.x()<x_inmin)
          x_inmin=point.x();
      if(point.y()<y_inmin)
          y_inmin=point.y();
      if(point.z()<z_inmin)
          z_inmin=point.z();
      ObstacleCloud.push_back(point);
    }
    cout<<x_inmin<<"<x<"<<x_inmax<<endl;
    cout<<y_inmin<<"<y<"<<y_inmax<<endl;
    cout<<z_inmin<<"<z<"<<z_inmax<<endl;
    cout<<"size:"<<ObstacleCloud.size()<<endl;
 }

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "TestEigen");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("cloud_output", 1, outputCallback);
    ros::spin();

    return 0;
}
