#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "SendPoints");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("SolutionPoints", 10);


  ros::Rate r(0.1);

  while (ros::ok())
  {

    visualization_msgs::Marker points;
    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "SendPoints";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;


    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;


    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 10; ++i)
    {
      float y = 5 * sin(i / 100.0f * 2 * M_PI);
      float z = 5 * cos(i / 100.0f * 2 * M_PI);
      y = 5 * drand48();
      z = 5 * drand48();
      geometry_msgs::Point p;
      p.x = i * 10;
      p.y = y;
      p.z = z;

      points.points.push_back(p);

    }


    marker_pub.publish(points);

    r.sleep();
  }
}
