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

//tf
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
//namespace
using  std::string;

//creater a publisher
ros::Publisher pub;
ros::Publisher pub1;


void doTransform(const sensor_msgs::PointCloud2 &p_in, sensor_msgs::PointCloud2 &p_out, const geometry_msgs::TransformStamped& t_in)
{
  p_out = p_in;
  p_out.header = t_in.header;
  Eigen::Transform<float,3,Eigen::Affine> t = Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
                                                                   t_in.transform.translation.z) * Eigen::Quaternion<float>(
                                                                     t_in.transform.rotation.w, t_in.transform.rotation.x,
                                                                     t_in.transform.rotation.y, t_in.transform.rotation.z);

  sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, "z");

  sensor_msgs::PointCloud2Iterator<float> x_out(p_out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(p_out, "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(p_out, "z");

  Eigen::Vector3f point;
  for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
    point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
    *x_out = point.x();
    *y_out = point.y();
    *z_out = point.z();
  }
}
//void doTransform(const sensor_msgs::PointCloud2 , sensor_msgs::PointCloud2 , const geometry_msgs::TransformStamped& );


//main
int main (int argc, char** argv)
{
  //print error
  if(argc<4)
  {
	std::cerr<<"Please input the path and the number of the pcd file and the compress precision(example :/home/gary/ 10 0.05)";
	exit(0);
  }
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");//register the node
  ros::NodeHandle nh;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2 cloud_mixed;
  pcl::PCDReader reader;

  ros::Rate send_rate(1);//set the frequencly 1Hz
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_output", 1);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output",1);
  //read the PointCloud2 data from a pcd file
  int err;
  string pcdfilepath=argv[1];
  for(int i=0;i<atoi(argv[2])+1;i++)
  {
  	//transform int to string
  	std::stringstream ss;
  	string num;
  	ss<<i;
  	ss>>num;

  	string pcdfile=pcdfilepath+num+".pcd";
  	err=reader.read (pcdfile, *cloud);
  	std::cout<<"Read"<<pcdfile<<std::endl; 
	  if(err!=0)
  	{
		  ROS_INFO("error path");
		  exit(0);
  	}  
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
	  double prec=atof(argv[3]);
    sor.setLeafSize (prec, prec, prec);
    sor.filter (cloud_filtered);
	  err=pcl::concatenatePointCloud(cloud_mixed, cloud_filtered, cloud_mixed);//connect the point
	  if(err==0)
	  {
		  ROS_INFO("Connect error");
		  exit(0);
	  }
  }
  ROS_INFO("Total point: %d",cloud_mixed.width *cloud_mixed.height);  


  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 cloud_out;
  geometry_msgs::TransformStamped tf_transform;
  

  pcl_conversions::fromPCL(cloud_mixed, output);
  // pcl::toROSMsg(cloud_mixed,output);the way convert the PointCloud to ros message
  //output.header.frame_id="/odom";//set the frame id
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "sensor_frame1";

  tf_transform.header.stamp=ros::Time::now();
  tf_transform.header.frame_id = "odom";
  tf_transform.child_frame_id = "sensor_frame1";

  ROS_INFO("Publish pointcloud message frame is base_link");
  doTransform (output, cloud_out, tf_transform);
  while(ros::ok())
  {
        // Publish the data
        pub1.publish(output);
        pub.publish (cloud_out);
        static_tf_broadcaster_.sendTransform(tf_transform);
        send_rate.sleep();
  }

}


