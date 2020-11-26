#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_load");

  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("map", 1);
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("/home/yellow/self-driving-cars-course/SDC_HW4/bag/map_downsample.pcd", pointcloud);
  //Convert the cloud to ROS message
  pcl::toROSMsg(pointcloud, output);
  output.header.frame_id = "/world";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
   pcl_pub.publish(output);
   ros::spinOnce();
   loop_rate.sleep();
  }
  return 0;

}
