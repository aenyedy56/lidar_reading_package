#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

ros::Publisher mod_cloud_pub;

//this is the operation that occurs when the subscriber receives the pointcloud from the SICK publisher
void pointCloudCallBack(const sensor_msgs::PointCloud2& lidar_pointcloud){
  //This callback currently writes that it found th-
  ROS_INFO("Received pointcloud");
  //regurgitate the data from the pointcloud received from the lidar

  //rotation matrix to use global coordinates (get z)
  //use a rotation about x to convert the coordinates
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  float theta = M_PI/2; //angle of rotations in radians
  transform_1(1,1) = std::cos(theta);
  transform_1(1,2) = -sin(theta);
  transform_1(2,1) = sin(theta);
  transform_1(2,2) = std::cos(theta);

  const Eigen::Matrix4f transform = transform_1;
  
  //set up blank pointcloud to store transformed pointcloud
  sensor_msgs::PointCloud2 modified_pointcloud;
  
  //apply transform
  pcl_ros::transformPointCloud (transform, lidar_pointcloud, modified_pointcloud);  
  
  //publish modified pointcloud
  mod_cloud_pub.publish(modified_pointcloud);
}

//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "lidar_listener");
  
  //set up the node's subscriber and specify what topic it is subscribing to (here, "cloud"), the queue of waiting data it can handle (here, 1 for best results), and the operation that will be run when the message is received (here, pointCloudCallBack, the operation we declared at the top of the file) 
  ros::NodeHandle n;
  ros::Subscriber lidar_sub = n.subscribe("cloud", 1, pointCloudCallBack);
  
  //setup the node's publisher and specify the topic it will publish, with a queue of 1 for best results
  mod_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("modified_pointcloud", 1);
  
  //keep running node to obtain messages
  ros::spin();

  return 0;
}
