#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

ros::Publisher mod_transformed_pub;

//this is the operation that occurs when the subscriber receives the pointcloud from the SICK publisher
void point_transformed_CallBack(const sensor_msgs::PointCloud2::ConstPtr& transformed_pointcloud){
  //This callback currently writes that it found th-
  ROS_INFO("Received pointcloud");
  //regurgitate the data from the pointcloud received from the lidar
  mod_transformed_pub.publish(*transformed_pointcloud);

  
}

//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "tranformed_listener");
  
  //set up the node's subscriber and specify what topic it is subscribing to (here, "cloud"), the queue of waiting data it can handle (here, 1 for best results), and the operation that will be run when the message is received (here, pointCloudCallBack, the operation we declared at the top of the file) 
  ros::NodeHandle nh;
  ros::Subscriber transformed_sub = nh.subscribe("modified_pointcloud", 1, point_transformed_CallBack);
  
  //setup the node's publisher and specify the topic it will publish, with a queue of 1 for best results
  mod_transformed_pub = nh.advertise<sensor_msgs::PointCloud2>("modified_transformed", 1);
  
  //keep running node to obtain messages
  ros::spin();

  return 0;
}
