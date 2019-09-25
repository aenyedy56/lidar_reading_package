#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"


ros::Publisher mod_cloud_pub;

//this is the operation that occurs when the subscriber receives the pointcloud from the SICK publisher
void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& lidar_pointcloud){
  //This callback currently writes that it found th-
  ROS_INFO("Received pointcloud");
  //regurgitate the data from the pointcloud received from the lidar
  mod_cloud_pub.publish(*lidar_pointcloud);
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
