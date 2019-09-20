#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

//this is the operation that occurs when the subscriber receives the pointcloud from the SICK publisher
void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& lidar_pointcloud){
  //This callback currently writes that it found the pointcloud in the terminal window
  //however, we will update this to take the data from the PointCloud2 message received 
  ROS_INFO("I found the pointcloud.");
}

//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "lidar_listener");
  
  //set up the node as a subscriber and specify what topic it is subscribing to (here, "cloud"), the queue of waiting data it can handle (here, 1000), and the operation that will be run when the message is received (here, pointCloudCallBack, the operation we declared at the top of the file) 
  ros::NodeHandle n;
  ros::Subscriber lidar_sub = n.subscribe("cloud", 1000, pointCloudCallBack);

  //keep running node to obtain messages
  ros::spin();

  return 0;
}
