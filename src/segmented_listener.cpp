#include "ros/ros.h"
#include "StairParameterExtractor.cpp"


//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "stair_param_extractor");
 
  ros::NodeHandle n;

  // Create class that listens to segmented point cloud messages
  StairParameterExtractor segmenter(n);
  //keep running node to obtain messages
  ros::spin();
  return 0;
}