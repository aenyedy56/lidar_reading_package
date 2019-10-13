#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/extract_indices.h>
#include "PointCloudSegmenter.cpp"


//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "lidar_listener");
  float theta = -1*M_PI/2;
  ros::NodeHandle n;

  // Create class that listens to lidar messages and published the filtered and segmented data
  PointCloudSegmenter segmenter(135, 225, theta, n);
  //keep running node to obtain messages
  ros::spin();
  return 0;
}