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

// declare publisher to use in subscriber callback
ros::Publisher mod_transformed_pub;



//    use this for aid in your journey:
//    http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
//this is the operation that occurs when the subscriber receives the pointcloud from the SICK publisher
void point_transformed_CallBack(const sensor_msgs::PointCloud2& transformed_pointcloud){
  
  //create a new blank pointcloud object for math operations  
  pcl::PointCloud<pcl::PointXYZ>::Ptr recpc (new pcl::PointCloud<pcl::PointXYZ> ());
  
  //convert transformed_pointcloud (PointCloud2 msg) to recpc (PointCloud obj)
  pcl::fromROSMsg(transformed_pointcloud, *recpc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr front_view (new pcl::PointCloud<pcl::PointXYZ> ());

  int angles[271];
  int start_angle = 180;
  int end_angle = 225;

  for (int i = 0; i<271; i++) {
    angles[i]= i*3;
  }

  for (int i=135*3; i<225*3; i++) {
    pcl::PointXYZ p = recpc->at(i);
    front_view->push_back(p); 
  }  

  front_view->header.frame_id = "cloud";
  front_view->height = 1;
  front_view->width = 270;

  //blank ROS PointCloud2 message to be filled in ros conversion
  sensor_msgs::PointCloud2 lined_pointcloud;  

  //convert segpc PointCloud into a ROS message PointCloud2 for publishing
  pcl::toROSMsg(*front_view, lined_pointcloud);

  //publish the segmented pointcloud
  mod_transformed_pub.publish(lined_pointcloud);


}

//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "front_view_listener");

  //set up the node's subscriber and specify what topic it is subscribing to (here, "cloud"), the queue of waiting data it can handle (here, 1 for best results), and the operation that will be run when the message is received (here, pointCloudCallBack, the operation we declared at the top of the file) 
  ros::NodeHandle nh;
  ros::Subscriber transformed_sub = nh.subscribe("modified_pointcloud", 1, point_transformed_CallBack);
  
  //setup the node's publisher and specify the topic it will publish, with a queue of 1 for best results
  mod_transformed_pub = nh.advertise<sensor_msgs::PointCloud2>("front_view", 1);
  
  //keep running node to obtain messages
  ros::spin();

  return 0;
}
