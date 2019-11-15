#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <vector>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_representation.h>

ros::Subscriber stair_point_cloud_sub;
ros::Publisher stair_xyz_pub;


void stairCloudCallback(const sensor_msgs::PointCloud2& stair_pointcloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr recpc (new pcl::PointCloud<pcl::PointXYZ> ());
    
    pcl::fromROSMsg(stair_pointcloud, *recpc);

	// set up xyz message 
	std_msgs::Float32MultiArray xyz_msg;	

	float vec[recpc->points.size()] = {};
	float* vecp = &vec[0];
	//recpc->points.size()
	// access xyz data from pointcloud
	for(int i = 0; i < recpc->points.size(); i++){
		pcl::DefaultPointRepresentation<pcl::PointXYZ> p; 
    	p.copyToFloatArray(recpc->points[i], vecp);
		if(i == 0){
			xyz_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
			xyz_msg.layout.dim[0].size = 3;
			xyz_msg.layout.dim[0].stride = 1;
			xyz_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1
		}
		xyz_msg.data.insert(xyz_msg.data.end(), &vec[0], &vec[recpc->points.size()-1]);
	}

	stair_xyz_pub.publish(xyz_msg);
    // END PUBLISHING OF ROTATED XYZ DATA FOR USE IN MATPLOTLIB VISUALIZATION


}


//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "xyz_extractor");
  float theta = -1*M_PI/2;
  ros::NodeHandle node;

  stair_point_cloud_sub = node.subscribe("stair_point_cloud", 100, stairCloudCallback);
  stair_xyz_pub = node.advertise<std_msgs::Float32MultiArray>("stair_xyz", 1);


  ros::spin();
  return 0;
}