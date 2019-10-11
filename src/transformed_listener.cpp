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

  //set up blank pointcloud for copying over the segmented data
  pcl::PointCloud<pcl::PointXYZ> segpc;
  //blank pointcloud for storing all lines simultaneously
  pcl::PointCloud<pcl::PointXYZ> total_pc;
  //blank ROS msgs for segmented pointcloud and total pointcloud
  sensor_msgs::PointCloud2 sec_pc_message; 
  sensor_msgs::PointCloud2 total_pc_message; 


  // taken from pointclouds.org documentation about planar segmentation
  // this is segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;


  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE); // looking for lines in the data
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02); // perhaps tune this to have higher room for error?
  

  int line_count =0;
  bool foundLine = true;
  do {
    // // taken from pointclouds.org documentation about planar segmentation
    // // this is segmentation
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;

    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_LINE); // looking for lines in the data
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.02); // perhaps tune this to have higher room for error?
    
    //set the pointcloud that will be source for segmenter
    std::cerr << "Input size " << recpc->size();
    seg.setInputCloud (recpc);
    
    //segment the pointcloud, placing results in inliers and coefficients
    seg.segment (*inliers, *coefficients);
    
    std::cerr << "Coefficients " << line_count << std::endl;
    for (int i=0; i<coefficients->values.size(); i++ ) {
      std::cerr << coefficients->values[i] << " ";     
    }
    std::cerr << std::endl;     
    
    //make a copy of the pointcloud recpc, using the inliers indices to determine 
    //which points carry over to new cloud and get visualized
    pcl::copyPointCloud(*recpc, *inliers, segpc);  

    std::cerr << "results size " << segpc.size() << std::endl;
     for (int i=0; i< inliers->indices.size(); i++ ) {
       segpc.at(i).y = line_count;
       //std::cerr << "item " << segpc.at(i).x << " " << segpc.at(i).z << " " << segpc.at(i).y << std::endl;
     } 

    total_pc += segpc;

    // Filter out points that beloned to the line 
    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(recpc);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*recpc);

    line_count+=1;

  } while(foundLine && line_count < 5 && recpc->size() > 10);
  //while(foundLine && line_count < 1);
  //if (!sac_->computeModel (0))
  //!coefficients->values.empty() 

  total_pc.header.frame_id = "cloud";


  pcl::toROSMsg(total_pc, total_pc_message);
  //publish the segmented pointcloud
  //formerly:mod_transformed_pub.publish(lined_pointcloud); 
  mod_transformed_pub.publish(total_pc_message);

}

//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){

  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "tranformed_listener");
  
  //set up the node's subscriber and specify what topic it is subscribing to (here, "cloud"), the queue of waiting data it can handle (here, 1 for best results), and the operation that will be run when the message is received (here, pointCloudCallBack, the operation we declared at the top of the file) 
  ros::NodeHandle nh;
  ros::Subscriber transformed_sub = nh.subscribe("front_view", 1, point_transformed_CallBack);
  
  //setup the node's publisher and specify the topic it will publish, with a queue of 1 for best results
  mod_transformed_pub = nh.advertise<sensor_msgs::PointCloud2>("modified_transformed", 1);
  
  //keep running node to obtain messages
  ros::spin();

  return 0;
}
