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


class PointCloudSegmenter {
 
 public:

    int angles[270];
    int startAngle;
    int endAngle;
    float theta;
    ros::Publisher mod_cloud_pub;
    ros::Publisher rotated_cloud_pub;
	ros::Publisher xyz_array_pub;
    ros::Subscriber mock_point_cloud_sub;
    ros::Subscriber point_cloud_sub;
    ros::Subscriber modified_point_cloud_sub;
    Eigen::Matrix4f transform;

  PointCloudSegmenter(int sa, int ea, float th, ros::NodeHandle node) {
    startAngle = sa;
    endAngle = ea;
    theta = th; 

    for (int i = 0; i<271; i++) {
      angles[i]= i*3;
    }

    //rotation matrix to use global coordinates (get z)
    //use a rotation about x to convert the coordinates
    this->transform = Eigen::Matrix4f::Identity();
    this->transform(1,1) = std::cos(theta);
    this->transform(1,2) = -sin(theta);
    this->transform(2,1) = sin(theta);
    this->transform(2,2) = std::cos(theta);

    std::cerr << "Subscribing topics" << std::endl;


    this->modified_point_cloud_sub = node.subscribe("modified_pointcloud", 100, &PointCloudSegmenter::unrotatedCallback, this);
    this->point_cloud_sub = node.subscribe("cloud", 100, &PointCloudSegmenter::lidarListenerCallback, this);
    this->mock_point_cloud_sub = node.subscribe("mock_cloud", 100, &PointCloudSegmenter::lidarListenerCallback, this);
    
   
    this->mod_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("segmented_pointcloud", 1);
    this->rotated_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("rotated_cloud", 1);
    this->xyz_array_pub = node.advertise<std_msgs::Float32MultiArray>("xyz_data", 1);
  }

  void unrotatedCallback(const sensor_msgs::PointCloud2& lidar_pointcloud){
    sensor_msgs::PointCloud2 rotated_pc;
    pcl_ros::transformPointCloud(this->transform, lidar_pointcloud, rotated_pc);
    lidarListenerCallback(rotated_pc);

  }

  // This function will perform 3 operations 
  //  1) Rotate the lidar scan by -90 degrees around the x axis 
  //  2) Filter the lidar scan to points with the range startAngle-endAngle
  //  3) Segment the filtered lidar scan to line segments using RANSC placing each line
  //        into a different y plane 
  void lidarListenerCallback(const sensor_msgs::PointCloud2& lidar_pointcloud){
    //This callback currently writes that it found th- *static*
    ROS_INFO("Received pointcloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr recpc (new pcl::PointCloud<pcl::PointXYZ> ());
    
    pcl::fromROSMsg(lidar_pointcloud, *recpc);

    sensor_msgs::PointCloud2 rotated_pc;
    //ros-bag has already rotated data, comment this out for now 
    pcl_ros::transformPointCloud(this->transform, lidar_pointcloud, rotated_pc);

    // rosbag has already rotated data, change it back to rotated-pc later
    pcl::fromROSMsg(rotated_pc, *recpc);
	

    // PUBLISHING ROTATED XYZ DATA FOR USE IN MATPLOTLIB VISUALIZATION
    rotated_cloud_pub.publish(rotated_pc);

	// set up xyz message 
	std_msgs::Float32MultiArray xyz_msg;	

	float vec[recpc->points.size()] = {};
//recpc->points.size()
	// access xyz data from pointcloud
	for(int i = 0; i < recpc->points.size(); i++){
		pcl::DefaultPointRepresentation<pcl::PointXYZ>::copyToFloatArray(recpc->points[i],vec);
		if(i == 0){
			xyz_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
			xyz_msg.layout.dim[0].size = 3;
			xyz_msg.layout.dim[0].stride = 1;
			xyz_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1
		}
		xyz_msg.data.insert(xyz_msg.data.end(), vec.begin(), vec.end());
	}

	xyz_array_pub.publish(xyz_msg);
    // END PUBLISHING OF ROTATED XYZ DATA FOR USE IN MATPLOTLIB VISUALIZATION


    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    for (int i = angles[startAngle]; i < angles[endAngle]; i++) {
        inliers->indices.push_back(i);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(recpc);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*recpc);
    recpc->header.frame_id = "cloud";

    std::cerr<< "Filtered pointcloud ";
    
    //set up blank pointcloud for copying over the segmented data
    pcl::PointCloud<pcl::PointXYZ> segpc;
    //blank pointcloud for storing all lines simultaneously
    pcl::PointCloud<pcl::PointXYZ> total_pc;
    //blank ROS msgs for segmented pointcloud and total pointcloud
    sensor_msgs::PointCloud2 total_pc_message; 

    // taken from pointclouds.org documentation about planar segmentation
    // this is segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
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
       } 

      total_pc += segpc;

      // Filter out points that beloned to the line 
      pcl::ExtractIndices<pcl::PointXYZ> extract; 
      extract.setInputCloud(recpc);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*recpc);

      line_count+=1;

    } while(foundLine && recpc->size() > 10);

    std::cerr<< "Segmented pointcloud";

    total_pc.header.frame_id = "cloud";

    pcl::toROSMsg(total_pc, total_pc_message);
    //publish modified pointcloud
    mod_cloud_pub.publish(total_pc_message);
  }



};
