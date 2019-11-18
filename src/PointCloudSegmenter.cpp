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
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <tuple>

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

    pcl::PointCloud<pcl::PointXYZ> pc_buffer;
    int pc_buffer_count = 1;

  PointCloudSegmenter(int sa, int ea, float th, ros::NodeHandle node) {
    this->startAngle = sa;
    this->endAngle = ea;
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
    this->point_cloud_sub = node.subscribe("cloud", 100, &PointCloudSegmenter::unrotatedCallback, this); //unrotated <-> lidarListener
    this->mock_point_cloud_sub = node.subscribe("mock_cloud", 100, &PointCloudSegmenter::lidarListenerCallback, this);
    
   
    this->mod_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("segmented_pointcloud", 1);
    this->rotated_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("rotated_cloud", 1);
    this->xyz_array_pub = node.advertise<std_msgs::Float32MultiArray>("xyz_data", 1);
  }

  void unrotatedCallback(const sensor_msgs::PointCloud2& lidar_pointcloud){
    sensor_msgs::PointCloud2 rotated_pc;
    pcl_ros::transformPointCloud(this->transform, lidar_pointcloud, rotated_pc);
    
    // PUBLISHING ROTATED XYZ DATA FOR USE IN MATPLOTLIB VISUALIZATION
    rotated_cloud_pub.publish(rotated_pc);
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

/*
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

	xyz_array_pub.publish(xyz_msg);
    // END PUBLISHING OF ROTATED XYZ DATA FOR USE IN MATPLOTLIB VISUALIZATION
  */
 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for (int i = 405; i < 675; i++) {
        inliers->indices.push_back(i);
    }
  
    std::cerr << "Pre-Filter" << recpc->size() << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(recpc);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*recpc);
    recpc->header.frame_id = "cloud";

    std::cerr << "Post-Filter" << recpc->size() << std::endl;
    std::cerr<< "Filtered pointcloud ";

    pc_buffer += *recpc;
    pc_buffer_count++;

    std::cerr << "buffer " << pc_buffer_count << std::endl;

    if(pc_buffer_count >= 25) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr recpc2 (new pcl::PointCloud<pcl::PointXYZ> ());
      recpc2 = pc_buffer.makeShared();
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
      seg.setDistanceThreshold (0.01); // perhaps tune this to have higher room for error?
      
      int line_count =0;
      bool foundLine = true;
      do {
        //set the pointcloud that will be source for segmenter
        std::cerr << "Input size " << recpc2->size();
        seg.setInputCloud (recpc2);
        
        //segment the pointcloud, placing results in inliers and coefficients
        seg.segment (*inliers, *coefficients);
        
        std::cerr << "Coefficients " << line_count << std::endl;
        for (int i=0; i<coefficients->values.size(); i++ ) {
          std::cerr << coefficients->values[i] << " ";     
        }
        std::cerr << std::endl;     
        
        //make a copy of the pointcloud recpc, using the inliers indices to determine 
        //which points carry over to new cloud and get visualized
        pcl::copyPointCloud(*recpc2, *inliers, segpc);  

        std::cerr << "results size " << segpc.size() << std::endl;
         for (int i=0; i< inliers->indices.size(); i++ ) {
           segpc.at(i).y = line_count;
        } 

        total_pc += segpc;

        // Filter out points that beloned to the line 
        pcl::ExtractIndices<pcl::PointXYZ> extract; 
        extract.setInputCloud(recpc2);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*recpc2);

        line_count+=1;

      } while(line_count < 100 && recpc2->size() > 10);

      std::cerr<< "Segmented pointcloud ";
      std::cerr<< total_pc.size() << std::endl;
     
      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_pc (new pcl::PointCloud<pcl::PointXYZ> ());
      sort_by_x_dist(total_pc.makeShared(), sorted_pc);
     
      sorted_pc->header.frame_id = "cloud";
      std:: cerr<< "Sorted PC" << sorted_pc->size() << std::endl;
      pcl::toROSMsg(*sorted_pc, total_pc_message);
      //publish modified pointcloud
      mod_cloud_pub.publish(total_pc_message);
      pc_buffer_count = 0;
      recpc2->clear();
      pc_buffer.clear();
    }
  }


    void sort_by_x_dist(pcl::PointCloud<pcl::PointXYZ>::Ptr src_point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dst_point_cloud) {
      pcl::PointXYZ min, max;
      pcl::getMinMax3D(*src_point_cloud, min, max);

      std::vector<std::tuple<double, double, double>> min_point;
      pcl::PointCloud<pcl::PointXYZ> temp_point_cloud;

      for(int i = min.y; i < max.y; i++) {
        pcl::CropBox<pcl::PointXYZ> segmentFilter;
        segmentFilter.setInputCloud(src_point_cloud);
        segmentFilter.setMin(Eigen::Vector4f(-10000, i,-10000, 1.0));
        segmentFilter.setMax(Eigen::Vector4f(10000, i,10000, 1.0));
        segmentFilter.filter(temp_point_cloud);
      
        pcl::PointXYZ min, max;
        pcl::getMinMax3D(temp_point_cloud, min, max);
        std::cerr << min.x << " " << min.y << " " << min.z << std::endl;
        min_point.push_back(std::make_tuple(min.x, min.y, min.z));
      }


      sort(min_point.begin(), min_point.end()); 

      for (int i = 0; i < min_point.size(); i++) {
        std::tuple<int, int, int> p = min_point.at(i);
        double y = std::get<1>(p);
        double x = std::get<0>(p);
        std::cerr << "p " << y <<  " " << x << std::endl;
       
        pcl::CropBox<pcl::PointXYZ> segmentFilter;
        segmentFilter.setInputCloud(src_point_cloud);
        segmentFilter.setMin(Eigen::Vector4f(-10000, y,-10000, 1.0));
        segmentFilter.setMax(Eigen::Vector4f(10000, y,10000, 1.0));
        segmentFilter.filter(temp_point_cloud);

        pcl::PointCloud<pcl::PointXYZ>::iterator it;
        for (it = temp_point_cloud.begin(); it < temp_point_cloud.end(); it++) {
          //std::cerr << it->x << " " << it->y << " " << it->z << std::endl;
          it->y = i;
          //std::cerr << it->x << " " << it->y << " " << it->z << std::endl;
        }

        *dst_point_cloud += temp_point_cloud;
      }
      std::cerr << "sorted_size " << dst_point_cloud->size() << std::endl;
    }

};
