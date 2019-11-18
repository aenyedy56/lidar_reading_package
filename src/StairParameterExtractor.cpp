#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "lidar_reading_package/Stair.h"
#include "lidar_reading_package/Stairs.h"
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/impl/angles.hpp>

class StairParameterExtractor {

public:

    ros::Subscriber segmented_point_cloud_sub;

    ros::Publisher stair_pub;
    ros::Publisher viz_pub;	

    StairParameterExtractor(ros::NodeHandle node) {
    	segmented_point_cloud_sub = node.subscribe("segmented_pointcloud",1, &StairParameterExtractor::segmentedCloudCallback, this);
    	stair_pub = node.advertise<lidar_reading_package::Stairs>("stairs", 1);
	viz_pub = node.advertise<sensor_msgs::PointCloud2>("stair_point_cloud", 1);
    
    }

	void segmentedCloudCallback(const sensor_msgs::PointCloud2& segmented_pointcloud){
  		pcl::PointCloud<pcl::PointXYZ>::Ptr recpc (new pcl::PointCloud<pcl::PointXYZ> ());
	    pcl::fromROSMsg(segmented_pointcloud, *recpc);

	    pcl::PointXYZ min, max;
	    pcl::getMinMax3D(*recpc, min, max);

	    std::cerr << "Min " << min.y << " Max " << max.y << std::endl;

	    if (max.y - min.y < 2) {
	    	std::cerr << "Not enough lines to extract stairs" << std::endl;
	    	return;
	    }

	    std::cerr << "Extracting stairs" << std::endl;
  		lidar_reading_package::Stairs stairs_msg;

		pcl::PointCloud<pcl::PointXYZ>::Ptr approach (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr up (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr depth (new pcl::PointCloud<pcl::PointXYZ> ());
		
		pcl::ModelCoefficients::Ptr approach_coefficients (new pcl::ModelCoefficients);
		pcl::ModelCoefficients::Ptr up_coefficients (new pcl::ModelCoefficients);
		pcl::ModelCoefficients::Ptr depth_coefficients (new pcl::ModelCoefficients);


		pcl::PointCloud<pcl::PointXYZ> stair_pc;
    
	    for (int i = min.y+1; i <= max.y-1; i++) {

  			lidar_reading_package::Stair s;

  			// select only the points in the specific y slice
  			get_line_coefficient(recpc, approach, approach_coefficients, i-1);
  			get_line_coefficient(recpc, up, up_coefficients, i);
  			get_line_coefficient(recpc, depth, depth_coefficients, i+1);
  			
   			// check if the following conditions are met:
   				// 1) the appraoch and up lines are perpendicular or almost (within 5 degrees of it)
  				// 2) the up and depth lines are perpendicular or almost (within 5 degrees of it)
  				// 3) the approach and depth lines are parrallel or almost (within 5 degrees of it)
			if(!angle_between(approach_coefficients, up_coefficients, 92.5, 87.5) 
				|| !angle_between(up_coefficients, depth_coefficients, 92.5, 87.5)
				|| !angle_between(approach_coefficients, depth_coefficients, 2.5, -2.5)){
				continue;
			}
		    	stair_pc += *approach;
		    	stair_pc += *up;
		        stair_pc += *depth;

   			pcl::PointXYZ minApproach, maxApproach;
   			pcl::getMinMax3D(*approach, minApproach, maxApproach);

   			std::cerr << " dist " << maxApproach.x << " " << minApproach.x << std::endl;
   			s.distance_to_stair = maxApproach.x - minApproach.x; 

   			pcl::PointXYZ minUp, maxUp;
   			pcl::getMinMax3D(*up, minUp, maxUp);
   			std::cerr << " up " << maxUp.z << " " << minUp.z << std::endl;
   			s.height = maxUp.z - minUp.z;


   			pcl::PointXYZ minDepth, maxDepth;
   			pcl::getMinMax3D(*depth, minDepth, maxDepth);
   			std::cerr << " depth " << maxDepth.x << " " << minDepth.x << std::endl;
   			s.depth = maxDepth.x - minDepth.x;

   			std::cerr << "Distance " << s.distance_to_stair << " Height " << s.height << " depth " << s.depth << std::endl;
   			stairs_msg.stairs.push_back(s);
   			break;
       	}
       	if(stair_pc.size() > 0) {
	  		std::cerr << "Publishing message" << std::endl;
			stair_pc.header.frame_id = "cloud";
			for (int i = 0; i < stair_pc.size(); i++) {
				stair_pc[i].y=0;
			}
			sensor_msgs::PointCloud2 stair_pc_msg; 
			pcl::toROSMsg(stair_pc, stair_pc_msg);
	  		stair_pub.publish(stairs_msg);
			viz_pub.publish(stair_pc_msg);
		}
  	}

  	void get_line_coefficient(pcl::PointCloud<pcl::PointXYZ>::Ptr src_point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dst_point_cloud, pcl::ModelCoefficients::Ptr coefficients, int i) {
	   	pcl::CropBox<pcl::PointXYZ> segmentFilter;
		segmentFilter.setInputCloud(src_point_cloud);
		segmentFilter.setMin(Eigen::Vector4f(-10000, i,-10000, 1.0));
		segmentFilter.setMax(Eigen::Vector4f(10000, i,10000, 1.0));
		segmentFilter.filter(*dst_point_cloud);
	
		// re-run the segmentation to fit a line to those points
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_LINE); 
		seg.setDistanceThreshold (0.01);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
 		std::cerr << "Input size " << dst_point_cloud->size() << std::endl;
		seg.setInputCloud (dst_point_cloud);
		seg.segment (*inliers, *coefficients);
  	}

  	bool angle_between(pcl::ModelCoefficients::Ptr line1, pcl::ModelCoefficients::Ptr line2, int max, int min) {

		float l1x = line1->values[3];
		float l1y = line1->values[4];
		float l1z = line1->values[5];

		float l2x = line2->values[3];
		float l2y = line2->values[4];
		float l2z = line2->values[5];

		double angle = pcl::getAngle3D(Eigen::Vector4f(l1x, l1y, l1z, 0.0), Eigen::Vector4f(l2x,l2y,l2z, 0.0));
  	
  		std::cerr << "angles " << pcl::rad2deg(angle) << std::endl;
  		std::cerr << "l1 " << l1x << " " << l1y << " " << l1z << " " << std::endl;
  	
		std::cerr << "l2 " << l2x << " " << l2y << " " << l2z << " " << std::endl;
  		if (pcl::rad2deg(angle) <= max && pcl::rad2deg(angle) >= min) {
			return true;
		} else {
			return false;
		}
  	}

};
