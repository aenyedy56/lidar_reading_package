#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "lidar_reading_package/Stair.h"
#include "lidar_reading_package/Stairs.h"
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class StairParameterExtractor {

public:

    ros::Subscriber segmented_point_cloud_sub;

    ros::Publisher stair_pub;

    StairParameterExtractor(ros::NodeHandle node) {
    	segmented_point_cloud_sub = node.subscribe("segmented_pointcloud",1, &StairParameterExtractor::segmentedCloudCallback, this);
    	stair_pub = node.advertise<lidar_reading_package::Stairs>("stairs", 1);
    
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

	    for (int i = min.y+1; i <= max.y-1; i++) {

  			lidar_reading_package::Stair s;

	    	pcl::PointCloud<pcl::PointXYZ>::Ptr approach (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr up (new pcl::PointCloud<pcl::PointXYZ> ());
   			pcl::PointCloud<pcl::PointXYZ>::Ptr depth (new pcl::PointCloud<pcl::PointXYZ> ());
   			
   			pcl::CropBox<pcl::PointXYZ> segmentFilter;
   			segmentFilter.setInputCloud(recpc);
   			segmentFilter.setMin(Eigen::Vector4f(-1000, i-1,-1000, 1.0));
			segmentFilter.setMax(Eigen::Vector4f(1000, i-1,1000, 1.0));
   			segmentFilter.filter(*approach);

   			segmentFilter.setMin(Eigen::Vector4f(-1000, i,-1000, 1.0));
			segmentFilter.setMax(Eigen::Vector4f(1000, i,1000, 1.0));
   			segmentFilter.filter(*up);

   			segmentFilter.setMin(Eigen::Vector4f(-1000, i+1,-1000, 1.0));
			segmentFilter.setMax(Eigen::Vector4f(1000, i+1,1000, 1.0));
   			segmentFilter.filter(*depth);

   			pcl::PointXYZ minApproach, maxApproach;
   			pcl::getMinMax3D(*approach, minApproach, maxApproach);
   			std::cerr << " dist " << maxApproach.x << " " << minApproach.x << std::endl;
   			s.distance = maxApproach.x - minApproach.x; 

   			pcl::PointXYZ minUp, maxUp;
   			pcl::getMinMax3D(*up, minUp, maxUp);
   			std::cerr << " up " << maxUp.z << " " << minUp.z << std::endl;
   			s.height = maxUp.z - minUp.z;


   			pcl::PointXYZ minDepth, maxDepth;
   			pcl::getMinMax3D(*depth, minDepth, maxDepth);
   			std::cerr << " depth " << maxDepth.x << " " << minDepth.x << std::endl;
   			s.depth = maxDepth.x - minDepth.x;

   			std::cerr << "Distance " << s.distance << " Height " << s.height << " depth " << s.depth << std::endl;
   			stairs_msg.stairs.push_back(s);
       	}
  		std::cerr << "Publishing message" << std::endl;

  		stair_pub.publish(stairs_msg);

  	}


};