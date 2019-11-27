This package contains the code for two ROS Nodes (https://www.ros.org/). These nodes are used to process lidar sensor data in an atempt to find and locate stairs. 

The nodes are as follows:
1) lidar_listener - Takes the raw sensor data and rotates, filters then segments each point cloud message into planar line segments. 
2) stair_param_extractor  - takes the messages from the lidar_listener node and looks for lines matching the gemoetry of staris 

The nodes make some assumptions:
1) There is a lidar publishing on the 'cloud' or 'mock_cloud' ros topic 
2) The lidar produces a data array with 810 items (270 degrees * 3 measurements per degree) 


