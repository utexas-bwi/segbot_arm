#include "ros/ros.h"
#include "grounded_logging/ProcessAudio.h"
#include "grounded_logging/ProcessVision.h"
#include "grounded_logging/StorePointCloud.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cstdlib>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	//Initialize ROS
	ros::init (argc, argv, "grounded_learning_client");
	if(argc != 5) 
	{
		ROS_INFO("Usage: grounded_learning_client <depthImageFilePath> <rgbImageFilePath> <wavFilePath> <dftFilePath> ");
		return 1;
	}
	
	ros::NodeHandle nh;
	ros::ServiceClient depth_client = nh.serviceClient<grounded_logging::StorePointCloud>("point_cloud_logger_service");
	ros::ServiceClient image_client = nh.serviceClient<grounded_logging::ProcessVision>("vision_logger_service");
	ros::ServiceClient audio_client = nh.serviceClient<grounded_logging::ProcessAudio>("audio_logger_service");
	
	grounded_logging::StorePointCloud depth_srv;
	grounded_logging::ProcessVision image_srv;
	grounded_logging::ProcessAudio audio_srv;
	
	depth_srv.request.start = 1;
	depth_srv.request.pointCloudFilePath = argv[1];
	image_srv.request.start = 1;
	image_srv.request.generalImageFilePath = argv[2];
	audio_srv.request.start = 1;
	audio_srv.request.outputRawFilePath = argv[3];
	audio_srv.request.outputDftFilePath = argv[4];
	
	if (depth_client.call(depth_srv))
	{
		ROS_INFO("Point_cloud_logger_service called...");
	}
	else
	{
		ROS_ERROR("Failed to call point_cloud_logger_service. Server might not have been launched yet.");
		return 1;
	}
	if (image_client.call(image_srv))
	{
		ROS_INFO("Vision_logger_service called...");
	}
	else
	{
		ROS_ERROR("Failed to call vision_logger_service. Server might not have been launched yet.");
		return 1;
	}
	if (audio_client.call(audio_srv))
	{
		ROS_INFO("Audio_logger_service called...");
	}
	else
	{
		ROS_ERROR("Failed to call audio_logger_service. Server might not have been launched yet.");
		return 1;
	}
	
}

