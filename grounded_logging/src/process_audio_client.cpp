/*  Run the file giving it a name of the file to save both 
    raw data (.wav) and dft (.txt) in and also the sampleDuration (int)   */  
// rosrun grounded_logging process_audio_client.cpp <filename.wav> <filename.txt> <sampleDuration>  

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "grounded_logging/ProcessAudio.h"

using namespace std;

int main (int argc, char** argv)
{
	//Initialize ROS
	ros::init (argc, argv, "audio_processing_client");
	if(argc != 4) 
	{
		ROS_INFO("Usage: audio_processing_client <filename1.wav> <filename2.txt> <sampleDuration>");
		return 1;
	}
	
	ros::NodeHandle nh;
	ros::service::waitForService("process_audio");
	ros::ServiceClient client = nh.serviceClient<grounded_logging::ProcessAudio>("process_audio");
	grounded_logging::ProcessAudio srv;
	srv.request.outputRawFileName = argv[1];
	srv.request.outputDftFileName = argv[2];
	srv.request.sampleDuration = atoll(argv[3]);
		
	if (client.call(srv))
	{
		ROS_INFO("Service process_audio called...");
	}
	else
	{
		ROS_ERROR("Failed to call service process_audio. Server might not have been launched yet.");
		return 1;
	}
	
	return 0;
}
