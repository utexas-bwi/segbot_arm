#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <signal.h> 
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/image_encodings.h>
#include "grounded_logging/ProcessVision.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>

using namespace std;

bool g_caught_sigint = false;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// should start recording or not				
bool recording_samples;
string generalImageFileName;
string generalDepthImageName;    

int image_count = 0;
int pcd_count = 0;

// function to handle Ctrl-C
void sig_handler(int sig)
{
	g_caught_sigint = true;
	exit (0);
};

//callback funtion to store depth images
void collect_vision_depth_data(const sensor_msgs::PointCloud2ConstPtr& msg){
	if(recording_samples == true){
		 if ((msg->width * msg->height) == 0)
			return;
		// General point cloud to store the whole image
		PointCloudT::Ptr image_cloud (new PointCloudT);
		
		//convert the msg to PCL format
		pcl::fromROSMsg (*msg, *image_cloud);
		
		//get the start time of recording
		double begin = ros::Time::now().toSec();
		string startTime = boost::lexical_cast<std::string>(begin);
		
		// append start timestamp with filenames
		std::stringstream convert;
		convert << pcd_count;
		std::string filename = generalDepthImageName+convert.str()+"_"+startTime+".pcd";
		pcl::io::savePCDFileASCII(filename, *image_cloud);
		ROS_INFO("Saved pcd file %s", filename.c_str());
		
		pcd_count++;
	}
}

//callback funtion to store rgb images
void collect_vision_rgb_data(const sensor_msgs::ImageConstPtr& msg){
	if(recording_samples == true){
		cv_bridge::CvImagePtr cv_image;
		try{	
			cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		if (cv_image) {
			//get the start time of recording
			double begin = ros::Time::now().toSec();
			string startTime = boost::lexical_cast<std::string>(begin);
		
			// append start timestamp with filenames
			std::stringstream convert;
			convert << image_count;
			std::string filename = generalImageFileName+convert.str()+"_"+startTime+".jpg";
			cv::imwrite(filename.c_str(), cv_image->image);
			ROS_INFO("Saved image %s", filename.c_str());
					
			image_count++;
		}
	}
}

//callback funtion, if recording samples is true then make a call to store the visual feed 
bool vision_service_callback(grounded_logging::ProcessVision::Request &req, 
							grounded_logging::ProcessVision::Response &res){
	if (req.start == 1){
		//start recording
		recording_samples = true;
		
		//also store the filenames that are in the request
		generalImageFileName = req.generalImageFileName;
		generalDepthImageName = req.generalDepthImageName;       
	}
	else{
		//set a flag to stop recording
		recording_samples = false;
	}
	
	res.success = true;
	return true;
}
							
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "vision_logging_server");
	ros::NodeHandle nh;
	
	//Set up the service
	ros::ServiceServer service = nh.advertiseService("vision_logger_service", vision_service_callback);
	
	//subscribe to the vision depth topic
	ros::Subscriber sub_depth = nh.subscribe ("/camera/depth/points", 1000, collect_vision_depth_data);

	//subsribe to the vision rgb topic
	//ros::Subscriber sub_rgb = nh.subscribe ("/camera/rgb/image_color", 1000, collect_vision_rgb_data);
		
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	ROS_INFO("Ready to record and process vision data.");
	ros::spin();
	
	return 0;
}
