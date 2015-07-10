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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

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
		PointCloudT::Ptr cloud_plane (new PointCloudT);
		PointCloudT::Ptr image_cloud_filtered (new PointCloudT);
		
		//convert the msg to PCL format
		pcl::fromROSMsg (*msg, *image_cloud);
		
		//get the start time of recording
		double begin = ros::Time::now().toSec();
		string startTime = boost::lexical_cast<std::string>(begin);
		
		// append start timestamp with filenames
		std::stringstream convert;
		convert << pcd_count;
		std::string filename = generalDepthImageName+convert.str()+"_"+startTime+".pcd";
		
		//Before saving, do a z-filter
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud (image_cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 1.15);
		pass.filter (*image_cloud);
		
		//Place fitting
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		// Create the segmentation object
		pcl::SACSegmentation<PointT> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.01);
		
		// Create the filtering object
		pcl::ExtractIndices<PointT> extract;
		
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (image_cloud);
		seg.segment (*inliers, *coefficients);
		
		// Extract the plane
		extract.setInputCloud (image_cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_plane);
		
		//extract everything else
		extract.setNegative (true);
		extract.filter (*image_cloud_filtered);
			
		//Save the cloud to a .pcd file
		pcl::io::savePCDFileASCII(filename, *image_cloud_filtered);
		ROS_INFO("Saved pcd file %s", filename.c_str());
		
		/*std::cerr << "Start Cloud Viewer..." << std::endl;
		std::cerr << "z-filtered point cloud" << std::endl;
		pcl::visualization::CloudViewer viewer2 ("plane segmentation");
		viewer2.showCloud (image_cloud_filtered);
		while (!viewer2.wasStopped ());
			return;*/
		
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

