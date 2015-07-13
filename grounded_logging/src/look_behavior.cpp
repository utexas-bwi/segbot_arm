#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "grounded_logging/StorePointCloud.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <boost/lexical_cast.hpp>

using namespace std;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// General point cloud to store the whole image
PointCloudT::Ptr image_cloud (new PointCloudT);

//z-filter
pcl::PassThrough<PointT> pass;

int pcd_count = 0;

bool recording_samples;
//string to store the filename
string pointCloudFileName;

void collect_point_cloud_data(const sensor_msgs::PointCloud2ConstPtr& msg){
	if(recording_samples == true && pcd_count == 0){
		if ((msg->width * msg->height) == 0)
				return;
				
		//convert the msg to PCL format
		pcl::fromROSMsg (*msg, *image_cloud);
			
		//get the start time of recording
		double begin = ros::Time::now().toSec();
		string startTime = boost::lexical_cast<std::string>(begin);
			
		// append start timestamp with filenames
		std::string filename = pointCloudFileName+"_"+startTime+".pcd";
			
		//Before saving, do a z-filter	
		pass.setInputCloud (image_cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 2.15);
		pass.filter (*image_cloud);
		
		//Save the cloud to a .pcd file
		pcl::io::savePCDFileASCII(filename, *image_cloud);
		ROS_INFO("Saved pcd file %s", filename.c_str());
		
		pcd_count++;
	}
	if(pcd_count == 1){
		return;
	}
}

//callback funtion
bool point_cloud_service_callback(grounded_logging::StorePointCloud::Request &req, 
							      grounded_logging::StorePointCloud::Response &res){
	if (req.start == 1){
		recording_samples = true;
		pointCloudFileName = req.pointCloudFilePath;
	}
	
	else{
		//set a flag to stop recording
		recording_samples = false;
		pcd_count = 0;
	}
	
	res.success = true;
	return true;				
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "point_cloud_logging_server");
	ros::NodeHandle nh;
	
	//Set up the service
	ros::ServiceServer service = nh.advertiseService("point_cloud_logger_service", point_cloud_service_callback);
	
	//subscribe to the vision depth topic
	ros::Subscriber sub_depth = nh.subscribe ("/xtion_camera/depth_registered/points", 1, collect_point_cloud_data);
		
	ROS_INFO("Ready to record and process point cloud data.");
	ros::spin();
	
	return 0;
}



