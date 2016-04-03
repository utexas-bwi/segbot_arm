/*
 * obstacle_cloud_manager.cpp
 * 
 * A node that implements a server which published
 * point clouds to the obstacle cloud topic
 * 
 * author: Jivko Sinapov
 */ 

#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include "segbot_arm_perception/SetObstacles.h"


#define OBSTACLE_CLOUD_TOPIC "/cloud_arm_obstacles"
#define DEFAULT_FRAME_ID "/map"

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


//the publisher for the obstacle cloud
ros::Publisher cloud_pub;


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


bool set_obstacles_cb(segbot_arm_perception::SetObstacles::Request &req, segbot_arm_perception::SetObstacles::Response &res)
{
	
	PointCloudT::Ptr combined_cloud (new PointCloudT);
	
	std::string output_frame_id;
	
	for (unsigned int i = 0; i < req.clouds.size(); i ++){
		
		//convert to PCL format
		PointCloudT::Ptr cloud_i (new PointCloudT);
		pcl::fromROSMsg (req.clouds[i], *cloud_i);
		
		std::string frame_id_i = req.clouds[i].header.frame_id; 
		if (i == 0){
			output_frame_id = frame_id_i;
		}
		else if (frame_id_i != output_frame_id){
			ROS_ERROR("[obstacle_cloud_manager.cpp] Array of input clouds are not all in the same frame of reference!");
			return false;
		}
		
		*combined_cloud+=*cloud_i;
	}
	
	//check if the input vector is empty, in which case we publish an empty point cloud
	if (req.clouds.size() == 0){
		output_frame_id = DEFAULT_FRAME_ID;
	}
	
	//publish output 
	int num_msgs = 20;
	double frame_rate = 10.0;
	double duration = 1.0;
	ros::Rate r(frame_rate);
	
	//convert output cloud to ROS format
	sensor_msgs::PointCloud2 output_cloud_ros;
	pcl::toROSMsg(*combined_cloud, output_cloud_ros);
	output_cloud_ros.header.stamp = ros::Time::now();
	output_cloud_ros.header.frame_id = output_frame_id;
	
	double elapsed_time = 0;
	while (elapsed_time < duration){
		
		cloud_pub.publish(output_cloud_ros);
		
		r.sleep();
		
		elapsed_time += 1.0/(double)frame_rate;
	}
	
	
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "obstacle_cloud_manager");
	ros::NodeHandle nh;

	//obstacle cloud publisher
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(OBSTACLE_CLOUD_TOPIC, 1);

	//service
	ros::ServiceServer service = nh.advertiseService("set_obstacles", set_obstacles_cb);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	ros::spin();
	
	//refresh rate
	/*double ros_rate = 5.0;
	ros::Rate r(ros_rate);

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();

		r.sleep();

	}*/
};