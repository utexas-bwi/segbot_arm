#include <ros/ros.h>
#include <signal.h>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <signal.h> 
#include <std_msgs/String.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//actions
#include <actionlib/client/simple_action_client.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

bool g_caught_sigint = false;

// counter to keep track of the objects
int obj_num = 1;

//Filepath to store the data
//Change the trial folder name each time --> Not very sophisticated but works for now
std::string pointCloudFilePath = "/home/bwi/look_behaviour/trial_1";

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;

sensor_msgs::PointCloud2 cloud_ros;
ros::Publisher object_cloud_pub;

// function to handle Ctrl-C
void sig_handler(int sig)
{
	g_caught_sigint = true;
	exit (0);
};

//currently, we just pick the one with the most points
int selectObjectToSave(std::vector<PointCloudT::Ptr > candidates){
	int max_num_points = -1;
	int index = -1;
	
	for (unsigned int i = 0; i < candidates.size(); i ++){
		if ((int)candidates.at(i)->points.size() > max_num_points){
			max_num_points = (int)candidates.at(i)->points.size();
			index = (int)i;
		}
	}
	
	return index;
}

int main(int argc, char **argv)
{
	//Initialize ROS
	ros::init (argc, argv, "look_behaviour_client");
	ros::NodeHandle nh;
	
	//debugging publisher
	object_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("look_behaviour_client/object_cloud", 10);
		
	//step 1: query table_object_detection_node to segment the blobs on the table
	ros::ServiceClient client_tabletop_perception = nh.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
	while (ros::ok()){
		segbot_arm_perception::TabletopPerception srv; //the srv request is just empty
		if (client_tabletop_perception.call(srv))
		{
			ROS_INFO("Received response from tabletop_object_detection_service");
		}
		else
		{
			ROS_ERROR("Failed to call tabletop_object_detection_service");
			return 1;
		}
		
		// clear out the vector of detected objects
		detected_objects.clear();
		
		//step 2: extract the data from the response
		for (unsigned int i = 0; i < srv.response.cloud_clusters.size(); i++){
			PointCloudT::Ptr object_i (new PointCloudT);
			pcl::PCLPointCloud2 pc_i;
			pcl_conversions::toPCL(srv.response.cloud_clusters.at(i),pc_i);
			pcl::fromPCLPointCloud2(pc_i,*object_i);
			detected_objects.push_back(object_i);
		}
		
		if (detected_objects.size() == 0){
			ROS_WARN("No objects detected...aborting.");
			return 1;
		}
		
		Eigen::Vector4f plane_coef_vector;
		for (int i = 0; i < 4; i ++)
			plane_coef_vector(i)=srv.response.cloud_plane_coef[i];
		
		//step 3: select which object to display
		int selected_object = selectObjectToSave(detected_objects);
		
		//publish object to find topic
		pcl::PCLPointCloud2 pc_target;
		pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
		pcl_conversions::fromPCL(pc_target,cloud_ros);
		
		//publish to extract_object
		ROS_INFO("Publishing object point cloud...");

		ros::spinOnce();
		object_cloud_pub.publish(cloud_ros);
			
		// Ask the user if he wants to save the point cloud
		cout << "Do you want to save this point cloud? (y/n)";
		if (cin.get() == 'y'){
			//get the start time of recording
			double begin = ros::Time::now().toSec();
			string startTime = boost::lexical_cast<std::string>(begin);
				
			std::stringstream convert;
			convert << obj_num;	
				
			// append start timestamp with filenames
			string pointCloudFileName = pointCloudFilePath+"/test"+convert.str()+"_"+startTime+".pcd";
					
			//Save the cloud to a .pcd file
			pcl::io::savePCDFileASCII(pointCloudFileName, *detected_objects.at(selected_object));
			ROS_INFO("Saved pcd file %s", pointCloudFileName.c_str());
			obj_num++;
		}
		if(cin.get() == 'n')
			continue;
	}
	
	return(0);
}
