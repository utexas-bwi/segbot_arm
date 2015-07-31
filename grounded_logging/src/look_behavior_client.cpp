#include <ros/ros.h>
#include <signal.h>
#include <math.h>
#include <cstdlib>
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

//Filepath to store the data
std::string generalFilePath = "/home/bwi/look_behaviour";

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;

sensor_msgs::PointCloud2 cloud_ros;
ros::Publisher object_cloud_pub;

//currently, we just pick the one with the most points
int selectObjectToGrasp(std::vector<PointCloudT::Ptr > candidates){
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
	
	//step 2: extract the data from the response
	detected_objects.clear();
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
	
	//step 3: select which object to grasp
	int selected_object = selectObjectToGrasp(detected_objects);
	
	//publish object to find grasp topic
	pcl::PCLPointCloud2 pc_target;
	pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
	pcl_conversions::fromPCL(pc_target,cloud_ros);
	
	//publish to extract_object
	ROS_INFO("Publishing object point cloud...");
	object_cloud_pub.publish(cloud_ros);
	
	return(0);
}
