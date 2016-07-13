#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>

//get table scene and color histogram
#include "segbot_arm_perception/TabletopPerception.h"
#include <segbot_arm_perception/segbot_arm_perception.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/arm_utils.h>

#include <segbot_arm_manipulation/PressAction.h>
#include <segbot_arm_manipulation/LiftVerifyAction.h>

//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


ros::Publisher vis_pub;
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;

bool heardJoinstState;
bool heardPose;

using namespace std;

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
	}
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardJoinstState && heardPose)
			return;
		
		r.sleep();
	}
}

int largest_obj(segbot_arm_perception::TabletopPerception::Response table_scene){
	int largest_pc_index = -1;
	int largest_num_points = -1;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		
		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
	
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
}

int chose_object(std::string message, segbot_arm_perception::TabletopPerception::Response table_scene){
	std::cout << message;
	while (true){
		char c = std::cin.get();
		if (c == '\n') {
			return largest_obj(table_scene);
		} else if (c >= '0' && c <= '9') {
			return c - '0';
		} else {
			std::cout <<  message;
		}
	}
}


//TO DO: make sure locations are accurate 
void show_indicies(segbot_arm_perception::TabletopPerception::Response table_scene){
	for(unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		//find the center of the point
		sensor_msgs::PointCloud2 ros_curr = table_scene.cloud_clusters[i];
		PointCloudT::Ptr pcl_curr;
		pcl::fromROSMsg(ros_curr, *pcl_curr);
		Eigen::Vector4f center;
		pcl::compute3DCentroid(*pcl_curr, center);
		
		//display a number in rviz for the index of every point cloud available
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		marker.text = "%d", i;
		
		marker.pose.position.x = center(0); //TO DO: check that you can see the text
		marker.pose.position.y = center(1);
		marker.pose.position.z = center(2);
	
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		//I think scales can be left alone 
		marker.scale.x = 1;
		marker.scale.y = 0.1;
		marker.scale.z = 1; //previously 0.1
		
		marker.color.a = 1.0; //alpha
		
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pub.publish(marker);
		
	}
}

sensor_msgs::JointState set_home_arm(){
	sensor_msgs::JointState arm_home;
	arm_home.position.clear();
	arm_home.position.push_back(-1.3417218624707292);
	arm_home.position.push_back(-0.44756153173493096);
	arm_home.position.push_back(-0.2887493796082798);
	arm_home.position.push_back(-1.1031276625138604);
	arm_home.position.push_back(1.1542971070664283);
	arm_home.position.push_back(2.9511931472480804);
	//arm_home.position.push_back(current_finger.finger1);
	//arm_home.position.push_back(current_finger.finger2);
}

int main (int argc, char** argv){
	ros::init(argc, argv, "demo_explore_object");
	
	ros::NodeHandle n;
	
	heardPose = false;
	heardJoinstState = false;
	
	sensor_msgs::JointState arm_home = set_home_arm();
	
	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	//create a publisher for rviz markers
	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

	listenForArmData();
	
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
	
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
	} 
	
	show_indicies(table_scene);
	
	int index = chose_object("Enter index of object or press enter to pick largest", table_scene);
	
	//create the action client
	actionlib::SimpleActionClient<segbot_arm_manipulation::PressAction> press_ac("arm_press_as",true);
	press_ac.waitForServer();
	ROS_INFO("press action server made...");
	
	segbot_arm_manipulation::PressGoal press_goal;
	
	press_goal.tgt_cloud = table_scene.cloud_clusters[index];
	press_goal.arm_home = arm_home; 
	
	//send the goal
	ROS_INFO("Sending goal to action server...");
	press_ac.sendGoal(press_goal);
		
	//block until the action is completed
	ROS_INFO("Waiting for result...");
	press_ac.waitForResult();
	ROS_INFO("Press action Finished...");
	
	
	
	return 0;
}
