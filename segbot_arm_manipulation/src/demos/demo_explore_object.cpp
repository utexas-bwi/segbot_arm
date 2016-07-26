#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/point_cloud_conversion.h>

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
#include <segbot_arm_manipulation/PushAction.h>
#include <segbot_arm_manipulation/TabletopGraspAction.h>
#include <segbot_arm_manipulation/LiftVerifyAction.h>
#include <segbot_arm_manipulation/ShakeAction.h>

//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>


#include <tf/transform_listener.h>
#include <tf/tf.h>

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

tf::TransformListener listener;

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
		//first transform into cloud into the arm's space
		sensor_msgs::PointCloud2 tgt = table_scene.cloud_clusters[i];
		
		std::string sensor_frame_id = tgt.header.frame_id;
		listener.waitForTransform(sensor_frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		
		
		sensor_msgs::PointCloud tran_pc;
		sensor_msgs::convertPointCloud2ToPointCloud(tgt,tran_pc);
		
		listener.transformPointCloud("mico_link_base", tran_pc ,tran_pc); 
		sensor_msgs::convertPointCloudToPointCloud2(tran_pc, tgt);
		
		PointCloudT pcl_curr;
		pcl::fromROSMsg(tgt, pcl_curr);
		
		
		Eigen::Vector4f center;
		pcl::compute3DCentroid(pcl_curr, center);
		
		//display a number in rviz for the index of every point cloud available
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		
		std::stringstream sstm;
		sstm << i;
		marker.text = sstm.str();
		
		marker.pose.position.x = center(0); 
		marker.pose.position.y = center(1);
		marker.pose.position.z = center(2);
	
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1; 
		
		marker.color.a = 1.0; //alpha
		
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pub.publish(marker);
		
	}
}

sensor_msgs::JointState set_home_arm(){
	sensor_msgs::JointState arm_home;
	arm_home.position.push_back(-1.3417218624707292);
	arm_home.position.push_back(-0.44756153173493096);
	arm_home.position.push_back(-0.2887493796082798);
	arm_home.position.push_back(-1.1031276625138604);
	arm_home.position.push_back(1.1542971070664283);
	arm_home.position.push_back(2.9511931472480804);
	arm_home.position.push_back(FINGER_FULLY_CLOSED);
	arm_home.position.push_back(FINGER_FULLY_CLOSED);
	return arm_home;
}

void pressEnter(std::string message){
	std::cout << message;
	while (true){
		char c = std::cin.get();
		if (c == '\n')
			break;
		else if (c == 'q'){
			ros::shutdown();
			exit(1);
		}
		else {
			std::cout <<  message;
		}
	}
}

int main (int argc, char** argv){
	ros::init(argc, argv, "demo_explore_object");
	
	ros::NodeHandle n;
	
	heardPose = false;
	heardJoinstState = false;
	
	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	//create a publisher for rviz markers
	vis_pub = n.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );

	listenForArmData();
	
	pressEnter("Press enter to get table scene or q to quit");
	
	//get table scene and find all objects on table 
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
	
	if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_WARN("No objects found on table. The end...");
			exit(1);
	} 
	
	pressEnter("Press enter to show indices or q to quit");
	show_indicies(table_scene);
	
	int index = chose_object("Enter index of object or press enter to pick largest ", table_scene);
	ROS_INFO("index chosen is: ");
	ROS_INFO_STREAM(index);
	
	sensor_msgs::JointState arm_home = set_home_arm();
	ROS_INFO("made arm home to send");
	
	pressEnter("Press enter to start press action or q to quit");
	//create the action client to press object
	actionlib::SimpleActionClient<segbot_arm_manipulation::PressAction> press_ac("arm_press_as",true);
	press_ac.waitForServer();
	ROS_INFO("press action server made...");
	
	segbot_arm_manipulation::PressGoal press_goal;
	
	press_goal.tgt_cloud = table_scene.cloud_clusters[index];
	press_goal.arm_home = arm_home; 
	ROS_INFO("all goals initialized");
	
	//send the goal
	ROS_INFO("Sending goal to action server...");
	press_ac.sendGoal(press_goal);
		
	//block until the action is completed
	ROS_INFO("Waiting for result...");
	press_ac.waitForResult();
	ROS_INFO("Press action Finished...");
	
	listenForArmData();
	
	
	pressEnter("Press enter to start push action or q to quit");
	//create the action client to push an object
	actionlib::SimpleActionClient<segbot_arm_manipulation::PushAction> push_ac("arm_push_as",true);
	push_ac.waitForServer();
	ROS_INFO("push action server made....");
	
	segbot_arm_manipulation::PushGoal push_goal;
	
	push_goal.tgt_cloud = table_scene.cloud_clusters[index];
	push_goal.cloud_plane = table_scene.cloud_plane; 
	push_goal.arm_home = arm_home; 
	
	//send the goal
	ROS_INFO("Sending goal to action server...");
	push_ac.sendGoal(push_goal);
	
	ROS_INFO("Waiting for results");
	push_ac.waitForResult();
	ROS_INFO("Push action finished...");
	
	
	listenForArmData();
	
	pressEnter("Press enter to start grasp and verify action or q to quit");
	
	//create the action client to grasp
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> grasp_ac("segbot_tabletop_grasp_as",true);
	grasp_ac.waitForServer();
	ROS_INFO("action server made...");
		
	//create and fill goal
	segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
		
	//we want the robot to execute the GRASP action
	grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
	
	//finally, we fill in the table scene
	grasp_goal.cloud_plane = table_scene.cloud_plane;
	grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
	}
	grasp_goal.target_object_cluster_index = index;
			
	//send goal
	ROS_INFO("Sending goal to action server...");
	grasp_ac.sendGoal(grasp_goal);
	
	ROS_INFO("Waiting for result...");
	grasp_ac.waitForResult();
	
	ROS_INFO("Grasp action Finished...");


	listenForArmData();
	
	//create action to lift and verify
	actionlib::SimpleActionClient<segbot_arm_manipulation::LiftVerifyAction> lift_ac("arm_lift_verify_as", true);
	lift_ac.waitForServer();
	
	//make goals to send to action
	segbot_arm_manipulation::LiftVerifyGoal lift_verify_goal;
	
	lift_verify_goal.tgt_cloud = table_scene.cloud_clusters[index];
	lift_verify_goal.bins = 8;
	
	ROS_INFO("sending goal to lift and verify action server...");
	lift_ac.sendGoal(lift_verify_goal);
	
	ROS_INFO("waiting for lift and verify action server result....");
	lift_ac.waitForResult();
	
	ROS_INFO("lift and verify action finished.");
	segbot_arm_manipulation::LiftVerifyResult result = *lift_ac.getResult();
	
	bool verified = result.success;
	
	if(verified){
		ROS_INFO("Verification succeeded.");
	}else{
		ROS_WARN("Verification failed");
	}
	
	pressEnter("Press enter to start shake action or q to quit");
	
	listenForArmData();
	//create action to shake object
	actionlib::SimpleActionClient<segbot_arm_manipulation::ShakeAction> shake_ac("arm_shake_as", true);
	shake_ac.waitForServer();
	
	//make and send goals to action 
	segbot_arm_manipulation::ShakeGoal shake_goal;
	shake_goal.tgt_cloud = table_scene.cloud_clusters[index];
	shake_goal.cloud_plane = table_scene.cloud_plane;
	shake_goal.arm_home = arm_home;
	shake_goal.verified = verified; 
	
	ROS_INFO("sending goal to shake action server....");
	shake_ac.sendGoal(shake_goal);
	
	ROS_INFO("waiting for shake action server result....");
	shake_ac.waitForResult();
	
	ROS_INFO("shake action finished...");
	segbot_arm_manipulation::ShakeResult shake_result = *shake_ac.getResult();
	
	
	return 0;
}
