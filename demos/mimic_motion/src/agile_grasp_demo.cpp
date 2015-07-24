#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

// PCL specific includes
//#include <pcl/conversions.h>
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

#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>

#define PI 3.14159265

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;
PointCloudT::Ptr cloud_plane (new PointCloudT);

using namespace std;

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS 8 //6+2 for the arm

sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;

ros::Publisher pub_velocity;
ros::Publisher cloud_pub;
ros::Publisher cloud_grasp_pub;
 
sensor_msgs::PointCloud2 cloud_ros;
 
//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
	}
  //ROS_INFO_STREAM(current_state);
}


//Joint state cb
void joint_effort_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_effort = *input;
  //ROS_INFO_STREAM(current_effort);
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  //  ROS_INFO_STREAM(current_pose);
}

//Joint state cb
void fingers_cb (const jaco_msgs::FingerPosition msg) {
  current_finger = msg;
}


void movePose(float d_z) {
  actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/mico_arm_driver/arm_pose/arm_pose", true);

  jaco_msgs::ArmPoseGoal goalPose;

  // Set goal pose coordinates

  goalPose.pose.header.frame_id = "mico_api_origin";
  
  ROS_INFO_STREAM(current_pose);

  goalPose.pose.pose.position.x = current_pose.pose.position.x;
  goalPose.pose.pose.position.y = current_pose.pose.position.y;
  goalPose.pose.pose.position.z = current_pose.pose.position.z + d_z;
  goalPose.pose.pose.orientation.x = current_pose.pose.orientation.x;
  goalPose.pose.pose.orientation.y = current_pose.pose.orientation.y;
  goalPose.pose.pose.orientation.z = current_pose.pose.orientation.z;
  goalPose.pose.pose.orientation.w = current_pose.pose.orientation.w;

  ROS_INFO_STREAM(goalPose);

  ac.waitForServer();
  ROS_INFO("Waiting for server.");
  //finally, send goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goalPose);
  ac.waitForResult();

}

// Range = [6, 7300] ([open, close])
void moveFinger(int finger_value) {
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac("/mico_arm_driver/fingers/finger_positions", true);

    jaco_msgs::SetFingersPositionGoal goalFinger;

    goalFinger.fingers.finger1 = finger_value;
    goalFinger.fingers.finger2 = finger_value;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;
    
    ac.waitForServer();

    ac.sendGoal(goalFinger);

    ac.waitForResult();
}

int selectObjectToGrasp(std::vector<PointCloudT::Ptr > candidates){
	//currently, we just pick the one with the most points
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

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "agile_grasp_demo");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);

	//create subscriber to joint torques
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	  
	//publish velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	//debugging publisher
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("table_object_detection_node/cloud", 10);
	cloud_grasp_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
	
	
	
	//step 1: query table_object_detection_node to segment the blobs on the table
	ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
	segbot_arm_perception::TabletopPerception srv; //the srv request is just empty
	if (client_tabletop_perception.call(srv))
	{
		ROS_INFO("[agile_grasp_demo.cpp] Received Response from tabletop_object_detection_service");
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
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
		ROS_WARN("[agile_grasp_demo.cpp] No objects detected...aborting.");
		return 1;
	}
	
	//step 3: select which object to grasp
	int selected_object = selectObjectToGrasp(detected_objects);
	
	//publish object to find grasp topic
	pcl::PCLPointCloud2 pc_target;
	pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
	pcl_conversions::fromPCL(pc_target,cloud_ros);
	
	cloud_grasp_pub.publish(cloud_ros);

	return 0;
}
