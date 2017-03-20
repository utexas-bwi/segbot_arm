#include <ros/ros.h>
#include <ros/package.h>

#include <cstdlib>

#include <geometry_msgs/TwistStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl_ros/impl/transforms.hpp>

//the action definition
#include "segbot_arm_manipulation/ObjReplacementAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ObjReplacementActionServer
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<segbot_arm_manipulation::ObjReplacementAction> as_; 
	std::string action_name_; 

	segbot_arm_manipulation::ObjReplacementFeedback feedback_; 
	segbot_arm_manipulation::ObjReplacementResult result_; 

	sensor_msgs::JointState current_state;
	sensor_msgs::JointState current_effort;
	jaco_msgs::FingerPosition current_finger;
	geometry_msgs::PoseStamped current_pose;
	bool heardPose;
	bool heardJoinstState; 

	bool heardGrasps;
	agile_grasp::Grasps current_grasps;

	//used to compute transforms
	tf::TransformListener listener;

	ros::Subscriber sub_angles;
	ros::Subscriber sub_torques;
	ros::Subscriber sub_tool;
	ros::Subscriber sub_finger;
	ros::Subscriber sub_grasps;

public:
	ObjReplacementActionServer(std::string name) :
		as_(nh_, name, boost::bind(&ObjReplacementActionServer::executeCB, this, _1), false),
    		action_name_(name)
    {
    	heardPose = false;
		heardJoinstState = false;
		heardGrasps = false;

		//create subscriber to joint angles
		sub_angles = nh_.subscribe ("/joint_states", 1, &ObjReplacementActionServer::joint_state_cb, this);

		//create subscriber to joint torques
		sub_torques = nh_.subscribe ("/mico_arm_driver/out/joint_efforts", 1, &ObjReplacementActionServer::joint_effort_cb,this);

		//create subscriber to tool position topic
		sub_tool = nh_.subscribe("/mico_arm_driver/out/tool_position", 1, &ObjReplacementActionServer::toolpos_cb, this);

		//subscriber for fingers
		sub_finger = nh_.subscribe("/mico_arm_driver/out/finger_position", 1, &ObjReplacementActionServer::fingers_cb, this);
	  
		//subscriber for grasps
		sub_grasps = nh_.subscribe("/find_grasps/grasps_handles",1, &ObjReplacementActionServer::grasps_cb,this);  

    	ROS_INFO("Starting replacement grasp action server..."); 
    	as_.start(); 
    }

	~ObjReplacementActionServer(void)
	{
	}

	//Joint state cb
	void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
		if (input->position.size() == NUM_JOINTS){
			current_state = *input;
			heardJoinstState = true;
		}
	}

	//Joint effort cb
	void joint_effort_cb (const sensor_msgs::JointStateConstPtr& input) {
		current_effort = *input;
	}

	//tool position cb
	void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
		current_pose = msg;
		heardPose = true;
	}

	//fingers state cb
	void fingers_cb (const jaco_msgs::FingerPosition msg) {
		current_finger = msg;
	}

	void grasps_cb(const agile_grasp::Grasps &msg){
		ROS_INFO("Heard grasps!");
		current_grasps = msg;
		heardGrasps = true;
	}
		
	void listenForArmData(float rate){
		heardPose = false;
		heardJoinstState = false;
		ros::Rate r(rate);
		
		while (ros::ok()){
			ros::spinOnce();
			
			if (heardPose && heardJoinstState)
				return;
			
			r.sleep();
		}
	}	
		
	void listenForGrasps(float rate){
		ros::Rate r(rate);
		heardGrasps = false;
		while (ros::ok()){
			ros::spinOnce();
			if (heardGrasps)
				return;
			r.sleep();
		}
	}

	void executeCB(const segbot_arm_manipulation::ObjReplacementGoalConstPtr  &goal)
	{
	}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_obj_replacement_as");

  ObjReplacementActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}