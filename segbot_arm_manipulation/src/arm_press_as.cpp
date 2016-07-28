//includes
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

#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>

//get table scene and color histogram
#include "segbot_arm_perception/TabletopPerception.h"
#include <segbot_arm_perception/segbot_arm_perception.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include <segbot_arm_manipulation/PressAction.h>
#include <segbot_arm_manipulation/arm_utils.h>

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

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/GetPositionIK.h>
//defines
#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

class PressActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::PressAction> as_; 
  
  std::string action_name_;
  
  segbot_arm_manipulation::PressFeedback feedback_;
  segbot_arm_manipulation::PressResult result_;
  
  ros::Publisher arm_vel;
  
  ros::Subscriber sub_angles;
  ros::Subscriber sub_torques;
  ros::Subscriber sub_tool;
  ros::Subscriber sub_finger;
  ros::Subscriber sub_wrench;
  
  ros::Publisher debug_pub;
  
  sensor_msgs::JointState current_state;
  sensor_msgs::JointState current_effort;
  
  sensor_msgs::JointState home_state;
  
  jaco_msgs::FingerPosition current_finger;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::WrenchStamped current_wrench; 
  
  bool heardPose;
  bool heardJoinstState;
  bool heardWrench;
  
  tf::TransformListener listener;

 
public:

  PressActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PressActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	heardPose = false;
	heardJoinstState = false;
	heardWrench = false;
	
	//publisher to move arm down
	arm_vel= nh_.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/joint_states", 1, &PressActionServer::joint_state_cb, this);

	//create subscriber to joint torques
	sub_torques = nh_.subscribe ("/mico_arm_driver/out/joint_efforts", 1, &PressActionServer::joint_effort_cb,this);

	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/mico_arm_driver/out/tool_position", 1, &PressActionServer::toolpos_cb, this);

	//subscriber for fingers
	sub_finger = nh_.subscribe("/mico_arm_driver/out/finger_position", 1, &PressActionServer::fingers_cb, this);
	  
	//subscriber for wrench
	sub_wrench = nh_.subscribe("/mico_arm_driver/out/tool_wrench", 1, &PressActionServer::wrench_cb, this);
	
	//advertise the goal pose for debugging
	debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mico_arm_driver/in/debug_pose", 2);
	
	ROS_INFO("Press action has started");
	
    as_.start();
  }

  ~PressActionServer(void)
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
	  //ROS_INFO_STREAM(current_effort);
	}

	//tool position cb
	void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
	  current_pose = msg;
	  heardPose = true;
	  //  ROS_INFO_STREAM(current_pose);
	}

	//fingers state cb
	void fingers_cb (const jaco_msgs::FingerPosition msg) {
	  current_finger = msg;
	}

	//Callback for toolwrench 
	void wrench_cb(const geometry_msgs::WrenchStamped &msg){ 
		current_wrench = msg;
		heardWrench = true;
	}
		
	void listenForArmData(float rate){
		heardPose = false;
		heardJoinstState = false;
		heardWrench = false;
		ros::Rate r(rate);
		
		while (ros::ok()){
			ros::spinOnce();
			
			if (heardPose && heardJoinstState && heardWrench)
				return;
			
			r.sleep();
		}
	}	
	
	geometry_msgs::Point find_top_center(PointCloudT pcl_curr){
		//find max of all points
		PointT max;
		PointT min; 
		pcl::getMinMax3D(pcl_curr, min, max);

		//find average of all points
		Eigen::Vector4f centroid_pts;
		pcl::compute3DCentroid(pcl_curr, centroid_pts);
		
		geometry_msgs::Point top_center;
		top_center.x = centroid_pts(0);
		top_center.y = centroid_pts(1);
		top_center.z = max.z;
		return top_center;
	}

	void press_down(float duration){
		geometry_msgs::TwistStamped v;
		ROS_INFO("inside press down");
		 
		v.twist.linear.x = 0;
		v.twist.linear.y = 0.0;
		v.twist.linear.z = -0.125;
		
		v.twist.angular.x = 0.0;
		v.twist.angular.y = 0.0;
		v.twist.angular.z = 0.0;
		
		float rate = 100;
		ros::Rate r(rate);
		listenForArmData(30.0);
		for(int i = 0; i< (int) rate * duration; i++){			
			v.twist.linear.x = 0;
			v.twist.linear.y = 0.0;
			v.twist.linear.z = -0.125;
			
			v.twist.angular.x = 0.0;
			v.twist.angular.y = 0.0;
			v.twist.angular.z = 0.0;
			
			arm_vel.publish(v);
			r.sleep();
			ros::spinOnce();
		}
		v.twist.linear.z = 0.0;
		arm_vel.publish(v);
	}
	



	std::vector<geometry_msgs::Quaternion> find_quat(geometry_msgs::PoseStamped goal_pose){
		float change = 0.0;
		float semi_circle = 3.14/4;
		
		listener.waitForTransform(goal_pose.header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
		
		std::vector<geometry_msgs::Quaternion> possible_quats;		
		while(change < semi_circle){
			geometry_msgs::Quaternion quat1= tf::createQuaternionMsgFromRollPitchYaw(-3.14/2, - change ,0);
			geometry_msgs::Quaternion quat2 = tf::createQuaternionMsgFromRollPitchYaw(3.14/2, - change ,0);
			
			goal_pose.pose.orientation = quat1;
			moveit_msgs::GetPositionIK::Response  ik_response_1 = segbot_arm_manipulation::computeIK(nh_,goal_pose);
			if (ik_response_1.error_code.val == 1){
				possible_quats.push_back(goal_pose.pose.orientation);
			}
			
			goal_pose.pose.orientation = quat2; 
			moveit_msgs::GetPositionIK::Response  ik_response_2 = segbot_arm_manipulation::computeIK(nh_,goal_pose);
			if (ik_response_2.error_code.val == 1){
				possible_quats.push_back(goal_pose.pose.orientation);
			}
			
			change += 3.14/16;
		}
		
		return possible_quats;
		
	}

	void executeCB(const segbot_arm_manipulation::PressGoalConstPtr  &goal){
		listenForArmData(30.0);
		
		if(goal -> tgt_cloud.data.size() == 0){
			result_.success = false;
			ROS_INFO("[arm_press_as.cpp] No object point clouds received...aborting");
			as_.setAborted(result_);
			return;
		}
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Press action: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
        
		ROS_INFO("checked that the goal is not preempted and tgt cloud is valid");
		//step 1: close fingers
		segbot_arm_manipulation::closeHand();
	
		//step 2: transform into the arm's base, transform cloud
		sensor_msgs::PointCloud2 tgt= goal -> tgt_cloud;
		
		std::string sensor_frame_id = tgt.header.frame_id;
			
	
		listener.waitForTransform(sensor_frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		
		sensor_msgs::PointCloud transformed_pc;
		sensor_msgs::convertPointCloud2ToPointCloud(tgt,transformed_pc);
		
		listener.transformPointCloud("mico_link_base", transformed_pc ,transformed_pc); 
		sensor_msgs::convertPointCloudToPointCloud2(transformed_pc, tgt);
		
		PointCloudT pcl_cloud;
		pcl::fromROSMsg(tgt, pcl_cloud);
		
		
		
		//step 3: find the top of the object, set goal to slightly above object
		geometry_msgs::Point top = find_top_center(pcl_cloud);
		geometry_msgs::PoseStamped goal_pose;
		
		goal_pose.header.frame_id = tgt.header.frame_id;
		goal_pose.pose.position = top; 
		goal_pose.pose.position.z += 0.125; 
		
		//step 4: find the hand orientation
		std::vector<geometry_msgs::Quaternion> possible_quats = find_quat(goal_pose);

		if(possible_quats.size() == 0 ){
			result_.success = false;
			ROS_INFO("[arm_press_as.cpp] No possible poses...");
			as_.setAborted(result_);
			return;
		}
		
		//for now go to first possible pose
		goal_pose.pose.orientation = possible_quats.at(0);
		
		listenForArmData(30.0);
		
		debug_pub.publish(goal_pose);
		
		ROS_INFO_STREAM("frame for goal_pose");
		ROS_INFO_STREAM(goal_pose.header.frame_id);
		
		listenForArmData(30.0); 
		
		//step 5: move to goal position
		segbot_arm_manipulation::moveToPoseMoveIt(nh_,goal_pose);
		listenForArmData(30.0);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_,goal_pose);

		//step 6: press down on the object
		press_down(5);
		
		
		listenForArmData(30.0);
		
		segbot_arm_manipulation::homeArm(nh_);
		//step 7: move arm home		
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);
		listenForArmData(30.0);
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);

		
		result_.success = true;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_press_as");

  PressActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
