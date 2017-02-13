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
#include <segbot_arm_manipulation/ShakeAction.h>
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
//defines
#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ShakeActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::ShakeAction> as_; 
  
  std::string action_name_;
  
  //messages for publishing feedback and result of action
  segbot_arm_manipulation::ShakeFeedback feedback_;
  segbot_arm_manipulation::ShakeResult result_;
  
  ros::Publisher arm_vel;
  
  ros::Subscriber sub_angles;
  ros::Subscriber sub_torques;
  ros::Subscriber sub_tool;
  ros::Subscriber sub_finger;
  ros::Subscriber sub_wrench;
  
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

  ShakeActionServer(std::string name) :
    as_(nh_, name, boost::bind(&ShakeActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	heardPose = false;
	heardJoinstState = false;
	heardWrench = false;
	
	//publisher to move arm down
	arm_vel= nh_.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/joint_states", 1, &ShakeActionServer::joint_state_cb, this);

	//create subscriber to joint torques
	sub_torques = nh_.subscribe ("/mico_arm_driver/out/joint_efforts", 1, &ShakeActionServer::joint_effort_cb,this);

	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/mico_arm_driver/out/tool_position", 1, &ShakeActionServer::toolpos_cb, this);

	//subscriber for fingers
	sub_finger = nh_.subscribe("/mico_arm_driver/out/finger_position", 1, &ShakeActionServer::fingers_cb, this);
	  
	//subscriber for wrench
	sub_wrench = nh_.subscribe("/mico_arm_driver/out/tool_wrench", 1, &ShakeActionServer::wrench_cb, this);
	
	ROS_INFO("Shake action has started");
	
    as_.start();
  }

  ~ShakeActionServer(void)
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
	
	//method to shake object, use planner
	void lift(ros::NodeHandle n, double x){
		listenForArmData(30.0);

		geometry_msgs::PoseStamped p_target = current_pose;

		p_target.pose.position.z += x;
		segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
	}	
	
	void shake_down(float duration){
		geometry_msgs::TwistStamped v;
		ROS_INFO("inside shake down");
		
		v.twist.linear.x = 0;
		v.twist.linear.y = 0.0;
		v.twist.linear.z = +0.125;
		
		v.twist.angular.x = 0.0;
		v.twist.angular.y = 0.0;
		v.twist.angular.z = 0.0;
		
		float elapsed_time = 0.0;
		 
		float rate = 40;
		ros::Rate r(rate);
		
		listenForArmData(30.0);
		
		while(ros::ok() && !as_.isPreemptRequested()){			
			v.twist.linear.z = +0.125;
			
			arm_vel.publish(v);
			r.sleep();
			
			
			elapsed_time += (1.0/rate);
		
			if (elapsed_time > duration)
				break;
		}
		v.twist.linear.z = 0.0;
		arm_vel.publish(v);
	}
	
	
	void shake_up(float duration){
		geometry_msgs::TwistStamped v;
		ROS_INFO("inside shake up");
		
		v.twist.linear.x = 0;
		v.twist.linear.y = 0.0;
		v.twist.linear.z = -0.125;
		
		v.twist.angular.x = 0.0;
		v.twist.angular.y = 0.0;
		v.twist.angular.z = 0.0;
		
		float elapsed_time = 0.0; 
		
		float rate = 40;
		ros::Rate r(rate);
		
		listenForArmData(30.0);
		
		while(ros::ok() && !as_.isPreemptRequested()){			

			v.twist.linear.z = -0.125;
			
			arm_vel.publish(v);
			r.sleep();
			elapsed_time += (1.0/rate);
		
			if (elapsed_time > duration)
				break;
		}
		
		v.twist.linear.z = 0.0;
		arm_vel.publish(v);
		
	}

	
	void executeCB(const segbot_arm_manipulation::ShakeGoalConstPtr  &goal){
		
		listenForArmData(30.0);
		
		if(goal -> tgt_cloud.data.size() == 0){
			result_.success = false;
			ROS_INFO("[arm_shake_as.cpp] No object point clouds received...aborting");
			as_.setAborted(result_);
			return;
		}
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Shake action: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
        //step 1: transform to mico link space
        std::string sensor_frame_id = goal -> tgt_cloud.header.frame_id;
			
		listener.waitForTransform(sensor_frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		
		PointCloudT pcl_cloud;
		pcl::fromROSMsg(goal -> tgt_cloud, pcl_cloud);
		
		listenForArmData(30.0);
				
		//step 2 : determine if object in in hand
		if(goal -> verified){
			//step 3: shake the object
			shake_up(4); //raise above table
			shake_down(1.5);
			shake_up(1.5);
			segbot_arm_manipulation::openHand();

		}else{
			//for now if object is not in hand, abort
			ROS_WARN("object must already be in hand... aborting");
			result_.success = false;
			as_.setAborted(result_);
			return;
		}
			
		//step 4: move arm home		
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);

		//step 5: set result of action
		result_.success = true;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_shake_as");

  ShakeActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
