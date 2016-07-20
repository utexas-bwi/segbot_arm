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
	
	//TO DO: make sure the xyz are facing the way I think they are 
	geometry_msgs::Point find_top_center(PointCloudT pcl_curr){
		//find max of all points
		Eigen::Vector4f max;
		Eigen::Vector4f min; 
		pcl::getMinMax3D(pcl_curr, min, max);

		//find average of all points
		Eigen::Vector4f centroid_pts;
		pcl::compute3DCentroid(pcl_curr, centroid_pts);
		
		geometry_msgs::Point top_center;
		top_center.x = centroid_pts(0);
		top_center.y = centroid_pts(1);
		top_center.z = max(2);
		return top_center;
	}

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
	
	/*geometry_msgs::Point find_top(PointCloudT pcl_cloud){
		//cloud is checked for size in the callback function
		geometry_msgs::Point result_pt;
		PointT max_point = pcl_cloud.points[0];
		for(unsigned int i = 0; i < pcl_cloud.points.size(); i++){
			PointT current = pcl_cloud.points[i];
			if(current.z >= max_point.z){
				max_point = current; 
			}
		}
		result_pt.x = max_point.x;
		result_pt.y = max_point.y;
		result_pt.z = max_point.z;
		return result_pt;
	}*/
	
	void press_down(){
		geometry_msgs::TwistStamped v;
		 
		v.twist.linear.x = 0;
		v.twist.linear.y = 0.0;
		v.twist.linear.z = -0.125;
		
		v.twist.angular.x = 0.0;
		v.twist.angular.y = 0.0;
		v.twist.angular.z = 0.0;
		
		ros::Rate r(40);
		
		geometry_msgs::Pose tool_pose_last = current_pose.pose;
		while(current_pose.pose.position.z <= tool_pose_last.position.z){
			arm_vel.publish(v);
			r.sleep();
			tool_pose_last = current_pose.pose;
			ros::spinOnce();
		} 
		v.twist.linear.z = 0.0;
		arm_vel.publish(v);
	}
	
	
	void executeCB(const segbot_arm_manipulation::PressGoalConstPtr  &goal){
		ROS_INFO("at beginning of press cb");
		listenForArmData(30.0);
		
		ROS_INFO("first heard data in press cb");
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
		//step one: close fingers
		//moveFinger(FINGER_FULLY_CLOSED);
		
		segbot_arm_manipulation::closeHand();
	
		//step 2: transform into the arm's base
		
		std::string sensor_frame_id = goal -> tgt_cloud.header.frame_id;
			
		listener.waitForTransform(sensor_frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		
		PointCloudT pcl_cloud;
		pcl::fromROSMsg(goal -> tgt_cloud, pcl_cloud);
		
		//find the top of the object
		geometry_msgs::Point top = find_top_center(pcl_cloud);
		
		geometry_msgs::PoseStamped goal_pose;
		//set the goal pose to slightly above the object
		ROS_INFO_STREAM("header info for goal pose");
		ROS_INFO_STREAM(goal -> tgt_cloud.header.frame_id);
		goal_pose.header.frame_id = goal -> tgt_cloud.header.frame_id;
		goal_pose.pose.position = top; 
		goal_pose.pose.position.z += 0.125; //TO DO: make sure this number is okay
		
		//set orientation to have fingers to the left with the knuckles facing up or down
		goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,3.14/1.9,0);
		
		//transform into the arm's space
		listener.transformPose("mico_api_origin", goal_pose, goal_pose);
		
		//compute IK
		moveit_msgs::GetPositionIK::Response  ik_response = segbot_arm_manipulation::computeIK(nh_,goal_pose);

		//if the IK are invalid, it is not possible to press
		//the arm has not moved yet so no need to return it anywhere
		if (ik_response.error_code.val != 1){
			result_.success = false;
			ROS_INFO("[arm_press_as.cpp] Cannot move above object");
			as_.setAborted(result_);
			return;
		}
		
		listenForArmData(30.0); 
		
		segbot_arm_manipulation::moveToPoseMoveIt(nh_,goal_pose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_,goal_pose);

		press_down();
		
		//To Do: check if a new goal has been sent
		//TO DO: add startSensoryDataCollection(); and 	stopSensoryDataCollection();
		//To Do: check if moveToPoseMoveIt is okay with the plane
		
		listenForArmData(30.0);
				
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);
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
