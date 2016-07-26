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
#include <segbot_arm_manipulation/PushAction.h>
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

#define MIN_DISTANCE_TO_PLANE 0.05

using namespace std;

class PushActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<segbot_arm_manipulation::PushAction> as_; 
  
  std::string action_name_;
  
  segbot_arm_manipulation::PushFeedback feedback_;
  segbot_arm_manipulation::PushResult result_;
  
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

  PushActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PushActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	heardPose = false;
	heardJoinstState = false;
	heardWrench = false;
	
	//publisher to move arm down
	arm_vel= nh_.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);

	//create subscriber to joint angles
	sub_angles = nh_.subscribe ("/joint_states", 1, &PushActionServer::joint_state_cb, this);

	//create subscriber to joint torques
	sub_torques = nh_.subscribe ("/mico_arm_driver/out/joint_efforts", 1, &PushActionServer::joint_effort_cb,this);

	//create subscriber to tool position topic
	sub_tool = nh_.subscribe("/mico_arm_driver/out/tool_position", 1, &PushActionServer::toolpos_cb, this);

	//subscriber for fingers
	sub_finger = nh_.subscribe("/mico_arm_driver/out/finger_position", 1, &PushActionServer::fingers_cb, this);
	  
	//subscriber for wrench
	sub_wrench = nh_.subscribe("/mico_arm_driver/out/tool_wrench", 1, &PushActionServer::wrench_cb, this);
	
	debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mico_arm_driver/in/debug_pose", 2);
	
	ROS_INFO("Push action has started");
	
    as_.start();
  }

  ~PushActionServer(void)
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
	
	geometry_msgs::Point find_right_side(PointCloudT pcl_cloud){
		PointT max;
		PointT min;
		pcl::getMinMax3D(pcl_cloud, min, max);
		
		Eigen::Vector4f center; 
		pcl::compute3DCentroid(pcl_cloud, center);
		
		geometry_msgs::Point goal_pt;
		goal_pt.x = center(0);
		goal_pt.y = max.y; 
		
		float dist = (max.z - min.z) /4;
		goal_pt.z = min.z - dist;
		return goal_pt;
	}
	
	void push(float duration){ 
		geometry_msgs::TwistStamped v;
		 
		v.twist.linear.x = 0;
		v.twist.linear.y = 0.125;
		v.twist.linear.z = 0.0;
		
		v.twist.angular.x = 0.0;
		v.twist.angular.y = 0.0;
		v.twist.angular.z = 0.0;
		
		float rate = 40;
		ros::Rate r(rate);
		for(int i = 0; i< (int) rate * duration; i++){
			arm_vel.publish(v);
			r.sleep();
			ros::spinOnce();
		}
		
		v.twist.linear.y = 0.0;
		arm_vel.publish(v);
	}
	
	std::vector<geometry_msgs::PoseStamped> find_quats (geometry_msgs::PoseStamped goal_pose){
		std::vector<geometry_msgs::Quaternion> possible_quats;
		possible_quats.push_back(tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 0, 3.14/2));
		possible_quats.push_back(tf::createQuaternionMsgFromRollPitchYaw(3.14/2, 0, 3.14/2));

		
		std::vector<geometry_msgs::PoseStamped> ik_possible;
		for(unsigned int i = 0; i< possible_quats.size(); i++){
			goal_pose.pose.orientation = possible_quats.at(i);
			
			//transform into the arm's space
			listener.transformPose("mico_api_origin", goal_pose, goal_pose);
			
			moveit_msgs::GetPositionIK::Response ik_response = segbot_arm_manipulation::computeIK(nh_,goal_pose);
			if(ik_response.error_code.val == 1) {
				ik_possible.push_back(goal_pose);
			}
			//ROS_INFO_STREAM(ik_response.error_code.val);
		}
		
		return ik_possible;
	}

	
	void executeCB(const segbot_arm_manipulation::PushGoalConstPtr  &goal){
		listenForArmData(30.0);
		
		if(goal -> tgt_cloud.data.size() == 0){
			result_.success = false;
			ROS_INFO("[arm_push_as.cpp] No object point clouds received...aborting");
			as_.setAborted(result_);
			return;
		}
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Push action: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
		
		ROS_INFO("checked for preempted and tgt cloud");
		//step one: close fingers
		segbot_arm_manipulation::closeHand();
		
		//step 2: transform into the arm's base
		sensor_msgs::PointCloud2 tgt= goal -> tgt_cloud;
		std::string sensor_frame_id = tgt.header.frame_id;
			
	
		listener.waitForTransform(sensor_frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		
		sensor_msgs::PointCloud transformed_pc;
		sensor_msgs::convertPointCloud2ToPointCloud(tgt,transformed_pc);
		
		listener.transformPointCloud("mico_link_base", transformed_pc ,transformed_pc); 
		sensor_msgs::convertPointCloudToPointCloud2(transformed_pc, tgt);
	
		PointCloudT pcl_cloud;
		pcl::fromROSMsg(tgt, pcl_cloud);
		
		ROS_INFO("made tgt_cloud into pcl cloud and transformed frame id");
		
		//step 3: find the side of the object, set to slightly in further right of object
		geometry_msgs::Point right_side = find_right_side(pcl_cloud);
		
		geometry_msgs::PoseStamped goal_pose;
		goal_pose.header.frame_id = goal -> tgt_cloud.header.frame_id;
		
		goal_pose.pose.position = right_side; 
		goal_pose.pose.position.y += 0.05;
		
		ROS_INFO("found right side of object");
	
		//step 4: find the possible hand orientation
		std::vector<geometry_msgs::PoseStamped> ik_possible = find_quats(goal_pose);
		
		//if the IK are invalid, it is not possible to push
		if (ik_possible.size() == 0 ){ 
			result_.success = false;
			ROS_INFO("[arm_push_as.cpp] No possible pose");
			as_.setAborted(result_);
			return;
		}
		
		goal_pose = ik_possible.at(0);
		
		debug_pub.publish(goal_pose);
		
		listenForArmData(30.0);
		
		//|| (pcl::pointToPlaneDistance(pt, goal -> cloud_plane) < MIN_DISTANCE_TO_PLANE)
		
		//step 5: move to the goal pose
		segbot_arm_manipulation::moveToPoseMoveIt(nh_,goal_pose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_,goal_pose);

		//step 6: push the object 
		push(1.5);
		
		listenForArmData(30.0);
		
		//step 7: move arm home
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);
		segbot_arm_manipulation::moveToJointState(nh_, goal -> arm_home);

		
		result_.success = true;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_push_as");
  
  ROS_WARN("about to call push action server");

  PushActionServer as(ros::this_node::getName());
  
  ROS_WARN("made push action server call");
  
  ros::spin();

  return 0;
}