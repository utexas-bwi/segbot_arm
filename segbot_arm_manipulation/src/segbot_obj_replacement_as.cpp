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
		//sub_grasps = nh_.subscribe("/find_grasps/grasps_handles",1, &ObjReplacementActionServer::grasps_cb,this);  

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

	//downsamples cloud using VoxelGrid filter
	void downsample_clouds(PointCloudT::Ptr in_cloud, PointCloudT::Ptr out_cloud, float leaf_size){
		pcl::VoxelGrid<PointT> grid; 
		grid.setInputCloud(in_cloud);
		grid.setLeafSize (leaf_size, leaf_size, leaf_size); 
		grid.filter(*out_cloud);
	}
	
	/*Assumptions: the robot has already approached the table, 
	 * an object is in hand, the arm is currently still in safety mode,
	 * cloud is organized and dense*/
	void executeCB(const segbot_arm_manipulation::ObjReplacementGoalConstPtr &goal){
		//step1: get the table scene, check validity
		segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh_);
		
		if ((int)table_scene.cloud_clusters.size() == 0){
			ROS_ERROR("[segbot_arm_replacement_as] a table must be present");
			result_.success = false;
			result_.error_msg = "a table must be present";
			as_.setAborted(result_);
			return;
		}
		sensor_msgs::PointCloud2 plane = table_scene.cloud_plane;
		
		
		//(later) step2: get information about the object
		
		//step3: cropbox filter to reduce size of points to check
		
		
		//transform the table scene point cloud 
		sensor_msgs::PointCloud scene_plane;
		listener.waitForTransform(plane.header.frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		sensor_msgs::convertPointCloud2ToPointCloud(plane, scene_plane);
		listener.transformPointCloud("mico_link_base", scene_plane ,scene_plane); 
		
		//update the plane cloud to the transformed cloud
		sensor_msgs::convertPointCloudToPointCloud2(scene_plane, plane);
		
		//get a pcl point cloud of the given plane
		PointCloudT::Ptr pcl_scene_plane (new PointCloudT);
		pcl::fromROSMsg(plane, *pcl_scene_plane);
		
		//step4: voxel grid filter with a distance size of 5cm
		PointCloudT::Ptr plane_down_sam (new PointCloudT);
		downsample_clouds(pcl_scene_plane, plane_down_sam, 0.05f);
		
		//step5: for each of these points, starting in the middle
			//get a z value some height above the plane
			//add some value to this to create a goal point 
			//check the IK, if possible go to location
		
		int num_points = (int) plane_down_sam->points.size();
		if ((int)num_points == 0){
			ROS_ERROR("[segbot_arm_replacement_as] the table contains no points");
			result_.success = false;
			result_.error_msg = "down sampled cloud must still have points";
			as_.setAborted(result_);
			return;
		}
		
		
		//for now go through points like normal
		for(int ind = 0; ind < num_points; ind++){
			//create the current goal for the arm to move to when resetting
			geometry_msgs::PoseStamped current_goal;
			current_goal.header.frame_id = current_pose.header.frame_id;
			current_goal.pose.position.x = plane_down_sam->points[ind].x;
			current_goal.pose.position.y = plane_down_sam->points[ind].y;
			current_goal.pose.position.z = plane_down_sam->points[ind].z + 0.1; //want it to be slightly above the table still
			//for now use 0,0,0; determine a better quaternion 
			current_goal.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0 , 0 , 0);
			
			moveit_msgs::GetPositionIK::Response ik_response_1 = segbot_arm_manipulation::computeIK(nh_,current_goal);
			if (ik_response_1.error_code.val == 1){
				//go to the position and for now, break
				segbot_arm_manipulation::moveToPoseMoveIt(nh_, current_goal);
				//to do: determine if the position was reached, if not retry?
				break;
			}

		}
		
		//drop the object
		segbot_arm_manipulation::openHand();
		
		//step7: home arm 
		segbot_arm_manipulation::homeArm(nh_);
		
		//todo: set results based on if the location was reached and if the object is on the table
		
	}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_obj_replacement_as");

  ObjReplacementActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
