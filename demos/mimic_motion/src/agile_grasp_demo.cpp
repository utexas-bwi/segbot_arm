#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"

#include "agile_grasp/Grasps.h"

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

//publishers
ros::Publisher pub_velocity;
ros::Publisher cloud_pub;
ros::Publisher cloud_grasp_pub;
ros::Publisher pose_array_pub;
 
sensor_msgs::PointCloud2 cloud_ros;

bool heardGrasps = false;
agile_grasp::Grasps current_grasps;


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

void grasps_cb(const agile_grasp::Grasps &msg){
	current_grasps = msg;
	
	heardGrasps = true;
}

void listenForGrasps(float rate){
	ros::Rate r(rate);
	
	while (ros::ok()){
		ros::spinOnce();
		
		if (heardGrasps)
			return;
		
		r.sleep();
		
		
	}
	
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

Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d& Q)
{
	std::vector<int> axis_order_;
	axis_order_.push_back(2);
	axis_order_.push_back(0);
	axis_order_.push_back(1);
	
	Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
	R.col(axis_order_[0]) = Q.col(0); // grasp approach vector
	R.col(axis_order_[1]) = Q.col(1); // hand axis
	R.col(axis_order_[2]) = Q.col(2); // hand binormal
	return R;
}



geometry_msgs::PoseStamped graspToPose(agile_grasp::Grasp grasp, double hand_offset, std::string frame_id){
	
	
	
	Eigen::Vector3d center_; // grasp position
	Eigen::Vector3d surface_center_; //  grasp position projected back onto the surface of the object
	Eigen::Vector3d axis_; //  hand axis
	Eigen::Vector3d approach_; //  grasp approach vector
	Eigen::Vector3d binormal_; //  vector orthogonal to the hand axis and the grasp approach direction
	
	tf::vectorMsgToEigen(grasp.axis, axis_);
	tf::vectorMsgToEigen(grasp.approach, approach_);
	tf::vectorMsgToEigen(grasp.center, center_);
	tf::vectorMsgToEigen(grasp.surface_center, surface_center_);
	
	approach_ = -1.0 * approach_; // make approach vector point away from handle centroid
	binormal_ = axis_.cross(approach_); // binormal (used as rotation axis to generate additional approach vectors)
	
	//step 1: calculate hand orientation
	
	// calculate first hand orientation
	Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
	R.col(0) = -1.0 * approach_;
	R.col(1) = axis_;
	R.col(2) << R.col(0).cross(R.col(1));
			
	// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
	Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(M_PI, approach_));
	
	// calculate second hand orientation
	Eigen::Matrix3d Q = Eigen::MatrixXd::Zero(3, 3);
	Q.col(0) = T * approach_;
	Q.col(1) = T * axis_;
	Q.col(2) << Q.col(0).cross(Q.col(1));
	
	// reorder rotation matrix columns according to axes ordering of the robot hand
	Eigen::Matrix3d R1 = reorderHandAxes(R);
	Eigen::Matrix3d R2 = reorderHandAxes(Q);
	
	
	// convert Eigen rotation matrices to TF quaternions and normalize them
	tf::Matrix3x3 TF1, TF2;
	tf::matrixEigenToTF(R1, TF1);
	tf::matrixEigenToTF(R2, TF2);
	tf::Quaternion quat1, quat2;
	TF1.getRotation(quat1);
	TF2.getRotation(quat2);
	quat1.normalize();
	quat2.normalize();
	
	std::vector<tf::Quaternion> quats;
	quats.push_back(quat1);
	quats.push_back(quat2);
	
	//use the first quaterneon for now
	tf::Quaternion quat = quats.at(0);
	
	//angles to try
	double theta = 0.0;
	
	// calculate grasp position
	Eigen::Vector3d position;
	Eigen::Vector3d approach = -1.0 * approach_;
	if (theta != 0)
	{
		// project grasp bottom position onto the line defined by grasp surface position and approach vector
		Eigen::Vector3d s, b, a;
		position = (center_ - surface_center_).dot(approach) * approach;
		position += surface_center_;
	}
	else
		position = center_;
		
	// translate grasp position by <hand_offset_> along the grasp approach vector
	position = position + hand_offset * approach;
			
	geometry_msgs::PoseStamped pose_st;
	pose_st.header.stamp = ros::Time(0);
	pose_st.header.frame_id = frame_id;
	tf::pointEigenToMsg(position, pose_st.pose.position);
    tf::quaternionTFToMsg(quat, pose_st.pose.orientation);
		
	
	
	return pose_st;
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
	  
	//subscriber for grasps
	ros::Subscriber sub_grasps = n.subscribe("/find_grasps/grasps_handles",1, grasps_cb);  
	  
	  
	//publish velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	//publish pose array
	pose_array_pub = n.advertise<geometry_msgs::PoseArray>("/agile_grasp_demo/pose_array", 10);
	
	//debugging publisher
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
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
	
	//publish to agile_grasp
	cloud_grasp_pub.publish(cloud_ros);
	
	//wait for response at 30 Hz
	listenForGrasps(30.0);
	
	geometry_msgs::PoseArray poses_msg;
	poses_msg.header.seq = 1;
	poses_msg.header.stamp = cloud_ros.header.stamp;
	poses_msg.header.frame_id = cloud_ros.header.frame_id;
	
	ROS_INFO("[agile_grasp_demo.cpp] Heard %i grasps",(int)current_grasps.grasps.size());
	
	double hand_offset = 0.0;
	
	for (unsigned int i = 0; i < current_grasps.grasps.size(); i++){
		geometry_msgs::PoseStamped p_i = graspToPose(current_grasps.grasps.at(i),hand_offset,cloud_ros.header.frame_id);

		
		poses_msg.poses.push_back(p_i.pose);
		
	}

	pose_array_pub.publish(poses_msg);

	return 0;
}
