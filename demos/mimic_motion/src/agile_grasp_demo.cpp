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
#include "jaco_msgs/ArmJointAnglesAction.h"



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

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>


#define PI 3.14159265


//const float home_position [] = { -1.84799570991366, -0.9422852495301872, -0.23388692957209883, -1.690986384686938, 1.37682658669572, 3.2439323416434624};
const float home_position [] = {-1.9461704803383473, -0.39558648095261406, -0.6342860089305954, -1.7290658598495474, 1.4053863262257316, 3.039252699220428};
const float home_position_approach [] = {-1.9480954131742567, -0.9028227948134995, -0.6467984718381701, -1.4125267937404524, 0.8651278801122975, 3.73659131064558};


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//where we store results from calling the perception service
std::vector<PointCloudT::Ptr > detected_objects;
PointCloudT::Ptr cloud_plane (new PointCloudT);

using namespace std;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

//some defines related to filtering candidate grasps
#define MAX_DISTANCE_TO_PLANE 0.075

sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;

geometry_msgs::PoseStamped current_moveit_pose;


//publishers
ros::Publisher pub_velocity;
ros::Publisher cloud_pub;
ros::Publisher cloud_grasp_pub;
ros::Publisher pose_array_pub;
ros::Publisher pose_pub;
ros::Publisher pose_fk_pub;
 
sensor_msgs::PointCloud2 cloud_ros;

bool heardGrasps = false;
agile_grasp::Grasps current_grasps;


struct GraspCartesianCommand {
	sensor_msgs::JointState approach_q;
	geometry_msgs::PoseStamped approach_pose;
	
	sensor_msgs::JointState grasp_q;
	geometry_msgs::PoseStamped grasp_pose;
	
	
};


/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
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
  heardPose = true;
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

void moveToCurrentAngles(){
	actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
	
	jaco_msgs::ArmJointAnglesGoal goalJoints;
	
	listenForArmData(30.0);
	
	goalJoints.angles.joint1 = current_state.position[0];
	goalJoints.angles.joint2 = current_state.position[1];
	goalJoints.angles.joint3 = current_state.position[2];
	goalJoints.angles.joint4 = current_state.position[3];
	goalJoints.angles.joint5 = current_state.position[4];
	goalJoints.angles.joint6 = current_state.position[5];
	
	ac.waitForServer();

    ac.sendGoal(goalJoints);

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

double angular_difference(geometry_msgs::Quaternion c,geometry_msgs::Quaternion d){
	Eigen::Vector4f dv;
	dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
	Eigen::Matrix<float, 3,4> inv;
	inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
	inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = c.w;	inv(1,3) = -c.x;
	inv(2,0) = -c.z; inv(2,1) = -c.y;inv(2,2) = c.x;  inv(2,3) = c.w;
	
	Eigen::Vector3f m = inv * dv * -2.0;
	return m.norm();
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
	
	// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
	Eigen::Transform<double, 3, Eigen::Affine> T_R(Eigen::AngleAxis<double>(M_PI/2, approach_));
	
	//to do: compute the second possible grasp by rotating -M_PI/2 instead
	
	// calculate first hand orientation
	Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
	R.col(0) = -1.0 * approach_;
	R.col(1) = T_R * axis_;
	R.col(2) << R.col(0).cross(R.col(1));
			
	Eigen::Matrix3d R1 = reorderHandAxes(R);
	tf::Matrix3x3 TF1;		
	tf::matrixEigenToTF(R1, TF1);
	tf::Quaternion quat1;
	TF1.getRotation(quat1);		
	quat1.normalize();
	
	// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
	/*Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(M_PI, approach_));
	
	// calculate second hand orientation
	Eigen::Matrix3d Q = Eigen::MatrixXd::Zero(3, 3);
	Q.col(0) = T * approach_;
	Q.col(1) = T * axis_;
	Q.col(2) << Q.col(0).cross(Q.col(1));
	
	// reorder rotation matrix columns according to axes ordering of the robot hand
	//Eigen::Matrix3d R1 = reorderHandAxes(R);
	Eigen::Matrix3d R2 = reorderHandAxes(Q);

	// convert Eigen rotation matrices to TF quaternions and normalize them
	tf::Matrix3x3 TF2;
	tf::matrixEigenToTF(R2, TF2);
	tf::Quaternion quat2;
	TF2.getRotation(quat2);
	quat2.normalize();
	
	std::vector<tf::Quaternion> quats;
	quats.push_back(quat1);
	quats.push_back(quat2);*/
	
	//use the first quaterneon for now
	tf::Quaternion quat = quat1;
	
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

bool acceptGrasp(GraspCartesianCommand gcc, PointCloudT::Ptr object, Eigen::Vector4f plane_c){
	//filter 1: if too close to the plane
	pcl::PointXYZ p_a;
	p_a.x=gcc.approach_pose.pose.position.x;
	p_a.y=gcc.approach_pose.pose.position.y;
	p_a.z=gcc.approach_pose.pose.position.z;
	
	pcl::PointXYZ p_g;
	p_g.x=gcc.grasp_pose.pose.position.x;
	p_g.y=gcc.grasp_pose.pose.position.y;
	p_g.z=gcc.grasp_pose.pose.position.z;
	
	if (pcl::pointToPlaneDistance(p_a, plane_c) < MAX_DISTANCE_TO_PLANE 
		|| pcl::pointToPlaneDistance(p_g, plane_c) < MAX_DISTANCE_TO_PLANE){
		
		return false;
	}
	
	
	
	return true;
}

bool moveToPose(geometry_msgs::PoseStamped g){
	actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/mico_arm_driver/arm_pose/arm_pose", true);

	jaco_msgs::ArmPoseGoal goalPose;
  
 

	goalPose.pose = g;


	ROS_INFO_STREAM(goalPose);

	  ac.waitForServer();
	  ROS_DEBUG("Waiting for server.");
	  //finally, send goal and wait
	  ROS_INFO("Sending goal.");
	  ac.sendGoal(goalPose);
	  ac.waitForResult();
		
	return true;
}

moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
	ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
	
	
	moveit_msgs::GetPositionIK::Request ikine_request;
	moveit_msgs::GetPositionIK::Response ikine_response;
	ikine_request.ik_request.group_name = "arm";
	ikine_request.ik_request.pose_stamped = p;
	
	/* Call the service */
	if(ikine_client.call(ikine_request, ikine_response)){
		ROS_INFO("IK service call success:");
		ROS_INFO_STREAM(ikine_response);
	} else {
		ROS_INFO("IK service call FAILED. Exiting");
	}
	
	return ikine_response;
}

void spinSleep(double duration){
	int rateHertz = 40;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)duration * rateHertz; i++) {
		
		
		ros::spinOnce();
		r.sleep();
	}
}

void updateFK(ros::NodeHandle n){
	ros::ServiceClient fkine_client = n.serviceClient<moveit_msgs::GetPositionFK> ("/compute_fk");
	
	moveit_msgs::GetPositionFK::Request fkine_request;
	moveit_msgs::GetPositionFK::Response fkine_response;

	
	//wait to get lates joint state values
	listenForArmData(30.0);
	sensor_msgs::JointState q_true = current_state;
	
	//Load request with the desired link
	fkine_request.fk_link_names.push_back("mico_end_effector");

	//and the current frame
	fkine_request.header.frame_id = "mico_link_base";

	//finally we let moveit know what joint positions we want to compute
	//in this case, the current state
	fkine_request.robot_state.joint_state = q_true;

	ROS_INFO("Making FK call");
 	if(fkine_client.call(fkine_request, fkine_response)){
 		pose_fk_pub.publish(fkine_response.pose_stamped.at(0));
 		ros::spinOnce();
 		current_moveit_pose = fkine_response.pose_stamped.at(0);
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(fkine_response);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
 	
 	
 	//
}


// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

/*
void moveToJointState(const float* js){
	
	
	moveit_utils::MicoMoveitJointPose::Request req;
	moveit_utils::MicoMoveitJointPose::Response res;
	
	for(int i = 0; i < 6; i++){
        switch(i) {
            case 0  :    req.target.joint1 = js[i]; break;
            case 1  :    req.target.joint2 =  js[i]; break;
            case 2  :    req.target.joint3 =  js[i]; break;
            case 3  :    req.target.joint4 =  js[i]; break;
            case 4  :    req.target.joint5 =  js[i]; break;
            case 5  :    req.target.joint6 =  js[i]; break;
        }
	//ROS_INFO("Requested angle: %f", q_vals.at(i));
    }
	
	if(client_joint_command.call(req, res)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
}*/

void moveToJointState(ros::NodeHandle n, sensor_msgs::JointState target){
	//check if this is specified just for the arm
	sensor_msgs::JointState q_target;
	if (target.position.size() != NUM_JOINTS_ARMONLY){
		//in this case, the first four values are for the base joints
		for (int i = 4; i < target.position.size(); i ++){
			q_target.position.push_back(target.position.at(i));
			q_target.name.push_back(target.name.at(i));
		}
		q_target.header = target.header;
	}
	else 
		q_target = target;
	
	ROS_INFO("Target joint state:");
	ROS_INFO_STREAM(q_target);
	
	moveit_utils::AngularVelCtrl::Request	req;
	moveit_utils::AngularVelCtrl::Response	resp;
	
	ros::ServiceClient ikine_client = n.serviceClient<moveit_utils::AngularVelCtrl> ("/angular_vel_control");
	
	req.state = q_target;
	
	pressEnter();
	
	if(ikine_client.call(req, resp)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(resp);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
	
}

void moveToJointStateMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target/*sensor_msgs::JointState q_target*/){
	moveit_utils::MicoMoveitCartesianPose::Request 	req;
	moveit_utils::MicoMoveitCartesianPose::Response res;
	
	req.target = p_target;
	
	ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose> ("/mico_cartesianpose_service");
	if(client.call(req, res)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}
	
	/*moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
    group.setPlanningTime(5.0); //10 second maximum for collision computation*/

	
	
	/*moveit_utils::MicoMoveitJointPose::Request req;
	moveit_utils::MicoMoveitJointPose::Response res;
	*/
	/*for(int i = 0; i < NUM_JOINTS_ARMONLY; i++){
        switch(i) {
            case 0  :    req.target.joint1 = q_target.position[0]; break;
            case 1  :    req.target.joint2 = q_target.position[1]; break;
            case 2  :    req.target.joint3 = q_target.position[2]; break;
            case 3  :    req.target.joint4 = q_target.position[3]; break;
            case 4  :    req.target.joint5 = q_target.position[4]; break;
            case 5  :    req.target.joint6 = q_target.position[5]; break;
        }
	//ROS_INFO("Requested angle: %f", q_vals.at(i));
    }
	ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitJointPose> ("/mico_jointpose_service");
	if(client.call(req, res)){
 		ROS_INFO("Call successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call failed. Terminating.");
 		//ros::shutdown();
 	}*/
}

void cartesianVelocityMove(double dx, double dy, double dz, double duration){
	int rateHertz = 40;
	geometry_msgs::TwistStamped velocityMsg;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)duration * rateHertz; i++) {
		
		
		velocityMsg.twist.linear.x = dx;
		velocityMsg.twist.linear.y = dy;
		velocityMsg.twist.linear.z = dz;
		
		velocityMsg.twist.angular.x = 0.0;
		velocityMsg.twist.angular.y = 0.0;
		velocityMsg.twist.angular.z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		ROS_INFO("Published cartesian vel. command");
		r.sleep();
	}
	
}

//lifts ef specified distance
void lift_velocity(double vel, double distance){
	ros::Rate r(4);
	ros::spinOnce();
	double distance_init = .2;
	geometry_msgs::TwistStamped T;
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;

	for(int i = 0; i < std::abs(distance/vel/.25); i++){
		ros::spinOnce();
		if(distance > 0)
			T.twist.linear.z = vel;
		else
			T.twist.linear.z= -vel;
		pub_velocity.publish(T);
		r.sleep();
	}
	T.twist.linear.z= 0.0;
	pub_velocity.publish(T);
	
	
}

void lift(ros::NodeHandle n, double x){
	listenForArmData(30.0);
	
	geometry_msgs::PoseStamped p_target = current_pose;
	
	p_target.pose.position.z += x;
	moveToJointStateMoveIt(n,p_target);
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "agile_grasp_demo");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);

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
	
	//publish pose 
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);
	pose_fk_pub = n.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_fk_out", 10);
	
	//debugging publisher
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
	cloud_grasp_pub = n.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
	
	//used to compute transfers
	tf::TransformListener listener;
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	
	//user input
    char in;
	
	

	//cartesianVelocityMove(0,0,0.2,1.0);
	//lift(n,0.1);
	//cartesianVelocityMove(0,0,-0.2,1.0);
	
	//open and open again fingers
	//pressEnter();
	//moveFinger(7200);
	
	/*lift(n,-0.1);
	spinSleep(3.0);
	moveToCurrentAngles();
	moveFinger(7200);
	lift(n,0.1);	
	spinSleep(3.0);
	moveToCurrentAngles();
	moveFinger(100);*/
	
	ROS_INFO("Demo starting...");
	pressEnter();
	
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
	
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=srv.response.cloud_plane_coef[i];
	
	//step 3: select which object to grasp
	int selected_object = selectObjectToGrasp(detected_objects);
	
	//publish object to find grasp topic
	pcl::PCLPointCloud2 pc_target;
	pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
	pcl_conversions::fromPCL(pc_target,cloud_ros);
	
	//publish to agile_grasp
	ROS_INFO("Publishing point cloud...");
	cloud_grasp_pub.publish(cloud_ros);
	
	//wait for response at 30 Hz
	listenForGrasps(30.0);
	
	geometry_msgs::PoseArray poses_msg;
	poses_msg.header.seq = 1;
	poses_msg.header.stamp = cloud_ros.header.stamp;
	poses_msg.header.frame_id = "mico_api_origin";
	
	ROS_INFO("[agile_grasp_demo.cpp] Heard %i grasps",(int)current_grasps.grasps.size());
	
	//next, compute approach and grasp poses for each detected grasp
	double hand_offset_grasp = -0.02;
	double hand_offset_approach = -0.13;
	
	//wait for transform from visual space to arm space
	listener.waitForTransform(cloud_ros.header.frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
	
	std::vector<GraspCartesianCommand> grasp_commands;
	std::vector<geometry_msgs::PoseStamped> poses;
	for (unsigned int i = 0; i < current_grasps.grasps.size(); i++){
		geometry_msgs::PoseStamped p_grasp_i = graspToPose(current_grasps.grasps.at(i),hand_offset_grasp,cloud_ros.header.frame_id);
		geometry_msgs::PoseStamped p_approach_i = graspToPose(current_grasps.grasps.at(i),hand_offset_approach,cloud_ros.header.frame_id);
		
		//
		
		GraspCartesianCommand gc_i;
		gc_i.approach_pose = p_approach_i;
		gc_i.grasp_pose = p_grasp_i;
		
		
		
		if (acceptGrasp(gc_i,detected_objects.at(selected_object),plane_coef_vector)){
			
			listener.transformPose("mico_api_origin", gc_i.approach_pose, gc_i.approach_pose);
			listener.transformPose("mico_api_origin", gc_i.grasp_pose, gc_i.grasp_pose);
			
			//filter two -- if IK fails
			moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,gc_i.approach_pose);
			moveit_msgs::GetPositionIK::Response  ik_response_grasp = computeIK(n,gc_i.grasp_pose);
	
			if (ik_response_approach.error_code.val == 1 && ik_response_grasp.error_code.val == 1){
				//store the IK results
				gc_i.approach_q = ik_response_approach.solution.joint_state;
				gc_i.grasp_q = ik_response_grasp.solution.joint_state;
				
				
				grasp_commands.push_back(gc_i);
				poses.push_back(p_grasp_i);
				poses_msg.poses.push_back(gc_i.approach_pose.pose);
			}
		}
		
		
	}
	
	//make sure we're working with the correct tool pose
	listenForArmData(30.0);
	ROS_INFO("[agile_grasp_demo.cpp] Heard arm pose.");
	
	//now, select the target grasp
	updateFK(n);
	
	//find the grasp with closest orientatino to current pose
	double min_diff = 1000000.0;
	int min_diff_index = -1;
	
	for (unsigned int i = 0; i < grasp_commands.size(); i++){
		double d_i = angular_difference(grasp_commands.at(i).approach_pose.pose.orientation, current_pose.pose.orientation);
		
		ROS_INFO("Distance for pose %i:\t%f",(int)i,d_i);
		if (d_i < min_diff){
			min_diff_index = (int)i;
			min_diff = d_i;
		}
	}

	if (min_diff_index == -1){
		ROS_WARN("No feasible grasps found, aborting.");
		return 0;
	}

	pose_array_pub.publish(poses_msg);


	//publish individual pose
	pose_pub.publish(grasp_commands.at(min_diff_index).approach_pose);
	pressEnter();
	moveToJointStateMoveIt(n,grasp_commands.at(min_diff_index).approach_pose);
	//pressEnter();
	
	//open fingers
	//pressEnter();
	//moveFinger(100);


	sleep(1.0);
	//pose_pub.publish(grasp_commands.at(min_diff_index).grasp_pose);
	/*std::cout << "Press '1' to move to approach pose or Ctrl-z to quit..." << std::endl;		
	std::cin >> in;*/
	
	moveToJointStateMoveIt(n,grasp_commands.at(min_diff_index).grasp_pose);
	spinSleep(3.0);
	
	//listenForArmData(30.0);
	//close fingers
	//pressEnter();
	moveFinger(7200);
	
	
	//lift for a while
	//pressEnter();
	//cartesianVelocityMove(0,0,0.2,1.0);
	lift(n,0.1);
	lift(n,-0.09);
	
	//lift_velocity(0.5,0.3);
	//lift_velocity(-0.5,0.25);
	
	spinSleep(3.0);
	moveFinger(100);
	lift(n,0.1);
	
	//moveToJointState(home_position_approach);
	//moveToJointState(home_position);
	
	return 0;
}
