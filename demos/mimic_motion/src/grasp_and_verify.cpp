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

//added
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

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


#include <segbot_arm_manipulation/arm_utils.h>


//Math and definitions
#include <stdlib.h>

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

//used when deciding whether a pair of an approach pose and a grasp pose are good;
//if the angular difference in joint space is too big, this means that the robot 
//cannot directly go from approach to grasp pose (so we filter those pairs out)
#define ANGULAR_DIFF_THRESHOLD 3.0

sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;

geometry_msgs::PoseStamped current_moveit_pose;

//store out-of-view position here
sensor_msgs::JointState joint_state_outofview;
geometry_msgs::PoseStamped pose_outofview;

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


moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
	ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
	
	
	moveit_msgs::GetPositionIK::Request ikine_request;
	moveit_msgs::GetPositionIK::Response ikine_response;
	ikine_request.ik_request.group_name = "arm";
	ikine_request.ik_request.pose_stamped = p;
	
	/* Call the service */
	if(ikine_client.call(ikine_request, ikine_response)){
		ROS_INFO("IK service call success:");
		
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
}


// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
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
	segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
}

/*Method to check if the fingers are open.
 * return: true if the fingers are open, grasp verified - false if the fingers are closed, grasp most likely failed
 * Jivko to work on reading force exerted downwards to help!
 */ 
bool fingers_open(){ 
	 float tolerance = 140; //might want to update appropriately //might want to change to 120 according to data
	 
	 float finger1_diff = abs(current_finger.finger1 - FINGER_FULLY_CLOSED); //make new fingers closed variable 
	 float finger2_diff = abs(current_finger.finger2 - FINGER_FULLY_CLOSED);
	 
	 /*ROS_INFO_STREAM("finger 1:");
	 ROS_INFO_STREAM(current_finger.finger1);
	 ROS_INFO_STREAM("difference for finger 1");
	 ROS_INFO_STREAM(finger1_diff);
	 ROS_INFO_STREAM("finger 2");
	 ROS_INFO_STREAM(current_finger.finger2);
	 ROS_INFO_STREAM("difference for finger 2");
	 ROS_INFO_STREAM(finger2_diff); */
	 if(finger1_diff < tolerance && finger2_diff < tolerance){
		 return false;
	 }
	return true; 
}

/*method to check if the desired object is no longer on the table
 * use point cloud 
 * Arm might be in the way of the sensor, in which case idk????
 */ 
bool not_on_table(ros::NodeHandle n){
	/*Find coordinates of original cloud
	 *if the cloud is not at these coordinates, it's moved
	 */ 
	segbot_arm_perception::TabletopPerception::Response new_scene = segbot_arm_manipulation::getTabletopScene(n);
	std::vector<PointCloudT::Ptr > new_objects;
	
	//get new objects
	new_objects.clear();
	for (unsigned int i = 0; i < new_scene.cloud_clusters.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(new_scene.cloud_clusters.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		new_objects.push_back(object_i);
	}
	//NEEDS MORE, will this work if it detects the object in the air as "on the table"
	//this doesn't guarantee you've picked up the correct object, just an object that
	//looks like the desired one (there could be multiple 
	
	
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_ros,pcl_pc2);
    PointCloudT::Ptr cloud_target(new PointCloudT);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_target);
	
	//should try to add approximate location if possible 
	for(unsigned int i = 0; i < new_objects.size(); i++){
		//int current_size = (int) new_objects.at(i) -> points.size(); //total amount of points 
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputSource(new_objects.at(i)); 
		icp.setInputTarget(cloud_target); // needs to be pcl
		PointCloudT Final;
		icp.align(Final);
		if(icp.hasConverged()){
			ROS_INFO_STREAM("it converged");
			ROS_INFO_STREAM("the fitness score is:");
			ROS_INFO_STREAM(icp.getFitnessScore());
			return false; //an object on the table is the same as the desired
		}
	}
	
	//we've gone through all of the new objects and none of them match the desired object
	return true; 
	
}

/*method to check if the desired object has moved with the arm
 * this requires we know what the desired object looks like
 * 
 * possibly use the feature cloud in the PCL library? 
 * 
 * return: true if the object is now where the arm is, 
 * false otherwise
 * 
 * potential problems: arm blocking the view of the camera.
 * how can we fix this?
 * 		possibly switch to next camera? assuming that can see where the arm is
 */ 
bool moved_with_arm(){
	//get coordinates of the arm??
	//somehow get the point cloud of the object 
	//do icp
	
	
	return false; //dummy
}


/*Method to verify if the arm has picked up the object
 * return: false if the arm failed in picking up the object, true if the arm succeed.
 * called before setting object back down. If grasp failed, try next grasp?
 * 
 * Method assumes robot has only two fingers
 * 
 * If the grasp has successfully happened, the fingers should be slightly open,
 * the table should no longer have the item on it, the item should now be in the air
 * with the arm
 */ 

bool verify_pickup(){
	
	 
	 /*TO DO LIST:
	  * Update tolerance appropriately after testing
	  * 
	  * in a separate method, have the arm try the next possibility if the arm failed
	  * 
	  * maybe use //moveToPoseMoveIt(n,pose_outofview); or similar!! to make sure hand doesn't interfere in third method
	  * 
	  * add to cmake list..........
	  * 
	  * turn arm on, see what the home and pose_outofview do on the robot, choose which is appropriate
	  * 
	  * finish other two methods 
	  */
	  
	 /*Added:
	  * added #include <stdlib.h>
	  * added #include <pcl/io/pcd_io.h>
		#include <pcl/point_types.h>
		#include <pcl/registration/icp.h>
		* 
		* added 	cloud_target -> pc_target; in the main method
	  */ 
	 
	 bool fingers_still_open = fingers_open();
	 
	 

	return false; //dummy, needs to be updated
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "grasp_and_verify");
	
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
	
	
	
	ROS_INFO("Demo starting...move the arm to a position where it is not occluding the table.");
	pressEnter();
	
	
	//store out of table joint position
	listenForArmData(30.0);
	joint_state_outofview = current_state;
	pose_outofview = current_pose;
	

	segbot_arm_manipulation::openHand();
	
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
	
	
	//step 2: extract the data from the response
	detected_objects.clear();
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(table_scene.cloud_clusters.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		detected_objects.push_back(object_i);
	}
	
	if (detected_objects.size() == 0){
		ROS_WARN("[agile_grasp_demo.cpp] No objects detected...aborting.");
		return 1;
	}
	
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=table_scene.cloud_plane_coef[i];
	
	//step 3: select which object to grasp
	int selected_object = selectObjectToGrasp(detected_objects);
	
	//publish object to find grasp topic
	pcl::PCLPointCloud2 pc_target;
	pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
	pcl_conversions::fromPCL(pc_target, cloud_ros);

	
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
	ROS_INFO("Waiting for transform...");
	listener.waitForTransform(cloud_ros.header.frame_id, "mico_link_base", ros::Time(0), ros::Duration(5.0));
	
	std::vector<GraspCartesianCommand> grasp_commands;
	std::vector<geometry_msgs::PoseStamped> poses;
	for (unsigned int i = 0; i < current_grasps.grasps.size(); i++){
		geometry_msgs::PoseStamped p_grasp_i = graspToPose(current_grasps.grasps.at(i),hand_offset_grasp,cloud_ros.header.frame_id);
		geometry_msgs::PoseStamped p_approach_i = graspToPose(current_grasps.grasps.at(i),hand_offset_approach,cloud_ros.header.frame_id);
		
	
		GraspCartesianCommand gc_i;
		gc_i.approach_pose = p_approach_i;
		gc_i.grasp_pose = p_grasp_i;
		
		if (acceptGrasp(gc_i,detected_objects.at(selected_object),plane_coef_vector)){
			
			listener.transformPose("mico_api_origin", gc_i.approach_pose, gc_i.approach_pose);
			listener.transformPose("mico_api_origin", gc_i.grasp_pose, gc_i.grasp_pose);
			
			//filter two -- if IK fails
			moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,gc_i.approach_pose);
			if (ik_response_approach.error_code.val == 1){
				moveit_msgs::GetPositionIK::Response  ik_response_grasp = computeIK(n,gc_i.grasp_pose);
		
				if (ik_response_grasp.error_code.val == 1){
					
					
					//now check to see how close the two sets of joint angles are
					std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(ik_response_approach.solution.joint_state, ik_response_grasp.solution.joint_state );
					
					double sum_d = 0;
					for (int p = 0; p < D.size(); p++){
						sum_d += D[p];
					}
				
					
					if (sum_d < ANGULAR_DIFF_THRESHOLD){
						ROS_INFO("Angle diffs for grasp %i: %f, %f, %f, %f, %f, %f",(int)grasp_commands.size(),D[0],D[1],D[2],D[3],D[4],D[5]);
						
						ROS_INFO("Sum diff: %f",sum_d);
					
					
						//store the IK results
						gc_i.approach_q = ik_response_approach.solution.joint_state;
						gc_i.grasp_q = ik_response_grasp.solution.joint_state;
						
						grasp_commands.push_back(gc_i);
						poses.push_back(p_grasp_i);
						poses_msg.poses.push_back(gc_i.approach_pose.pose);
					}
				}
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
	else {
		ROS_INFO("Picking grasp %i...",min_diff_index);
	}

	pose_array_pub.publish(poses_msg);


	//publish individual pose
	pose_pub.publish(grasp_commands.at(min_diff_index).approach_pose);
	ROS_INFO_STREAM(grasp_commands.at(min_diff_index).approach_q);
	pressEnter();
	
	//set obstacle avoidance
	/*std::vector<sensor_msgs::PointCloud2> obstacle_clouds;
	obstacle_clouds.push_back(cloud_ros);
	segbot_arm_manipulation::setArmObstacles(n,obstacle_clouds);*/
	
	
	segbot_arm_manipulation::moveToPoseMoveIt(n,grasp_commands.at(min_diff_index).approach_pose);
	
	
	//clear obstacles for final approach
	/*obstacle_clouds.clear();
	obstacle_clouds.push_back(table_scene.cloud_plane);
	segbot_arm_manipulation::setArmObstacles(n,obstacle_clouds);*/
	
	segbot_arm_manipulation::moveToPoseMoveIt(n,grasp_commands.at(min_diff_index).approach_pose);
	
	
	segbot_arm_manipulation::moveToPoseMoveIt(n,grasp_commands.at(min_diff_index).grasp_pose);
	spinSleep(3.0);
	
	
	segbot_arm_manipulation::closeHand();
	

	//LIFT action
	
	//we need to call angular velocity control before we can do cartesian control right after using the fingers
	lift(n,0.065); 
	spinSleep(0.5);
	//this is where we should check for verification
	bool verified = fingers_open();
	ROS_INFO_STREAM("The status of the object is ");
	ROS_INFO_STREAM(verified);
	
	ROS_INFO_STREAM("size of the desired cloud is:");
	ROS_INFO_STREAM((int)cloud_ros.data.size());
	bool table_verified = not_on_table(n);
	lift(n,-0.05); 
	segbot_arm_manipulation::openHand();
	
	
	
	//MOVE BACK
	lift(n,0.05); 
	

	segbot_arm_manipulation::homeArm(n);

	segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
	segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
	
	
	
	return 0;
}
