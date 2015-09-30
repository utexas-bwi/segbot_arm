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

//the action definition
#include "segbot_arm_manipulation/TabletopGraspAction.h"

#include <actionlib/server/simple_action_server.h>


#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm


//some defines related to filtering candidate grasps
#define MAX_DISTANCE_TO_PLANE 0.05


//the individual action server
//#include "segbot_arm_manipulation/tabletop_grasp_action.h"

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


struct GraspCartesianCommand {
	sensor_msgs::JointState approach_q;
	geometry_msgs::PoseStamped approach_pose;
	
	sensor_msgs::JointState grasp_q;
	geometry_msgs::PoseStamped grasp_pose;
	
	
};


class TabletopGraspActionServer
{
protected:
	
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<segbot_arm_manipulation::TabletopGraspAction> as_; 
	std::string action_name_;
	// create messages that are used to published feedback/result
	segbot_arm_manipulation::TabletopGraspFeedback feedback_;
	segbot_arm_manipulation::TabletopGraspResult result_;
  
	
	sensor_msgs::JointState current_state;
	sensor_msgs::JointState current_effort;
	jaco_msgs::FingerPosition current_finger;
	geometry_msgs::PoseStamped current_pose;
	bool heardPose;
	bool heardJoinstState; 
  
	
	geometry_msgs::PoseStamped current_moveit_pose;

	
	bool heardGrasps;
	agile_grasp::Grasps current_grasps;

	ros::Publisher pub_velocity;
	ros::Publisher cloud_pub;
	ros::Publisher cloud_grasp_pub;
	ros::Publisher pose_array_pub;
	ros::Publisher pose_pub;
	ros::Publisher pose_fk_pub;

	std::vector<PointCloudT::Ptr > detected_objects;
	
public:

  TabletopGraspActionServer(std::string name) :
    as_(nh_, name, boost::bind(&TabletopGraspActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	heardPose = false;
	heardJoinstState = false;
	heardGrasps = false;
	//create subscriber to joint angles
	ros::Subscriber sub_angles = nh_.subscribe ("/joint_states", 1, &TabletopGraspActionServer::joint_state_cb, this);

	//create subscriber to joint torques
	ros::Subscriber sub_torques = nh_.subscribe ("/mico_arm_driver/out/joint_efforts", 1, &TabletopGraspActionServer::joint_effort_cb,this);

	//create subscriber to tool position topic
	ros::Subscriber sub_tool = nh_.subscribe("/mico_arm_driver/out/tool_position", 1, &TabletopGraspActionServer::toolpos_cb, this);

	//subscriber for fingers
	ros::Subscriber sub_finger = nh_.subscribe("/mico_arm_driver/out/finger_position", 1, &TabletopGraspActionServer::fingers_cb, this);
	  
	//subscriber for grasps
	ros::Subscriber sub_grasps = nh_.subscribe("/find_grasps/grasps_handles",1, &TabletopGraspActionServer::grasps_cb,this);  
	  

	//publish velocities
	pub_velocity = nh_.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	//publish pose array
	pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("/agile_grasp_demo/pose_array", 10);
	
	//publish pose 
	pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);
	pose_fk_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_fk_out", 10);
	
	//debugging publisher
	cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
	cloud_grasp_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
	
    
    as_.start();
  }

  ~TabletopGraspActionServer(void)
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
	
	// Range = [6, 7300] ([open, close])
	void moveFingers(int finger_value) {
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
	
	void moveToPose(ros::NodeHandle n, geometry_msgs::PoseStamped p_target){
		moveit_utils::MicoMoveitCartesianPose::Request 	req;
		moveit_utils::MicoMoveitCartesianPose::Response res;
		
		req.target = p_target;
		
		ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose> ("/mico_cartesianpose_service");
		if(client.call(req, res)){
			ROS_INFO("Call successful. Response:");
			ROS_INFO_STREAM(res);
		} else {
			ROS_ERROR("Call failed. Terminating.");
		}
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
	
	void executeCB(const segbot_arm_manipulation::TabletopGraspGoalConstPtr &goal)
	{
		//declare some variables
		sensor_msgs::PointCloud2 cloud_ros;

		//the result
		segbot_arm_manipulation::TabletopGraspResult result;

    
		//used to compute transforms
		tf::TransformListener listener;
	
		//get the data out of the goal
		Eigen::Vector4f plane_coef_vector;
		for (int i = 0; i < 4; i ++)
			plane_coef_vector(i)=goal->cloud_plane_coef[i];
		int selected_object = goal->target_object_cluster_index;
		
		PointCloudT::Ptr cloud_plane (new PointCloudT);

		detected_objects.clear();
		for (unsigned int i = 0; i < goal->cloud_clusters.size(); i++){
			PointCloudT::Ptr object_i (new PointCloudT);
			pcl::PCLPointCloud2 pc_i;
			pcl_conversions::toPCL(goal->cloud_clusters.at(i),pc_i);
			pcl::fromPCLPointCloud2(pc_i,*object_i);
			detected_objects.push_back(object_i);
		}
		
		ROS_INFO("[segbot_tabletop_grasp_as.cpp] Publishing point cloud...");
		cloud_grasp_pub.publish(goal->cloud_clusters.at(goal->target_object_cluster_index));
		
		//wait for response at 5 Hz
		listenForGrasps(5.0);
		
		ROS_INFO("[segbot_tabletop_grasp_as.cpp] Heard %i grasps",(int)current_grasps.grasps.size());
	
		//next, compute approach and grasp poses for each detected grasp
		double hand_offset_grasp = -0.02;
		double hand_offset_approach = -0.13;
		
		//wait for transform from visual space to arm space
		listener.waitForTransform(goal->cloud_clusters.at(goal->target_object_cluster_index).header.frame_id, "mico_link_base", ros::Time(0), ros::Duration(3.0));
		
		geometry_msgs::PoseArray poses_msg;
		poses_msg.header.seq = 1;
		poses_msg.header.stamp = cloud_ros.header.stamp;
		poses_msg.header.frame_id = "mico_api_origin";
		
		
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
				moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(nh_,gc_i.approach_pose);
				moveit_msgs::GetPositionIK::Response  ik_response_grasp = computeIK(nh_,gc_i.grasp_pose);
		
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
		updateFK(nh_);
		
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
			
			as_.setPreempted(result);
            return;
		}

		pose_array_pub.publish(poses_msg);


		//publish individual pose
		pose_pub.publish(grasp_commands.at(min_diff_index).approach_pose);
		moveToPose(nh_,grasp_commands.at(min_diff_index).approach_pose);
		
		
		//open fingers
		//pressEnter();
		//moveFinger(100);


		sleep(1.0);
		pose_pub.publish(grasp_commands.at(min_diff_index).grasp_pose);
		/*std::cout << "Press '1' to move to approach pose or Ctrl-z to quit..." << std::endl;		
		std::cin >> in;*/
		
		moveToPose(nh_,grasp_commands.at(min_diff_index).grasp_pose);
		spinSleep(3.0);
		
		//listenForArmData(30.0);
		//close fingers
		//pressEnter();
		moveFingers(7200);
	}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  TabletopGraspActionServer fibonacci(ros::this_node::getName());
  ros::spin();

  return 0;
}


/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "segbot_arm_manipulation_node");
    ros::NodeHandle nh("~");
    boost::recursive_mutex api_mutex;

    bool is_first_init = true;
    while (ros::ok())
    {
        try
        {
			TabletopGraspServer grasp_server(nh);
			
            ros::spin();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
            ros::Duration(1.0).sleep();
        }

        is_first_init = false;
    }
    return 0;
}*/
