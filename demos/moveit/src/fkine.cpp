#include <ros/ros.h>
#include <signal.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <jaco_msgs/JointAngles.h>

#include <sensor_msgs/JointState.h>

bool g_caught_sigint = false;

geometry_msgs::PoseStamped pose_current;
sensor_msgs::JointState js_cur;

std::vector<double> angles_current;
void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};

void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	pose_current = msg;
}
//Joint state cb
void joint_state_cb(const sensor_msgs::JointState &input){
	angles_current.clear();
	for(int i = 0; i < 6; i++){
		angles_current.push_back(input.position.at(i));
	}
	js_cur = input;
}


int main(int argc, char **argv)
{
	ros::init (argc, argv, "mico_kinematic_demo");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;
	signal(SIGINT, sig_handler);

	// Start a service client
	ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
	ros::Publisher robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 1);
	
	//publish FK pose
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("fkine/pose", 10);
	
	//make controller service
	//ros::ServiceClient client = node_handle.serviceClient<moveit_utils::MicoController>("mico_controller");

	ros::Subscriber sub_tool = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	ros::Subscriber sub_angles = node_handle.subscribe ("/joint_states", 1, joint_state_cb);

	while(!service_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}

	/*
	 * Forward Kinematic service calls
	 */

	moveit_msgs::GetPositionFK::Request service_request;
	moveit_msgs::GetPositionFK::Response service_response;


	ROS_INFO("Grabbing current joint state for comparison.");
	ros::spinOnce();
	sensor_msgs::JointState q_true = js_cur;

	//Load request with the desired link
	service_request.fk_link_names.push_back("mico_end_effector");

	//and the current frame
	service_request.header.frame_id = "mico_link_base";

	//finally we let moveit know what joint positions we want to compute
	//in this case, the current state
	service_request.robot_state.joint_state = q_true;

	ROS_INFO("Making FK call");
 	if(service_client.call(service_request, service_response))
 			pose_pub.publish(service_response.pose_stamped.at(0));

	ros::shutdown();
	return 0;
}
