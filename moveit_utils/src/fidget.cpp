#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <jaco_msgs/JointAngles.h>
//used for assignment of vector
#include <boost/assign/std/vector.hpp>
//services
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

//actions
#include <actionlib/client/simple_action_client.h>
#include <jaco_msgs/ArmJointAnglesAction.h>

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define foreach BOOST_FOREACH

using namespace boost::assign;
bool g_caught_sigint = false;

geometry_msgs::PoseStamped pose_start;
geometry_msgs::PoseStamped pose_final;
geometry_msgs::PoseStamped pose_current;

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
	if (input.position.at(0) == input.position.at(1) && input.position.at(1) == input.position.at(2)) return;
	angles_current.clear();
	
	for(int i = 0; i < 6; i++){
		angles_current.push_back(input.position.at(i));
	}
}
int main(int argc, char **argv){
	ros::init(argc, argv, "move_group_interface");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	signal(SIGINT, sig_handler);

	// Jaco driver joint commands for fidgeting
	actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);

	//button position publisher
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("target_trajectory/pose", 10);
	
	//make controller service
	ros::ServiceClient client = node_handle.serviceClient<moveit_utils::MicoController>("mico_controller");
	std::cout << "Please ensure that demo.launch has been run!" << std::endl;
	//subscribers
	ros::Subscriber sub_tool = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	ros::Subscriber sub_angles = node_handle.subscribe ("/joint_states", 1, joint_state_cb);

	
	moveit::planning_interface::MoveGroup group("arm"); //this is the specific group name you'd like to move

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	
	group.setPlanningTime(5.0); //10 second maximum for collision computation
	group.setNumPlanningAttempts(10);
	group.allowReplanning(true);
	char in;
	int count = 0;
	
	boost::mt19937 *rng = new boost::mt19937();
	rng->seed(time(NULL));
	boost::normal_distribution<> distribution(0,0.05);
	boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dist(*rng, distribution);

	
	while(in != 'q'){
		std::cout << "MAIN MENU: q for quit, 1 for no fidget, 2 for fidget" << std::endl;		
		std::cin >> in;
		if (in == 'q') break;
		
		
		while (true){
			ROS_INFO("Angle %f % f %f %f %f %f", angles_current[0], angles_current[1], angles_current[2], angles_current[3], angles_current[4], angles_current[5]);
			jaco_msgs::ArmJointAnglesGoal goal;
			goal.angles.joint1 = angles_current[0] + dist();
			goal.angles.joint2 = angles_current[1] + dist();
			goal.angles.joint3 = angles_current[2] + dist();
			goal.angles.joint4 = angles_current[3] + dist();
			goal.angles.joint5 = angles_current[4] + dist();
			goal.angles.joint6 = angles_current[5] + dist() / 10;
			goal.speedLimitationActive = true;
			goal.speedParam1 = 0.05;
			
			ROS_INFO("Goal %f % f %f %f %f %f", goal.angles.joint1, goal.angles.joint2, goal.angles.joint3, goal.angles.joint4, goal.angles.joint5, goal.angles.joint6);
			ac.sendGoal(goal);
			bool timedOut = ac.waitForResult(ros::Duration(20.0));
			if (timedOut) {
				ac.cancelAllGoals();
			}

		}
		

		rosbag::Bag bag;
		std::string path = ros::package::getPath("moveit_utils");
		std::string filename = "side_grasp_from_home";

		bag.open(path + "/trajectories/" + filename + ".bag", rosbag::bagmode::Read);
		
		rosbag::View view(bag, rosbag::TopicQuery("moveitTrajectory"));
		moveit_msgs::RobotTrajectory fromFile;
		BOOST_FOREACH(rosbag::MessageInstance const m, view){
			moveit_msgs::RobotTrajectory::ConstPtr traj = m.instantiate<moveit_msgs::RobotTrajectory>();
			if (traj == NULL){
				ROS_INFO("Couldn't get trajectory from bag");
				continue;
			}

			fromFile = *traj;
			ROS_INFO("Trajectory successfully loaded from bag");
			
			moveit_utils::MicoController srv;
			srv.request.trajectory = fromFile;
			if(client.call(srv)){
				ROS_INFO("Service call sent. Expect arm movement");
			}
			else {
				ROS_INFO("Service call failed. Is the service running?");
			}
			
		}	
		bag.close();
			
		
	}
	ros::shutdown();
	return 0;
}
