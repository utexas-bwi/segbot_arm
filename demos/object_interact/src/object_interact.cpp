#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//used for assignment of vector
#include <boost/assign/std/vector.hpp>
//services
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
using namespace boost::assign;

bool g_caught_sigint = false;
geometry_msgs::Quaternion q;
void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};
void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	q = msg.pose.orientation;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	//button position publisher
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("target_trajectory/pose", 10);
	//make controller service
	ros::ServiceClient client = node_handle.serviceClient<moveit_utils::MicoController>("mico_controller");
	std::cout << "Please ensure that demo.launch has been run!" << std::endl;
	//subscribers
	ros::Subscriber sub_tool = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	moveit::planning_interface::MoveGroup group("arm"); //this is the specific group name you'd like to move

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	// Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
	//create a target goal pose
	
	char in;
	std::cout << "Enter 1 for a joint command, 2 for an EF command: ";
	std::cin >> in;
	group.setPlanningTime(5.0); //5 second maximum for collision computation
	double x,y,z;
	x = 0.381092965603;
	y = 0.0293015688658;
	z = -0.0460119433701;
	
	geometry_msgs::Pose target_pose1;
	
	target_pose1.orientation.x = 0.460225153532;
	target_pose1.orientation.y = 0.548914035582;
	target_pose1.orientation.z = 0.549409203369;
	target_pose1.orientation.w = 0.430157781344;
	target_pose1.position.x = x;
	target_pose1.position.y = y;
	target_pose1.position.z = z;
	
	group.setPoseTarget(target_pose1);
	group.setStartState(*group.getCurrentState());
	
	//publish pose
	geometry_msgs::PoseStamped stampOut;
	stampOut.header.frame_id = "/root_link";
	stampOut.pose = target_pose1;
	stampOut.pose.orientation = target_pose1.orientation;
	
	pose_pub.publish(stampOut);
		
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	if(success){
		char play;
		std::cout << "Enter 1 to play the trajectory" << std::endl;
		std::cin >> play;
		if(play == '1'){
			//call service
			moveit_utils::MicoController srv;
			srv.request.trajectory = my_plan.trajectory_;
			if(client.call(srv)){
				ROS_INFO("Service call sent. Beware!");
			}
			else {
				ROS_INFO("Service call failed. Is the service running?");
			}
		}
		else{
			std::cout << "Not playing trajectory." << std::endl;
		}
	}
	std::cout << "Enter 'q' to quit, 1 to send a joint pose goal, 2 to send cart. goal: ";
	std::cin >> in;
	std::cout<< std::endl;
	ros::shutdown();
	return 0;
	}
