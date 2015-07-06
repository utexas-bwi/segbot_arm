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
#include "geometry_msgs/Pose.h"

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace boost::assign;
bool g_caught_sigint = false;

geometry_msgs::Pose pose;
void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};
void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	pose = msg.pose;
}
int main(int argc, char **argv)
{
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
	
	group.setPlanningTime(5.0); //10 second maximum for collision computation
	group.setNumPlanningAttempts(10);
		
	char in;
	int count = 0;
	std::cout << "For each trajectory, 2 key presses are required. When prompted, move the arm to the desired end position and press the first instructed key." << std::endl;
	std::cout << "Then, move the arm to the desired start position, and key press once more. You will then be prompted for the trajectory name. Usefull names are descriptive!" << std::endl;
	while(in != 'q'){
		std::cout << "MAIN MENU: q for quit, 1 for record, 2 for playback" << std::endl;		
		std::cin >> in;
		if(in == '1'){
			group.clearPoseTargets();
			for(int i = 0; i < 2; i++){
				if(i == 1){
					ROS_INFO("Please move the arm to the desired start position and enter '1' when ready to capture.");
					std::cin >> in;
					group.setStartState(*group.getCurrentState());
				}
				else if(i == 0){			
					ROS_INFO("Please move the arm to the desired end position and enter '1' when ready to capture.");
					std::cin >> in;
					ros::spinOnce();
					//group.setApproximateJointValueTarget(pose, "mico_end_effector");
					//group.setPoseTarget(pose.pose, "mico_link_hand");
					group.setPositionTarget(pose.position.x,pose.position.y,pose.position.z,"mico_end_effector");
					//group.setRPYTarget(0,0,0,"mico_end_effector");
					group.setOrientationTarget(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, "mico_end_effector");		
				}
				if(i == 1){
					moveit::planning_interface::MoveGroup::Plan my_plan;
					bool success = group.plan(my_plan);
					ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
					if(success){
						std::string filename;
						std::cout << "Please enter a filename for the trajectory: ";
						std::cin >> filename;
						rosbag::Bag bag;
						std::string path = ros::package::getPath("moveit_utils");
						bag.open(path + "/trajectories/" + filename + ".bag", rosbag::bagmode::Write);
						bag.write("moveitTrajectory", ros::Time::now(), my_plan.trajectory_);
						bag.close();
						ROS_INFO("Trajectory bag written successfully in the %s/trajectories/ directory", path.c_str());
					}
				}
			}
		}
		else if(in == '2'){
			rosbag::Bag bag;
			std::string path = ros::package::getPath("moveit_utils");
			std::string filename;
			std::cout << "Please enter a filename for the trajectory: ";
			std::cin >> filename;

			bag.open(path + "/trajectories/" + filename + ".bag", rosbag::bagmode::Read);
			
			rosbag::View view(bag, rosbag::TopicQuery("moveitTrajectory"));
			moveit_msgs::RobotTrajectory fromFile;
			BOOST_FOREACH(rosbag::MessageInstance const m, view){
				moveit_msgs::RobotTrajectory::ConstPtr traj = m.instantiate<moveit_msgs::RobotTrajectory>();
				if (traj != NULL){
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
			}	
			bag.close();
			
		}
	}
	ros::shutdown();
	return 0;
}
