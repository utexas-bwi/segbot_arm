#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveGroupActionResult.h>
//used for assignment of vector
#include <boost/assign/std/vector.hpp>
//services
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace boost::assign;
bool writeTrajToFile = true;
bool g_caught_sigint = false;
geometry_msgs::PoseStamped cur;
moveit_msgs::RobotTrajectory robtraj;
bool gotPlan = false;
void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};
void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	cur = msg;
}
void result_cb(const moveit_msgs::MoveGroupActionResult &msg){
	robtraj = msg.result.planned_trajectory;
	gotPlan = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_rviz_mimic_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	signal(SIGINT, sig_handler);

	//button position publisher
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("target_trajectory/pose", 10);
	//make controller service
	ros::ServiceClient client = node_handle.serviceClient<moveit_utils::MicoController>("mico_controller");
	//subscribers
	ros::Subscriber sub_tool = node_handle.subscribe("/move_group/result", 1, result_cb);
	
	char in;
	while(in != 'q'){
		ros::spinOnce();
		if(gotPlan){
			ROS_INFO("Robot trajectory recieved. Enter 1 to play");
			std::cin >> in;
			if(in == '1'){
				//call service
				moveit_utils::MicoController srv;
				srv.request.trajectory = robtraj;
				if(client.call(srv)){
					ROS_INFO("Service call sent. Beware!");
				}
				else {
					ROS_INFO("Service call failed. Is the service running?");
				}
			}
		}
	}
	ros::shutdown();
	return 0;
	}
