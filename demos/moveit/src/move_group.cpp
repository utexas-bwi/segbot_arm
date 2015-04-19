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
	// BEGIN_TUTORIAL
	//
	// Setup
	// ^^^^^
	//
	// The :move_group_interface:`MoveGroup` class can be easily
	// setup using just the name
	// of the group you would like to control and plan for.
	moveit::planning_interface::MoveGroup group("arm"); //this is the specific group name you'd like to move
	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
	//create a target goal pose
	

	//object placement
	moveit_msgs::CollisionObject collision_object;
	//collision_object.header.frame_id = group.getPlanningFrame();
	collision_object.header.frame_id = "mico_link_base";	
	collision_object.id = "box1";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;
	primitive.dimensions[1] = 0.2;
	primitive.dimensions[2] = 1.0;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0.1;
	box_pose.position.y =  0.3;
	box_pose.position.z =  0.0;
	
	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;
	//second box
	moveit_msgs::CollisionObject collision_object2;
	collision_object2.header.frame_id = "mico_link_base";	
	collision_object2.id = "box2";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive2;
	primitive2.type = primitive2.BOX;
	primitive2.dimensions.resize(3);
	primitive2.dimensions[0] = 0.1;
	primitive2.dimensions[1] = 0.2;
	primitive2.dimensions[2] = 1.0;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose2;
	box_pose2.orientation.w = 1.0;
	box_pose2.position.x =  0.5;
	box_pose2.position.y =  0.3;
	box_pose2.position.z =  0.0;
	
	collision_object2.primitives.push_back(primitive2);
	collision_object2.primitive_poses.push_back(box_pose2);
	collision_object2.operation = collision_object2.ADD;
	
	//finally, add the collision objects to planning environment
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	//collision_objects.push_back(collision_object2);

	sleep(1.0);
	char in;
	std::cout << "Enter 1 for a joint command, 2 for an EF command: ";
	std::cin >> in;
	planning_scene_interface.addCollisionObjects(collision_objects);
	while(in != 'q' && !g_caught_sigint){
		planning_scene_interface.addCollisionObjects(collision_objects);
		group.setPlanningTime(5.0); //10 second maximum for collision computation
		if( in == '1'){
			std::vector<double> q_vals;
			std::cout << "Please specify 6 joint angles for a custom plan. Alternatively, enter the number of a preset." << std::endl;
			for(int i = 1; i < 7; i++){
				double in_q;
				std::cout << std::endl << "Angle " << i << ": ";
				std::cin >> in_q;
				if(!std::cin.good()){
					std::cout << "Invalid input, please try again." << std::endl;
					i -= 1;
					std::cin.clear();
				}
				else
					q_vals.push_back(in_q);
				std::cin.clear();
				}

			group.setJointValueTarget(q_vals);
		}
		else if( in == '2'){
			double x,y,z;
			std::cout << "X: "; std::cin >> x;
			std::cout << "Y: "; std::cin >> y;
			std::cout << "Z: "; std::cin >> z;
			geometry_msgs::Pose target_pose1;
			ros::spinOnce();
			target_pose1.orientation = q;
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
		}
		
		
		//bool success = group.plan(my_plan);
		// Now, we call the planner to compute the plan
		// and visualize it.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		
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
	}
	ros::shutdown();
	return 0;
	}
