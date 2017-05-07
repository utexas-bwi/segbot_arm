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
#include <jaco_msgs/JointVelocity.h>
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

ros::Publisher pub_angular_velocity;
ros::Publisher pub_cartesian_velocity;
ros::ServiceClient controller_client;
actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> *ac;

std::vector<double> angles_current;
std::vector<double> start_angles;
boost::variate_generator< boost::mt19937, boost::normal_distribution<> > *dist;
const char *trajectory_names[] = {"side_grasp_from_home", "side_grasp_from_home", "side_grasp_from_home"};
void sig_handler(int sig){
	ac->cancelAllGoals();
	delete ac;
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};

void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	pose_current = msg;
}

void joint_state_cb(const sensor_msgs::JointState &input){
	if (input.position.at(0) == input.position.at(1) && input.position.at(1) == input.position.at(2)) return;
	angles_current.clear();
	
	for(int i = 0; i < 6; i++){
		angles_current.push_back(input.position.at(i));
	}
}

bool move_to(std::vector<double> angles, double timeout, double speed_limit) {
	ROS_INFO("Angle %f % f %f %f %f %f", angles_current[0], angles_current[1], angles_current[2], angles_current[3], angles_current[4], angles_current[5]);
	jaco_msgs::ArmJointAnglesGoal goal;
	goal.angles.joint1 = angles[0];
	goal.angles.joint2 = angles[1];
	goal.angles.joint3 = angles[2];
	goal.angles.joint4 = angles[3];
	goal.angles.joint5 = angles[4];
	goal.angles.joint6 = angles[5];

	goal.speedLimitationActive = true;
	goal.speedParam1 = speed_limit;
	goal.speedParam2 = speed_limit;
	
	
	ROS_INFO("Goal %f % f %f %f %f %f", goal.angles.joint1, goal.angles.joint2, goal.angles.joint3, goal.angles.joint4, goal.angles.joint5, goal.angles.joint6);
	ac->sendGoal(goal);
	bool timedOut = ac->waitForResult(ros::Duration(timeout));
	if (timedOut) {
		ac->cancelAllGoals();
	}
}

bool move_with_velocities(geometry_msgs::TwistStamped velocity_msg, double duration){
	double elapsed_time = 0.0;
	
	double pub_rate = 40.0; //we publish at 40 hz
	ros::Rate r(pub_rate);
	
	while (ros::ok()){
		//collect messages
		ros::spinOnce();
		
		//publish velocity message
		pub_cartesian_velocity.publish(velocity_msg);
		
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		
		if (elapsed_time > duration)
			break;
	}
}

bool fidget_observantly(std::vector<double> mean_orientation) {

    //construct message
	geometry_msgs::TwistStamped velocity_msg;
	velocity_msg.twist.angular.x = (*dist)();
	velocity_msg.twist.angular.y = (*dist)();
	velocity_msg.twist.angular.z = (*dist)();
    move_with_velocities(velocity_msg, 0.5);
    
}


void load_and_execute_trajectory(std::string filename) {
	std::string path = ros::package::getPath("moveit_utils");
	rosbag::Bag bag;
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
		if(controller_client.call(srv)){
			ROS_INFO("Service call sent. Expect arm movement");
		}
		else {
			ROS_INFO("Service call failed. Is the service running?");
		}
	
	}	
	bag.close();
}

void spin_effector(bool clockwise, double duration) {
	jaco_msgs::JointVelocity msg;
	double pub_rate = 40.0;
	int velocity = 25;
	double elapsed_time = 0;
	msg.joint6 = velocity * (clockwise ? 1 : -1);
	pub_angular_velocity.publish(msg);	
	ros::Rate r(pub_rate);
	while (ros::ok()) {
		ros::spinOnce();
		pub_angular_velocity.publish(msg);
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		if (elapsed_time > duration) break;
	}
	msg.joint6 = 0;
	pub_angular_velocity.publish(msg);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "move_group_interface");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	signal(SIGINT, sig_handler);

    pub_cartesian_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	pub_angular_velocity = node_handle.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 10);
	// Jaco driver joint commands for fidgeting
	ac = new actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction>("/mico_arm_driver/joint_angles/arm_joint_angles", true);

	//button position publisher
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("target_trajectory/pose", 10);
	
	//make controller service
	controller_client = node_handle.serviceClient<moveit_utils::MicoController>("mico_controller");
	std::cout << "Please ensure that demo.launch has been run!" << std::endl;
	//subscribers
	ros::Subscriber sub_tool = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	ros::Subscriber sub_angles = node_handle.subscribe ("/joint_states", 1, joint_state_cb);

    boost::mt19937 *rng = new boost::mt19937();
	rng->seed(time(NULL));
	boost::normal_distribution<> distribution(0,0.04);
	dist = new boost::variate_generator< boost::mt19937, boost::normal_distribution<> > (*rng, distribution);
    std::cout << "hello";
	char in;
	int count = 0;
	
	ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10));
	start_angles = angles_current;
	spin_effector(true, 1.0);
	move_to(start_angles, 40, 40);
	while(true){
		std::cout << "MAIN MENU: q for quit, 1 for experiment, 2 for home" << std::endl;		
		std::cin >> in;
		if (in == 'q') break;
		else if (in == '2') { 
			move_to(start_angles, 40, 40);
			continue;
		}

		std::cout << "- Motion: 1 for none, 2 for observant fidgeting" << std::endl;
		char motion;
		char amount;
		std::cin >> motion;
		double idle_time = 45;
        
        std::cout << " - Object: 1, 2, 3" << std::endl;
        char object;
        std::cin >> object;
		int object_i = object - '0' - 1;
		switch (motion){
			case '1':
				ROS_INFO("Beginning dumb idle activity");
				ros::Duration(idle_time).sleep();
				break;
			case '2':{
				ROS_INFO("Beginning fidget");
				time_t end = time(NULL) + idle_time;
				while (time(NULL) <= end){
					fidget_observantly(start_angles);
				}
				break;
            }

		}
        move_to(start_angles, 40, 35);

        
        load_and_execute_trajectory(trajectory_names[object_i]);
        
			
	}
	ros::shutdown();
	return 0;
}
