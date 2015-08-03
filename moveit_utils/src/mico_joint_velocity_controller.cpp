#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>

//messages and types
#include "jaco_msgs/JointAngles.h"
#include "jaco_msgs/JointVelocity.h"
#include <sensor_msgs/JointState.h>
#include "moveit_utils/MicoController.h" //depending on needs, may need to create new srv

#define PI 3.1459
ros::Publisher j_vel_pub;
bool g_caught_sigint=false;
sensor_msgs::JointState js_cur;
float tol = 0.005; //Should get from mico launch / param server
std::vector<float> js_goal;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("Caught sigint, shutting down...");
    ros::shutdown();
    exit(1);
}
void joint_state_cb(const sensor_msgs::JointStateConstPtr& js){
	if (js->position.size() > 4)
		js_cur = *js;
};
float update_velocity(float p_cur, float p_goal, double time_remaining){
	return (float)(p_goal - p_cur)/time_remaining;
};
//update joint velocity based on target and cur position of the actuator.
//required: tolerance set, goal set, time tracked correctly
//returns: updated JointVelocity message for publishing
jaco_msgs::JointVelocity check_state(jaco_msgs::JointVelocity jv_cur, double time_remaining){
	jaco_msgs::JointVelocity jv_update;
	for(int i = 1; i <= 6; i++){
		if((js_cur.position.at(i-1) + tol) >= js_goal.at(i-1) && (js_cur.position.at(i-1) - tol <= js_goal.at(i-1))){
			switch(i) {
				case 1	: jv_update.joint1 = 0; break;
				case 2	: jv_update.joint2 = 0; break;
				case 3	: jv_update.joint3 = 0; break;
				case 4	: jv_update.joint4 = 0; break;
				case 5	: jv_update.joint5 = 0; break;
				case 6	: jv_update.joint6 = 0; break;
			}
		}
		//update velocity
		else{
			switch(i) {
				case 1	: jv_update.joint1 = update_velocity(js_goal.at(0), js_cur.position.at(0), time_remaining); break; 
				case 2	: jv_update.joint1 = update_velocity(js_goal.at(1), js_cur.position.at(1), time_remaining); break;
				case 3	: jv_update.joint1 = update_velocity(js_goal.at(2), js_cur.position.at(2), time_remaining); break;
				case 4	: jv_update.joint1 = update_velocity(js_goal.at(3), js_cur.position.at(3), time_remaining); break;
				case 5	: jv_update.joint1 = update_velocity(js_goal.at(4), js_cur.position.at(4), time_remaining); break;
				case 6	: jv_update.joint1 = update_velocity(js_goal.at(5), js_cur.position.at(5), time_remaining); break;
			}
		}
	}
	return jv_update;
}; 
//fills goal vector with the positions in the last entry in the trajectory vector
void fill_goal(trajectory_msgs::JointTrajectory jt, int length){
	for(int i = 0; i < 6; i++){
		js_goal.push_back(jt.points.at(length - 1).positions.at(i));
	}
}

bool service_cb(moveit_utils::MicoController::Request &req, moveit_utils::MicoController::Response &res){

	ROS_INFO("Mico controller service called:");
	ROS_INFO_STREAM(req.trajectory.joint_trajectory);
	
	
	trajectory_msgs::JointTrajectory trajectory = req.trajectory.joint_trajectory;
    jaco_msgs::JointVelocity jv_goal;
	bool next_point = false;
	
	ros::Rate r(1000);
	
	double last_sent;
	ros::Time first_sent;
	int trajectory_length = trajectory.points.size();
	

	
	if(trajectory_length == 0)
		ROS_INFO("Trajectory message empty. No movement generated.");
	else{
		std::cout << "preparing for " << trajectory_length << " number of trajs." << std::endl;
		js_goal.clear();
		fill_goal(trajectory, trajectory_length);
		ros::Duration last(0.0); //holds the last trajectory's time from start, and the current traj's tfs
		ros::Duration tfs(0.0);
		for(int i = 0; i < trajectory_length; i++){
			//set the target velocity
			ros::spinOnce();
			jv_goal.joint1 = -180/PI*trajectory.points.at(i).velocities.at(0);
			jv_goal.joint2 = 180/PI*trajectory.points.at(i).velocities.at(1);
			jv_goal.joint3 = -180/PI*trajectory.points.at(i).velocities.at(2);
			jv_goal.joint4 = -180/PI*trajectory.points.at(i).velocities.at(3);
			jv_goal.joint5 = -180/PI*trajectory.points.at(i).velocities.at(4);
			jv_goal.joint6 = -180/PI*trajectory.points.at(i).velocities.at(5);
	
			tfs = trajectory.points.at(i).time_from_start;
			last_sent = ros::Time::now().toSec();
			first_sent = ros::Time::now();
			j_vel_pub.publish(jv_goal);

			//check if velocity should re-up or be canceled
			//current implementation: assume that the traj velocities will take each joint to correct point
			//written but not invoked is to check the target and goal pos of the joints, and preempting 
			//further movement when required.
			
			while(!next_point){
				//rather than check conditionally, re-up on the message
				ros::spinOnce();
				j_vel_pub.publish(jv_goal);
				last_sent = ros::Time::now().toSec();
				//ROS_INFO("first_sent: %f tfs: %f", (ros::Time::now() - first_sent).toSec(), (tfs - last).toSec());
				//}
				if(((ros::Time::now() - first_sent).toSec() >= ((1-tol) * (tfs - last).toSec()))){ //movement should be preempted
					ROS_INFO("Expecting pos: %f, %f, %f, %f, %f, %f", trajectory.points.at(i).positions.at(0), trajectory.points.at(i).positions.at(1), trajectory.points.at(i).positions.at(2), trajectory.points.at(i).positions.at(3), trajectory.points.at(i).positions.at(4), trajectory.points.at(i).positions.at(5));
					ROS_INFO("At       pos: %f, %f, %f, %f, %f, %f", js_cur.position.at(0), js_cur.position.at(1), js_cur.position.at(2), js_cur.position.at(3), js_cur.position.at(4), js_cur.position.at(5));
					jaco_msgs::JointVelocity empty_goal;
					j_vel_pub.publish(empty_goal);
					next_point = true;
				}
				r.sleep();
			}
			next_point = false;
			last = trajectory.points.at(i).time_from_start;
		}
		//ROS_INFO("At       pos: %f, %f, %f, %f, %f, %f", js_cur.position.at(0), js_cur.position.at(1), js_cur.position.at(2), js_cur.position.at(3), js_cur.position.at(4), js_cur.position.at(5));

	}
	/*
	* Corrective velocity at very end
	*/
	int i = trajectory_length - 1;
	ros::spinOnce();
	ROS_INFO("Sending corrective velocity message");
	jv_goal.joint1 = -180/PI*(trajectory.points.at(trajectory_length - 1).positions.at(0) - js_cur.position.at(0))*4;
	jv_goal.joint2 = 180/PI*(trajectory.points.at(trajectory_length - 1).positions.at(1) - js_cur.position.at(1))*4;
	jv_goal.joint3 = -180/PI*(trajectory.points.at(trajectory_length - 1).positions.at(2) - js_cur.position.at(2))*4;
	jv_goal.joint4 = -180/PI*(trajectory.points.at(trajectory_length - 1).positions.at(3) - js_cur.position.at(3))*4;
	jv_goal.joint5 = -180/PI*(trajectory.points.at(trajectory_length - 1).positions.at(4) - js_cur.position.at(4))*4;
	jv_goal.joint6 = -180/PI*(trajectory.points.at(trajectory_length - 1).positions.at(5) - js_cur.position.at(5))*4;
	j_vel_pub.publish(jv_goal);
	sleep(1);
	ros::spinOnce();
	ROS_INFO("Expecting pos: %f, %f, %f, %f, %f, %f", trajectory.points.at(i).positions.at(0), trajectory.points.at(i).positions.at(1), trajectory.points.at(i).positions.at(2), trajectory.points.at(i).positions.at(3), trajectory.points.at(i).positions.at(4), trajectory.points.at(i).positions.at(5));
	ROS_INFO("At       pos: %f, %f, %f, %f, %f, %f", js_cur.position.at(0), js_cur.position.at(1), js_cur.position.at(2), js_cur.position.at(3), js_cur.position.at(4), js_cur.position.at(5));

	res.done = true;
	return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_joint_velocity_controller");
    signal(SIGINT, sig_handler);
    ros::NodeHandle n;

    //subscriber for position check
    ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
    //publisher for velocity commands
    j_vel_pub = n.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 1);
    
    ros::ServiceServer srv = n.advertiseService("mico_controller", service_cb);
    ROS_INFO("Mico joint velocity controller server started.");
    
    ros::spin();
    return 0;
}
