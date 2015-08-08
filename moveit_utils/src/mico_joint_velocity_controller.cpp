#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>

//messages and types
#include "jaco_msgs/JointAngles.h"
#include "jaco_msgs/JointVelocity.h"
#include <sensor_msgs/JointState.h>
#include "moveit_utils/MicoController.h" //depending on needs, may need to create new srv

#define PI 3.14159265
#define RAD_TO_DEG 57.2957795

#define TOLERANCE_RADIANS (0.01125*PI)

ros::Publisher j_vel_pub;
bool g_caught_sigint=false;
sensor_msgs::JointState js_cur;
float tol = 0.05; //Should get from mico launch / param server
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


jaco_msgs::JointVelocity toJacoJointVelocityMsg(std::vector<double> goal_vector){
	jaco_msgs::JointVelocity jv_goal;
	
	jv_goal.joint1 = -RAD_TO_DEG*goal_vector[0];
	jv_goal.joint2 = RAD_TO_DEG*goal_vector[1];
	jv_goal.joint3 = -RAD_TO_DEG*goal_vector[2];
	jv_goal.joint4 = -RAD_TO_DEG*goal_vector[3];
	jv_goal.joint5 = -RAD_TO_DEG*goal_vector[4];
	jv_goal.joint6 = -RAD_TO_DEG*goal_vector[5];
	
	return jv_goal;
}


// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

bool service_cb(moveit_utils::MicoController::Request &req, moveit_utils::MicoController::Response &res){

	ROS_INFO("Mico controller service called:");
	ROS_INFO_STREAM(req.trajectory.joint_trajectory);
	
	
	trajectory_msgs::JointTrajectory trajectory = req.trajectory.joint_trajectory;
    jaco_msgs::JointVelocity jv_goal;
	bool next_point = false;
	
	ros::Rate r(40);
	
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
		ROS_INFO("Current joint state:");
		ROS_INFO_STREAM(js_cur);
		ROS_INFO("Trajectroy:");
		ROS_INFO_STREAM(trajectory);
		for(int i = 1; i < trajectory_length - 1; i++){
			bool done = false;
			//set the target velocity
			ros::spinOnce();
			
			jv_goal = toJacoJointVelocityMsg(trajectory.points.at(i).velocities);
			
			/*jv_goal.joint1 = -180/PI*trajectory.points.at(i).velocities.at(0);
			jv_goal.joint2 = 180/PI*trajectory.points.at(i).velocities.at(1);
			jv_goal.joint3 = -180/PI*trajectory.points.at(i).velocities.at(2);
			jv_goal.joint4 = -180/PI*trajectory.points.at(i).velocities.at(3);
			jv_goal.joint5 = -180/PI*trajectory.points.at(i).velocities.at(4);
			jv_goal.joint6 = -180/PI*trajectory.points.at(i).velocities.at(5);*/
			
			ROS_INFO("Target joint state:");
			ROS_INFO_STREAM(trajectory.points[i]);
			ROS_INFO("Current joint state:");
			ROS_INFO_STREAM(js_cur);
			
			ROS_INFO("Joint velocity goal for point %i:",i);
			ROS_INFO_STREAM(jv_goal);
			
			pressEnter();
			
			tfs = trajectory.points.at(i).time_from_start;
			last_sent = ros::Time::now().toSec();
			first_sent = ros::Time::now();
			j_vel_pub.publish(jv_goal);

			//check if velocity should re-up or be canceled
			//current implementation: assume that the traj velocities will take each joint to correct point
			//written but not invoked is to check the target and goal pos of the joints, and preempting 
			//further movement when required.
			
			/*std::vector<double> distances;
			std::vector<double> starting;
			std::vector<int> directions;
			ros::spinOnce();
			
			for(int j = 0; j < 6; j++){
				if(trajectory.points.at(i).velocities.at(j) < 0)
					directions.push_back(-1);
				else
					directions.push_back(1);
				distances.push_back(abs(trajectory.points.at(i).positions.at(j) - js_cur.position.at(j)));
				starting.push_back(js_cur.position.at(j));

			}*/
			//std::vector<char> zeros;
			
			if(false){
				if(i == 0 || i == trajectory_length - 1)
					done = true;
				
				//while we haven't reached the next point, do
				while(!done){
					
					double sum = 0.0;
					//check if any of the joints have reached their targets
					std::vector<double> jv_goal_array;
					jv_goal_array.resize(6);
					for (unsigned int j = 0; j < 6; j ++){
						if (fabs(js_cur.position[j]- trajectory.points[i].positions[j]) < TOLERANCE_RADIANS){	
							jv_goal_array[j]=0.0;
							//ROS_INFO("Joint %i has reached it's target.",j);
						}
						else {
							jv_goal_array[j]=trajectory.points.at(i).velocities[j];
						}	
						sum+=fabs(jv_goal_array[j]);
					}
					
					//ROS_INFO("Sum of joint velocities: %f",sum);
					
					//check if we're done
					if (sum < 0.01){
						done = true;
						ROS_INFO("Advancing to next point.");
					}
					else {
						
						
						jv_goal = toJacoJointVelocityMsg(jv_goal_array);
						
						//ROS_INFO("JV message:");
						//ROS_INFO_STREAM(jv_goal);
						
						
						j_vel_pub.publish(jv_goal);
						ros::spinOnce();
						r.sleep();
						
					}
					
					
					/*for(int j = 0; j < 6; j++){
						
						
						
						//double tolerance = (tol) * (abs(trajectory.points.at(i).positions.at(j) - js_cur.position.at(j))); 
						
						/*double tolerance = 0.1;
						ros::spinOnce();
						double target = trajectory.points.at(i).positions.at(j);
						if(target > 1.5*PI)
							target -= 2*PI;
						if( js_cur.position.at(j) + tolerance >= target &&
							 js_cur.position.at(j) - tolerance <= target){
							switch(j) {
								case 0	: jv_goal.joint1 = 0; zeros.push_back('1'); break; 
								case 1	: jv_goal.joint2 = 0; zeros.push_back('1'); break;
								case 2	: jv_goal.joint3 = 0; zeros.push_back('1'); break;
								case 3	: jv_goal.joint4 = 0; zeros.push_back('1'); break;
								case 4	: jv_goal.joint5 = 0; zeros.push_back('1'); break;
								case 5	: jv_goal.joint6 = 0; zeros.push_back('1'); break;
							}
						}*/
						//ROS_INFO("cur: %f", js_cur.position.at(j));
						//ROS_INFO("Starting: %f", starting.at(j));
						//ROS_INFO("Distance: %f", distances.at(j));
						/*if(abs(js_cur.position.at(j) - starting.at(j)) >= abs(distances.at(j))){
							switch(j) {
								case 0	: jv_goal.joint1 = 0; zeros.push_back('1'); break;
								case 1	: jv_goal.joint2 = 0; zeros.push_back('1'); break;
								case 2	: jv_goal.joint3 = 0; zeros.push_back('1'); break;
								case 3	: jv_goal.joint4 = 0; zeros.push_back('1'); break;
								case 4	: jv_goal.joint5 = 0; zeros.push_back('1'); break;
								case 5	: jv_goal.joint6 = 0; zeros.push_back('1'); break;
							}
						}
						
					}*/
					
				
					
					
					/*ROS_INFO("expecting q1: %f, q2: %f, q3: %f, q4: %f, q5: %f, q6: %f",
					 trajectory.points.at(i).positions.at(0), trajectory.points.at(i).positions.at(1), trajectory.points.at(i).positions.at(2),
					 trajectory.points.at(i).positions.at(3),trajectory.points.at(i).positions.at(4),trajectory.points.at(i).positions.at(5));
					ROS_INFO("at q1: %f, q2: %f, q3: %f, q4: %f, q5: %f, q6: %f", js_cur.position.at(0), js_cur.position.at(1), js_cur.position.at(2)
					,js_cur.position.at(3),js_cur.position.at(4),js_cur.position.at(5));

					ROS_INFO("Vector size: %lu", zeros.size());
					*/
					

				}
			}
			else{
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
		}
		//ROS_INFO("At       pos: %f, %f, %f, %f, %f, %f", js_cur.position.at(0), js_cur.position.at(1), js_cur.position.at(2), js_cur.position.at(3), js_cur.position.at(4), js_cur.position.at(5));

	}
	/*
	* Corrective velocity at very end
	*/
	/*int i = trajectory_length - 1;
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
*/
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
