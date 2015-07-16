/*
* This ROS action logs perceptual data relating to the arm
* 
* The following series of data are logged into csv(s)
* 	-Joint efforts
* 	-Joint positions
* 	-EF position
* 
* 
* Output is based on the input string for the service request. 
* 
* File naming is as follows: input file string should contain the absolute path to the folder (without extension)
* ie: filePath should be passed in as: ~/someBaseFolder/armPerceps/
* this will then be appended by the name of the file - *test_* (kept constant) and the time of the experiment (and movement) and finally by the extension
* 
* File parsing is as follows: 6 doubles > efforts, 6 doubles > positions, 3 doubles > EF cart position
* 
* Author Maxwell J Svetlik
*/


#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include "segbot_arm_perception/LogPerceptionAction.h"
#include "jaco_msgs/FingerPosition.h"
#include <boost/lexical_cast.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <signal.h>


sensor_msgs::JointState efforts;
sensor_msgs::JointState joint_state;
geometry_msgs::PoseStamped toolpos;
bool start;
bool g_caught_sigint = false;


void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};


class LogPerceptionAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<segbot_arm_perception::LogPerceptionAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  segbot_arm_perception::LogPerceptionFeedback feedback_;
  segbot_arm_perception::LogPerceptionResult result_;
  std::ofstream myfile;
  bool handleMade;
  
public:

  LogPerceptionAction(std::string name) :
    as_(nh_, name, boost::bind(&LogPerceptionAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    handleMade = false;
  }

  ~LogPerceptionAction(void)
  {
  }
  
  bool createHandle(std::string filePath){
	double begin = ros::Time::now().toSec();
	std::string startTime = boost::lexical_cast<std::string>(begin);
	filePath.append("/test_" + startTime + ".csv");
	ROS_INFO("Making %s", filePath.c_str());	
	myfile.open(filePath.c_str());
	//write header
	myfile << "efforts_q1,efforts_q2,efforts_q3,efforts_q4,efforts_q5,efforts_q6,joint_position_q1,joint_position_q2,joint_position_q3,joint_position_q4,joint_position_q5,joint_position_q6,tool_position_x,tool_position_y,tool_position_z\n";
	return true;
  }

  void executeCB(const segbot_arm_perception::LogPerceptionGoalConstPtr &goal){
    // helper variables
    ros::Rate r(15);
    bool success = true;

	if(goal->start == true){
		if(!handleMade){
			handleMade = createHandle(goal->filePath);
			//as_.setSucceeded(result_);
		}
		while(!as_.isNewGoalAvailable() && goal->start){
			ros::spinOnce();
			for(int i = 0; i < 6; i++){
				myfile << efforts.effort[i] <<",";
			}
			for(int i = 0; i < 6; i++){
				myfile << joint_state.position[i] <<",";
			}
			myfile << toolpos.pose.position.x << ",";
			myfile << toolpos.pose.position.y << ",";
			myfile << toolpos.pose.position.z << ",";
			myfile << "\n";
		
		  // check that preempt has not been requested by the client
		  if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as_.setPreempted();
			success = false;
		  } else{
			  feedback_.logging = true;
			  // publish the feedback
			  as_.publishFeedback(feedback_);
			  r.sleep();
		  }
	  }
    }

    if(success){
		if(handleMade){
			myfile.close();
			ROS_INFO("Closing file.");
			handleMade = false;
		}
      result_.success = success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};

void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	toolpos = msg;
}

void jointstate_cb(const sensor_msgs::JointStateConstPtr& input){
	joint_state = *input;
}

void fingers_cb(const jaco_msgs::FingerPosition input){
}

void joint_effort_cb(const sensor_msgs::JointStateConstPtr& input){
	efforts = *input;
}

void write(std::string filePath){
	std::string path = filePath + "/example.csv";
	ROS_INFO("Making %s", path.c_str());
	
	std::ofstream myfile;
	myfile.open(path.c_str());
	//header
	ros::spinOnce();
	myfile << "efforts_q1,efforts_q2,efforts_q3,efforts_q4,efforts_q5,efforts_q6,joint_position_q1,joint_position_q2,joint_position_q3,joint_position_q4,joint_position_q5,joint_position_q6,tool_position_x,tool_position_y,tool_position_z,time\n";
	for(int i = 0; i < 6; i++){
		myfile << efforts.effort[i] <<",";
	}
	for(int i = 0; i < 6; i++){
		myfile << joint_state.position[i] <<",";
	}
	myfile << toolpos.pose.position.x << ",";
	myfile << toolpos.pose.position.y << ",";
	myfile << toolpos.pose.position.z << ",";
	myfile << ros::Time::now();

	myfile << "\n";

	myfile.close();

}

int main(int argc, char** argv){
	ros::init(argc, argv, "arm_perceptual_log_action");
	ros::NodeHandle n;
	signal(SIGINT, sig_handler);
	std::string tool_pos_, finger_pos_, joint_efforts_, joint_states_;
	
	//get the topics from the launch file  ---> grounded_logging.launch
	n.param<std::string>("tool_pos", tool_pos_, "/mico_arm_driver/out/tool_position");
	n.param<std::string>("finger_pos", finger_pos_, "/mico_arm_driver/out/finger_position");
	n.param<std::string>("joint_efforts", joint_efforts_, "/mico_arm_driver/out/joint_efforts");
	n.param<std::string>("joint_states", joint_states_, "joint_states");
	
	ros::Subscriber sub_tool = n.subscribe(tool_pos_, 1, toolpos_cb);
  	ros::Subscriber sub_finger = n.subscribe(finger_pos_, 1, fingers_cb);
	ros::Subscriber sub_torques = n.subscribe (joint_efforts_, 1, joint_effort_cb);
	ros::Subscriber sub_states = n.subscribe (joint_states_, 1, jointstate_cb);
	LogPerceptionAction log(ros::this_node::getName());
	
	ROS_INFO("Ready to record and process haptic data.");
	ros::spin();
}
