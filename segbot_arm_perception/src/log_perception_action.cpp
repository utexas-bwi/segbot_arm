/*
* This ROS service logs perceptual data relating to the arm (and vision if applicable)
* 
* The following series of data are logged into csv(s)
* 	-Joint efforts
* 	-Joint positions
* 	-EF position
* 
* Output is based on the input string for the service request. A file naming scheme
* is to be determined based on the experimental structure.
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
	std::string path = filePath + "/example.csv";
	ROS_INFO("Making %s", path.c_str());	
	myfile.open(path.c_str());
	//write header
	myfile << "efforts,joint_position,tool_position\n";
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
				myfile << efforts.effort[i] <<"," << joint_state.position[i] << ",";
				if(i < 3){
					switch(i){
						case 0:
							myfile << toolpos.pose.position.x << ","; break;
						case 1:
							myfile << toolpos.pose.position.y << ","; break;
						case 2:
							myfile << toolpos.pose.position.z << ","; break;
					}
				}
				myfile << "\n";
			}
		
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
	myfile << "efforts,joint_position,tool_position\n";
	for(int i = 0; i < 6; i++){
		myfile << efforts.effort[i] <<"," << joint_state.position[i] << ",";
		if(i < 3){
			switch(i){
				case 0:
					myfile << toolpos.pose.position.x << ","; break;
				case 1:
					myfile << toolpos.pose.position.y << ","; break;
				case 2:
					myfile << toolpos.pose.position.z << ","; break;
			}
		}
		myfile << "\n";
	}
	myfile.close();

}

int main(int argc, char** argv){
	ros::init(argc, argv, "arm_perceptual_log_action");
	ros::NodeHandle n;
	signal(SIGINT, sig_handler);
	
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
	ros::Subscriber sub_states = n.subscribe ("joint_states", 1, jointstate_cb);
	LogPerceptionAction log(ros::this_node::getName());
	ros::spin();
}
