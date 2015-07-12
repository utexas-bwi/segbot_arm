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
  actionlib::SimpleActionServer<segbot_arm_perception::FibonacciAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  learning_actionlib::FibonacciFeedback feedback_;
  learning_actionlib::FibonacciResult result_;
  std::ofstream myfile;
  bool handleMade = false;
  
public:

  LogPerceptionAction(std::string name) :
    as_(nh_, name, boost::bind(&LogPerceptionAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
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

  void executeCB(const learning_actionlib::FibonacciGoalConstPtr &goal){
    // helper variables
    ros::Rate r(1);
    bool success = true;

	if(goal->start == true){
		if(!handleMade){
			handleMade = createHandle(); 
		}
		
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
		  write(goal->filePath);
		  //set feedback here
		  // publish the feedback
		  as_.publishFeedback(feedback_);
		  // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
		  r.sleep();
	  }
    } else if(goal->start == false){
		myfile.close();
	}

    if(success){
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
bool log_cb(segbot_arm_perception::LogPerception::Request &req, segbot_arm_perception::LogPerception::Response &res){
	if(req.start == true){
		start = true;
		write(req.filePath);
		res.success = true;
	}else if(req.start == false){
		start = false;
		write("");
		res.success = true;
	}
	return res.success;

}

int main(int argc, char** argv){
	ros::init(argc, argv, "arm_perceptual_log_node");
	ros::NodeHandle n;
	signal(SIGINT, sig_handler);
	
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
	ros::Subscriber sub_states = n.subscribe ("joint_states", 1, jointstate_cb);
	ros::ServiceServer service = n.advertiseService("log_perception", log_cb);
	FibonacciAction log(ros::this_node::getName());
	ros::spin();
}
