#include "ros/ros.h"
#include "segbot_arm_perception/LogPerception.h"
#include <vector>
#include <ros/package.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "LogPerceptionClient");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<segbot_arm_perception::LogPerception>("log_perception");
	segbot_arm_perception::LogPerception srv_msg;
	std::string path = ros::package::getPath("segbot_arm_perception");

	srv_msg.request.filePath = path;
	if (client.call(srv_msg)){
		ROS_INFO("Called service");
	} else{
    ROS_ERROR("Failed to call service");
		return 1;
    }
  return 0;
}
