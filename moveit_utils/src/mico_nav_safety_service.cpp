#include <signal.h>
//services
#include "moveit_utils/MicoMoveitJointPose.h"
#include "ros/ros.h"
#include "moveit_utils/MicoNavSafety.h"
#include "jaco_msgs/JointAngles.h"
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

bool g_caught_sigint = false;
std::vector<double> q_safe;
ros::ServiceClient movement_client;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};

bool service_cb(moveit_utils::MicoNavSafety::Request &req, moveit_utils::MicoNavSafety::Response &res){
    moveit_utils::MicoMoveitJointPose movement_srv;
    jaco_msgs::JointAngles target;
    target.joint1 = q_safe.at(0);
    target.joint2 = q_safe.at(1);
    target.joint3 = q_safe.at(2);
    target.joint4 = q_safe.at(3);
    target.joint5 = q_safe.at(4);
    target.joint6 = q_safe.at(5);
    movement_srv.request.target = target;
    if(movement_client.call(movement_srv)){
    	ROS_INFO("Safety service call sent. Preparing to move arm to save location.");
	res.safe = movement_srv.response.completed;
    }
    else{
	ROS_INFO("Safety service call failed. Is the service running?");
	res.safe = false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_nav_safety_node");
    ros::NodeHandle nh;
    movement_client = nh.serviceClient<moveit_utils::MicoMoveitJointPose>("mico_jointpose_service");
    ros::ServiceServer srv = nh.advertiseService("mico_nav_safety", service_cb);

    q_safe += 2.86,-1.88,-0.198,-1.634,-0.1,2.49;


    ros::spin();
    return 0;
}

