#include <signal.h>
//services
#include "moveit_utils/MicoMoveitJointPose.h"
#include "ros/ros.h"
#include "moveit_utils/MicoNavSafety.h"
#include "jaco_msgs/JointAngles.h"
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;


/*
Safe q_poses for navigation

joint1: 16.2132263184
joint2: 179.944854736
joint3: 83.7132339478
joint4: 327.477294922
joint5: 99.8863830566
joint6: 337.704528809


safe position:
position: [-2.121345227360501, -2.8422558770681094, 0.9624977766765357, -3.030922863962878, -0.07378021483956007, 2.7596034742150044, 0.9559199999999999, 0.95256]



position in footprint, out of way of camera
position: [-2.1973822049077945, -2.712318649586481, 0.9615352436795538, -2.4823340499596753, -0.49860873719346416, 2.753653439707549, 0.9559199999999999, 0.95256]

Transition pose to pose above
position: [-2.1993071377437037, -1.3080344725778785, 0.9615352436795538, -2.4799542492095803, -0.49860873719346416, 2.753653439707549, 0.9559199999999999, 0.95256]


*/



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

