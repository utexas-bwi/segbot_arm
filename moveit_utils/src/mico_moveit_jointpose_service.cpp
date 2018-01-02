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
#include "moveit_utils/MicoMoveitJointPose.h"
#include <moveit_msgs/MoveItErrorCodes.h>
using namespace boost::assign;
    
#define NUM_JOINTS 6

bool g_caught_sigint = false;
std::vector<double> q_vals;

ros::ServiceClient controller_client;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};  
bool service_cb(moveit_utils::MicoMoveitJointPose::Request &req, moveit_utils::MicoMoveitJointPose::Response &res){
    moveit_utils::MicoController srv_controller;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
    group.setPlanningTime(5.0); //10 second maximum for collision computation


    q_vals.clear();
    for(int i = 0; i < NUM_JOINTS; i++){
        switch(i) {
            case 0  :    q_vals.push_back(req.target.joint1); break;
            case 1  :    q_vals.push_back(req.target.joint2); break;
            case 2  :    q_vals.push_back(req.target.joint3); break;
            case 3  :    q_vals.push_back(req.target.joint4); break;
            case 4  :    q_vals.push_back(req.target.joint5); break;
            case 5  :    q_vals.push_back(req.target.joint6); break;
        }
	//ROS_INFO("Requested angle: %f", q_vals.at(i));
    }
    group.setJointValueTarget(q_vals);
    group.setStartState(*group.getCurrentState());
    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit_msgs::MoveItErrorCodes result = group.plan(my_plan);
    ROS_INFO("result: %d", result.val);
    //call service
    moveit_utils::MicoController srv;
    srv_controller.request.trajectory = my_plan.trajectory_;
    if(controller_client.call(srv_controller)){
       ROS_INFO("Service call sent. Prepare for movement.");
       res.completed = srv_controller.response.done;
    }
    else {
       ROS_INFO("Service call failed. Is the service running?");
       res.completed = false;
    }
    ros::spinOnce();
    return true;
}   
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //make controller service
    //TODO: as a rosparam, option to use different controllers
    //ros::ServiceClient client = nh.serviceClient<moveit_utils::MicoMoveitJointPose>("mico_moveit_joint_pose");
    controller_client = nh.serviceClient<moveit_utils::MicoController>("mico_controller");
    ros::ServiceServer srv = nh.advertiseService("mico_jointpose_service", service_cb);

    //TODO: as a rosparam, option for planning time
    ros::spin();
    return 0;
}
 


