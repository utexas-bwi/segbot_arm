#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <boost/assign/std/vector.hpp>
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "moveit_utils/MicoMoveitJointPose.h"
using namespace boost::assign;
    
#define NUM_JOINTS 6

bool g_caught_sigint = false;


moveit::planning_interface::MoveGroup *group;


void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};  
bool service_cb(moveit_utils::MicoMoveitJointPose::Request &req, moveit_utils::MicoMoveitJointPose::Response &res){

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(req.collision_objects);


    std::vector<double> q_vals;
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
    group->setJointValueTarget(q_vals);
    group->setStartState(*group->getCurrentState());

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    //call service

    if (!success) {
        res.completed = false;
        return true;
    }
    moveit::planning_interface::MoveItErrorCode error = group->move();
    res.completed = error;
    return true;
}   
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "mico_moveit_jointpose_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group = new moveit::planning_interface::MoveGroup("arm");
    group->setGoalTolerance(0.01);
    group->setPoseReferenceFrame("m1n6s200_end_effector");
    ros::ServiceServer srv = nh.advertiseService("mico_jointpose_service", service_cb);

    //TODO: as a rosparam, option for planning time
    ros::spin();
    return 0;
}
 


