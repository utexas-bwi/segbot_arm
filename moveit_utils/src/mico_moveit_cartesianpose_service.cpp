/* This service is the IK flavor of the movement interface to the MoveIt! move_group and planning_action interfaces.
 * 
 * Author: Maxwell Svetlik
 * 
 * */

/* Temporary note: The IK solver currently used is likely too slow.
 * Switch to IK Fast: 
 * 	http://moveit.ros.org/wiki/Kinematics/IKFast
 *  http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution
 * */

#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//services
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_utils/MicoMoveitCartesianPose.h"


bool g_caught_sigint = false;
std::vector<double> q_vals;

ros::ServiceClient controller_client;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};
bool service_cb(moveit_utils::MicoMoveitCartesianPose::Request &req, moveit_utils::MicoMoveitCartesianPose::Response &res){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");
    
    moveit_utils::MicoController srv_controller;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setPlanningTime(5.0); //5 second maximum for collision computation
    moveit::planning_interface::MoveGroup::Plan my_plan;
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = req.target.pose.orientation;
    goal.pose.position = req.target.pose.position;
    group.setPoseTarget(req.target);
    group.setStartState(*group.getCurrentState());

	ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    bool success = group.plan(my_plan);
    ROS_INFO("planning success: %c", success);
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
    ros::init(argc, argv, "mico_moveit_cartesianpose_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //make controller service
    //TODO: as a rosparam, option to use different controllers
    //ros::ServiceClient client = nh.serviceClient<moveit_utils::MicoMoveitJointPose>("mico_moveit_joint_pose");
    controller_client = nh.serviceClient<moveit_utils::MicoController>("mico_controller");
    ros::ServiceServer srv = nh.advertiseService("mico_cartesianpose_service", service_cb);

    //TODO: as a rosparam, option for planning time
    ros::spin();
    return 0;
}


