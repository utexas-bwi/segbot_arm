#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
//services
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_utils/MicoMoveitCartesianPose.h"

bool g_caught_sigint = false;
std::vector<double> q_vals;


ros::Publisher pose_pub;
moveit::planning_interface::MoveGroup *group;
robot_state::JointModelGroup *joint_model_group;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
}

bool service_cb(moveit_utils::MicoMoveitCartesianPose::Request &req, moveit_utils::MicoMoveitCartesianPose::Response &res){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");

    //publish target pose
    pose_pub.publish(req.target);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(req.collision_objects);
    group->setPlanningTime(5.0); //5 second maximum for collision computation
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group->setPoseTarget(req.target);
    group->setStartState(*group->getCurrentState());
    

    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    bool success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    if (!success) {
        res.completed = false;
        return true;
    }
    group->move();
    ros::spinOnce();
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mico_moveit_cartesianpose_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group = new moveit::planning_interface::MoveGroup("arm");
    group->setGoalTolerance(0.01);
    group->setPoseReferenceFrame("m1n6s200_end_effector");
    ros::ServiceServer srv = nh.advertiseService("mico_cartesianpose_service", service_cb);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mico_cartesianpose_service/target_pose", 1, true);

    ros::spin();
    return 0;
}


