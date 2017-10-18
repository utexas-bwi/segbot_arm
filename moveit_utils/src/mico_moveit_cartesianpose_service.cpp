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
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
//services
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_utils/MicoMoveitCartesianPose.h"


bool g_caught_sigint = false;
std::vector<double> q_vals;

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac;

ros::Publisher pose_pub;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};
bool service_cb(moveit_utils::MicoMoveitCartesianPose::Request &req, moveit_utils::MicoMoveitCartesianPose::Response &res){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");
    
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setPlanningTime(5.0); //5 second maximum for collision computation
    moveit::planning_interface::MoveGroup::Plan my_plan;
    {
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = req.target.pose.orientation;
    goal.pose.position = req.target.pose.position;
    }
	//publish target pose
	pose_pub.publish(req.target);
   
    group.setPoseTarget(req.target);
    group.setStartState(*group.getCurrentState());

	ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    bool success = group.plan(my_plan);
    ROS_INFO("planning success: %c", success);
    //call service
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = my_plan.trajectory_.joint_trajectory;
    ac->sendGoal(goal);
    if(ac->waitForResult(ros::Duration(30.0))){
       ROS_INFO("Trajectory sent. Prepare for movement.");
       res.completed = true;
    }
    else {
       ROS_INFO("Action call failed.");
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

    ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/m1n6s200/follow_joint_trajectory", true);
    ros::ServiceServer srv = nh.advertiseService("mico_cartesianpose_service", service_cb);

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mico_cartesianpose_service/target_pose", 10);
	

    //TODO: as a rosparam, option for planning time
    ros::spin();
    return 0;
}


