nclude <signal.h>
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
using namespace boost::assign;
    
#define NUM_JOINTS 6


bool g_caught_sigint = false;
std::vector<double> q_vals;


void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};  
void toolpos_cb(const geometry_msgs::PoseStamped &msg){
    q = msg.pose.orientation;
}
void service_cb(moveit_utils::MicoMoveitJointPose::Request &req, moveit_utils::MicoMoveitJointPose::Response &res){
    for(int i = 0; i < NUM_JOINTS; i++){
        switch(i) {
            case 1  :    q_vals.push_back(req.target.joint1); break;
            case 2  :    q_vals.push_back(req.target.joint2); break;
            case 3  :    q_vals.push_back(req.target.joint3); break;
            case 4  :    q_vals.push_back(req.target.joint4); break;
            case 5  :    q_vals.push_back(req.target.joint5); break;
            case 6  :    q_vals.push_back(req.target.joint6); break;
        }
    }
    group.setJointValueTarget(q_vals);
    group.setStartState(*group.getCurrentState());
    //publish pose
    geometry_msgs::PoseStamped stampOut;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    //call service
    moveit_utils::MicoController srv;
    srv.request.trajectory = my_plan.trajectory_;
    if(client.call(srv)){
       ROS_INFO("Service call sent. Prepare for movement.");
       res.completed = req.done;
    }
    else {
       ROS_INFO("Service call failed. Is the service running?");
       res.completed = false;
    }
}   
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //button position publisher
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("target_trajectory/pose", 10);
    //make controller service
    //TODO: as a rosparam, option to use different controllers
    ros::ServiceClient client = node_handle.serviceClient<moveit_utils::MicoVelocityController>("mico_velocity_controller");
    //subscribers
    ros::Subscriber sub_tool = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
    moveit::planning_interface::MoveGroup group("arm"); //this is the specific group name you'd like to move
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/       display_planned_path", 1, true);
    //TODO: as a rosparam, option for planning time
    group.setPlanningTime(5.0); //10 second maximum for collision computation

    ros::shutdown();
    return 0;
}
 


