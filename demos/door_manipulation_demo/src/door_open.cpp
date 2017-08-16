#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/impl/transforms.hpp>
//including package services 
#include "door_manipulation_demo/door_perception.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
//#include <controller_manager.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <time.h>
#include <stdint.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/grasp_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>
#include "agile_grasp/Grasps.h"

#define NUM_JOINTS 8
#define HAND_OFFSET_GRASP -0.02
#define HAND_OFFSET_APPROACH -0.13
#define ANGULAR_DIFF_THRESHOLD 12.0

Eigen::Vector4f centroid;

geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped first_goal;
geometry_msgs::PoseStamped second_goal;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Quaternion plane_coeff;
geometry_msgs::Quaternion orig_plane_coeff;

ArmPositionDB *posDB;
agile_grasp::Grasps current_grasps;
sensor_msgs::JointState joint_state_outofview;
sensor_msgs::JointState current_state;

bool heardPose = false;
bool heardJoinstState = false;
bool heardGoal = false;
bool g_caught_sigint = false;

ros::Publisher first_goal_pub;
ros::Publisher second_goal_pub;
ros::Publisher pub_velocity;

bool similar(float x1, float x2){
    if(x1 - .05 < x2 && x2 < x1 +.05){
        return true;
    }
    return false;
}   

void sig_handler(int sig)
{
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};


moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
    ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
    
    
    moveit_msgs::GetPositionIK::Request ikine_request;
    moveit_msgs::GetPositionIK::Response ikine_response;
    ikine_request.ik_request.group_name = "arm";
    ikine_request.ik_request.pose_stamped = p;
    
    /* Call the service */
    if(ikine_client.call(ikine_request, ikine_response)){
        ROS_INFO("IK service call success:");
   
    } else {
        ROS_INFO("IK service call FAILED. Exiting");
    }
    
    return ikine_response;
}

//get the recorded topics
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input){
    
    if (input->position.size() == NUM_JOINTS){
        current_state = *input;
        heardJoinstState = true;
    }
}


// Blocking call for user input
void pressEnter(){
    std::cout << "Press the ENTER key to continue";
    while (std::cin.get() != '\n'){
        std::cout << "Please press ENTER\n";
    }    
}

//listen for where the arm is 
void listenForArmData(float rate){
    heardPose = false;
    heardJoinstState = false;
    ros::Rate r(rate);
    
    while (ros::ok()){
        ros::spinOnce();
        
        if (heardPose && heardJoinstState){
            return;
        }
        r.sleep();
    }
}

//get the first approach goal
void goal_cb (const geometry_msgs::PoseStampedConstPtr& input)
{
    ROS_INFO("entered goal_cb");
    first_goal.header = input->header;
    first_goal.pose = input->pose; 
    first_goal.pose.position.x -= .05;
    second_goal.header = input->header;
    second_goal.pose = input->pose;
    second_goal.pose.position.x += .15;
    heardGoal = true;
    
}
void toolpos_cb (const geometry_msgs::PoseStampedConstPtr& input)
{
    ROS_INFO("entered toolposecb");
    current_pose.header = input->header;
    current_pose.pose = input->pose;
        
}

void plane_coeff_cb (const geometry_msgs::QuaternionConstPtr& input){
    plane_coeff = *input;
}   

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "segbot_arm_door_open_detector");
    ros::NodeHandle n;
    tf::TransformListener listener;
    
    first_goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_picked", 1);
    second_goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_to_go_2", 1);
    ros::ServiceClient client = n.serviceClient<door_manipulation_demo::door_perception>("/door_handle_detection/door_perception");
    signal(SIGINT, sig_handler);
    
    //get arm position
    segbot_arm_manipulation::closeHand();
    //set goal for mico service
    door_manipulation_demo::door_perception door_srv;
    moveit_utils::MicoMoveitCartesianPose mico_srv;
    mico_srv.request.target = first_goal;
    //home arm and move it to the start position
    segbot_arm_manipulation::homeArm(n);
    ros::spinOnce();
    //move arm out of camera space to see the door
    if (posDB->hasCarteseanPosition("side_view")){
        ROS_INFO("Moving out of the way...");
        geometry_msgs::PoseStamped out_of_view_pose = posDB->getToolPositionStamped("side_view","/mico_link_base");
        //now go to the pose
        segbot_arm_manipulation::moveToPoseMoveIt(n,out_of_view_pose);
        segbot_arm_manipulation::moveToPoseMoveIt(n,out_of_view_pose);
    }
    else{
        ROS_ERROR("[segbot_table_approach_as.cpp] Cannot move arm out of view!");
    }
    ros::spinOnce();
    //make calls to get vision
    if(client.call(door_srv)){
        ros::spinOnce();
        ROS_INFO("entered");
    } else{
        ROS_INFO("didn't enter vision");
    }
    orig_plane_coeff = plane_coeff; 
    //make an array of poses for first goal     
    geometry_msgs::PoseArray poses_msg_first;
    poses_msg_first.header.seq = 1;
    poses_msg_first.header.stamp = first_goal.header.stamp;
    poses_msg_first.header.frame_id = "mico_api_origin";
    int changex1 = 0;
    int changey1 = 0;
    poses_msg_first.poses.push_back(first_goal.pose);
    for(int changex = 0; changex < 20; changex++){
        int occurances = 0;
        for(int changey = 0; changey < 20; changey++){
            geometry_msgs::Pose potential_approach;
            potential_approach = first_goal.pose;
            potential_approach.position.z -= .03;
            potential_approach.position.y += .03;
            poses_msg_first.poses.push_back(potential_approach);
            occurances++;
        }   
    }
    //make array of poses for the second goal
    geometry_msgs::PoseArray poses_msg_2nd;
    poses_msg_first.header.seq = 1;
    poses_msg_first.header.stamp = second_goal.header.stamp;
    poses_msg_first.header.frame_id = "mico_api_origin";
    poses_msg_first.poses.push_back(second_goal.pose);
    int changez = 0;
    int changey = 0;
    while(changez < 20){
        while( changey < 20){
            geometry_msgs::Pose push_point;
            push_point = second_goal.pose;
            push_point.position.z -= .025;
            push_point.position.y += .025;
            poses_msg_first.poses.push_back(push_point);
            changey++;
        }   
        changez++;
    }
    //here, we'll store all the oush options that pass the filters
    std::vector<geometry_msgs::PoseStamped> push_commands;
    for (unsigned int i = 0; i < poses_msg_first.poses.size(); i++){
        geometry_msgs::PoseStamped temp_first_goal; 
        temp_first_goal.header = poses_msg_first.header;
        temp_first_goal.pose = poses_msg_first.poses.at(i);
        geometry_msgs::PoseStamped temp_second_goal = temp_first_goal;  
        //filter two -- if IK fails
        moveit_msgs::GetPositionIK::Response  ik_response_approach = segbot_arm_manipulation::computeIK(n,temp_first_goal);
        //see if passed inverse kinematics
        if (ik_response_approach.error_code.val == 1){
            if (ik_response_approach.error_code.val == 1){
                //check if can move forward afterwards 
                temp_second_goal.pose.position.x += .05;
                moveit_msgs::GetPositionIK::Response  ik_response_approach = segbot_arm_manipulation::computeIK(n,temp_second_goal);
                if (ik_response_approach.error_code.val == 1){
                    std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(current_state, ik_response_approach.solution.joint_state);
                    double sum_d = 0;
                    for (int p = 0; p < D.size(); p++){
                        sum_d += D[p];
                    }
                    if (sum_d < ANGULAR_DIFF_THRESHOLD && sum_d > 1){
                        //now check to see how close the two sets of joint angles are -- if the joint configurations for the approach and grasp poses differ by too much, the grasp will not be accepted
                        ROS_INFO("Sum diff: %f",sum_d);
                        ROS_INFO("added to push commands size"); //%d", push_commands.size());
                        //store the IK results
                        
                        push_commands.push_back(temp_first_goal);
                    }
                }   
            }
        }
    }
                
    //check to see if all potential grasps have been filtered out
    if (push_commands.size() == 0){
        ROS_INFO("No feasible poses found demo done.");

    } else{
        int selected_push_index = -1;
        //find the grasp with closest orientatino to current pose
        double min_diff = 1000000.0;
        for (unsigned int i = 0; i < push_commands.size(); i++){
            double d_i = segbot_arm_manipulation::grasp_utils::quat_angular_difference(push_commands.at(i).pose.orientation, current_pose.pose.orientation);
                
            ROS_INFO("Distance for pose %i:\t%f",(int)i,d_i);
            if (d_i < min_diff){
                selected_push_index = (int)i;
                min_diff = d_i;
                ROS_INFO("picked orientation");
            }
        }
    if (selected_push_index == -1 || selected_push_index > push_commands.size()){
        ROS_WARN("selection failed. kill.");            
    } else {
        pressEnter();
        ROS_INFO("goal picked...check if pose is what you want in rviz if not ctr c.");
        first_goal_pub.publish(first_goal);
        //made vision calls check in rviz to see if correct then procede
        pressEnter();
        ROS_INFO("Demo starting...Move the arm to a 'ready' position .");
        ros::spinOnce();
        pressEnter();
        ROS_INFO("goal picked...check if pose is what you want in rviz if not ctr c.");
        segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
        ros::spinOnce();                                            
        segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
        ros::spinOnce(); 
        segbot_arm_manipulation::moveToPoseMoveIt(n,first_goal);
        ros::spinOnce(); 
        bool isReachable = false;
        //check if second goal is possible for back up if arm driver not in configuration not to move forward
        moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,second_goal);
        if(ik_response_approach.error_code.val == 1){
            ROS_INFO("entered first pose passed");
            second_goal_pub.publish(second_goal);
            isReachable = true;
        } else { 
            while( changex1 < 15 && !isReachable){
                second_goal.pose.position.z -= .01;
                while( changey1 < 15 && !isReachable){
                    second_goal.pose.position.y += .01;
                    moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(n,second_goal);
                    if(ik_response_approach.error_code.val == 1){
                        ROS_INFO("entered second pose passed");
                        second_goal_pub.publish(second_goal);
                        isReachable = true;
                    }   
                    changey1 ++;
                }   
                changex1 ++;
            }   
        }
        //move forward at this point
        ROS_INFO("moving forward");
        pressEnter();
        double timeoutSeconds = 3.85;
        int rateHertz = 100;
        geometry_msgs::TwistStamped velocityMsg;
        
        ros::Rate r(rateHertz);
        for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++){
            velocityMsg.twist.linear.x = 1.25;
            velocityMsg.twist.linear.y = 0.1;
            velocityMsg.twist.linear.z = 0.1;
            velocityMsg.twist.angular.x = 0.0;
            velocityMsg.twist.angular.y = 0.0;
            velocityMsg.twist.angular.z = 0.0;
            pub_velocity.publish(velocityMsg);
            r.sleep();
        }   
        //fail safe if talking directly to driver failed
        ROS_INFO("fail safe");
        pressEnter();
        segbot_arm_manipulation::moveToPoseMoveIt(n,second_goal);
        ros::spinOnce();                 
        segbot_arm_manipulation::moveToPoseMoveIt(n,second_goal);
        ros::spinOnce();  
        pressEnter();
        //return back to home position
        ROS_INFO("Demo ending...arm will move back 'ready' position .");
        segbot_arm_manipulation::homeArm(n);
        segbot_arm_manipulation::homeArm(n);
        //do a vision call to see if the dorr has moved
        if(client.call(door_srv)){
            ros::spinOnce();
        } else {
            ROS_INFO("didn't enter vision");
        }
        //check if door has moved and sent response of the action
        if(similar(orig_plane_coeff.x, plane_coeff.x) && similar(orig_plane_coeff.y, plane_coeff.y) && similar(orig_plane_coeff.z, plane_coeff.z)
            && similar(orig_plane_coeff.w, plane_coeff.w)){
                ROS_INFO("didn't move  door");
        } else {
                ROS_INFO("moved door");
        }   
        }
    }       
                
};
