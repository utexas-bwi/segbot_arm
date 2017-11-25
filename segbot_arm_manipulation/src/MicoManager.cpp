#include "kinova_msgs/ArmJointAnglesAction.h"
#include <moveit_msgs/GetPositionIK.h>

#include "segbot_arm_perception/SetObstacles.h"
#include <segbot_arm_manipulation/arm_utils.h>

#include <segbot_arm_manipulation/MicoManager.h>

using namespace std;

MicoManager::MicoManager(ros::NodeHandle n) : pose_action(pose_action_topic, true), fingers_action(finger_action_topic, true) {

    //joint positions
    joint_state_sub = n.subscribe (joint_state_topic, 1, &MicoManager::joint_state_cb, this);
    //cartesian tool position and orientation
    tool_sub = n.subscribe(tool_pose_topic, 1, &MicoManager::toolpose_cb, this);
    //finger positions
    finger_sub = n.subscribe(finger_position_topic, 1, &MicoManager::fingers_cb, this);
    home_client = n.serviceClient<kinova_msgs::HomeArm>(home_arm_service);
    safety_client = n.serviceClient<moveit_utils::MicoNavSafety>("/mico_nav_safety");
    pose_moveit_client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose> ("/mico_cartesianpose_service");
    positionDB = new ArmPositionDB(j_pos_filename, c_pos_filename);
    angular_velocity_pub = n.advertise<kinova_msgs::JointAngles>("/m1n6s200_driver/in/joint_velocity", 10);

    ik_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
}

//Joint positions cb
void MicoManager::joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
    if (msg->position.size() == NUM_JOINTS){
        current_state = *msg;
        heardJointState = true;
    }
}

//tool pose cb
void MicoManager::toolpose_cb (const geometry_msgs::PoseStampedConstPtr &msg) {
    current_pose = *msg;
    heardTool = true;
}

//fingers state cb
void MicoManager::fingers_cb (const kinova_msgs::FingerPositionConstPtr& msg) {
    current_finger = *msg;
    heardFingers = true;
}

//blocking call to listen for arm data (in this case, joint states)
void MicoManager::wait_for_data(){

    heardJointState = false;
    heardTool = false;
    heardFingers = false;

    ros::Rate r(40.0);

    while (ros::ok()){
        ros::spinOnce();

        if (heardJointState && heardTool && heardFingers)
            return;

        r.sleep();
    }
}

void MicoManager::move_to_pose(const geometry_msgs::PoseStamped &pose){
    kinova_msgs::ArmPoseGoal goalPose;
    goalPose.pose = pose;

    pose_action.waitForServer();

    //finally, send goal and wait
    pose_action.sendGoal(goalPose);
    pose_action.waitForResult();
}

void MicoManager::move_fingers(const int finger_value1, const int finger_value2){

    kinova_msgs::SetFingersPositionGoal goalFinger;
    goalFinger.fingers.finger1 = finger_value1;
    goalFinger.fingers.finger2 = finger_value2;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;

    fingers_action.waitForServer();
    fingers_action.sendGoal(goalFinger);
    fingers_action.waitForResult();
}

void MicoManager::move_fingers(int finger_value){
    move_fingers(finger_value, finger_value);
}


bool MicoManager::make_safe_for_travel(){
    safety_client.waitForExistence();
    moveit_utils::MicoNavSafety srv_safety;
    srv_safety.request.getSafe = true;

    if (safety_client.call(srv_safety)) {
        return srv_safety.response.safe;
    }
    else {
        return false;
    }
}

void MicoManager::move_home(){
    kinova_msgs::HomeArm srv;
    home_client.call(srv);
}


void MicoManager::open_hand(){
    move_fingers(OPEN_FINGER_VALUE);
}

void MicoManager::close_hand(){
    move_fingers(CLOSED_FINGER_VALUE);
}

moveit_msgs::GetPositionIK::Response MicoManager::compute_ik(const geometry_msgs::PoseStamped &p){

    moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;
    ik_request.ik_request.group_name = "arm";
    ik_request.ik_request.pose_stamped = p;

    /* Call the service */
    ik_client.call(ik_request, ik_response);
    return ik_response;

}


bool MicoManager::move_to_pose_moveit(const geometry_msgs::PoseStamped &target,
                                      const vector<sensor_msgs::PointCloud2> &obstacles){
    moveit_utils::MicoMoveitCartesianPose::Request 	req;
    moveit_utils::MicoMoveitCartesianPose::Response res;

    vector<moveit_msgs::CollisionObject> moveit_obstacles = segbot_arm_manipulation::get_collision_boxes(obstacles);

    req.target = target;
    req.collision_objects = moveit_obstacles;

    return pose_moveit_client.call(req, res);


}



void MicoManager::move_to_side_view(){
    if (positionDB->hasCarteseanPosition("side_view")){
        ROS_INFO("Moving arm to side view...");
        geometry_msgs::PoseStamped out_of_view_pose = positionDB->getToolPositionStamped("side_view","/m1n6s200_link_base");
        move_to_pose_moveit(out_of_view_pose);
    }else {
        ROS_ERROR("[arm_utils] Cannot move arm to side view!");
    }
}


void MicoManager::move_to_handover(){
    if (positionDB->hasCarteseanPosition("handover_front")){
        geometry_msgs::PoseStamped handover_pose = positionDB->getToolPositionStamped("handover_front","m1n6s200_link_base");

        ROS_INFO("Moving to handover position");

        move_to_pose_moveit(handover_pose);
    }else {
        ROS_ERROR("[arm_utils] cannot move to the handover position!");
    }

}