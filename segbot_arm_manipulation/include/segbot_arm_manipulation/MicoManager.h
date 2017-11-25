#ifndef SEGBOT_ARM_MANIPULATION_MICOMANAGER_H
#define SEGBOT_ARM_MANIPULATION_MICOMANAGER_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>
#include <ros/node_handle.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/HomeArm.h>
#include <moveit_utils/MicoNavSafety.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
#include <moveit_msgs/GetPositionIK.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include "MicoManager.h"

#define OPEN_FINGER_VALUE 100
#define CLOSED_FINGER_VALUE 7200

class ArmPositionDB;

const std::string moveit_cartesian_pose_service = "";

const std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
const std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";

class MicoManager {
    ros::Subscriber joint_state_sub;
    ros::Subscriber tool_sub;
    ros::Subscriber finger_sub;
    actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> pose_action;
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingers_action;
    ros::ServiceClient home_client;
    ros::ServiceClient safety_client;
    ros::ServiceClient pose_moveit_client;
    ros::ServiceClient ik_client;
    ArmPositionDB *positionDB;

public:
    ros::Publisher angular_velocity_pub;
    sensor_msgs::JointState current_state;
    geometry_msgs::PoseStamped current_pose;
    kinova_msgs::FingerPosition current_finger;

    bool heardJointState;
    bool heardTool;
    bool heardFingers;

    MicoManager(ros::NodeHandle n);

    void joint_state_cb(const sensor_msgs::JointStateConstPtr &msg);

    void toolpose_cb(const geometry_msgs::PoseStampedConstPtr &msg);

    void fingers_cb(const kinova_msgs::FingerPositionConstPtr &msg);

    void wait_for_data();

    void move_to_pose(const geometry_msgs::PoseStamped &pose);

    void move_fingers(int finger_value);

    void move_fingers(const int finger_value1, const int finger_value2);

    void open_hand();

    void close_hand();

    bool make_safe_for_travel();

    void move_home();

    void move_to_side_view();

    void move_to_handover();

    bool move_to_pose_moveit(const geometry_msgs::PoseStamped &target,
                             const std::vector<sensor_msgs::PointCloud2> &obstacles = std::vector<sensor_msgs::PointCloud2>());



    moveit_msgs::GetPositionIK::Response compute_ik(const geometry_msgs::PoseStamped &p);
};

#endif //SEGBOT_ARM_MANIPULATION_MICOMANAGER_H
