#include <signal.h>
//services
#include "moveit_utils/MicoMoveitJointPose.h"
#include "ros/ros.h"
#include "moveit_utils/MicoNavSafety.h"
#include <moveit_msgs/GetPositionFK.h>

#include "jaco_msgs/JointAngles.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <boost/assign/std/vector.hpp>
#include <tf/transform_listener.h>

using namespace boost::assign;


/*
Safe q_poses for navigation

joint1: 16.2132263184
joint2: 179.944854736
joint3: 83.7132339478
joint4: 327.477294922
joint5: 99.8863830566
joint6: 337.704528809


name: ['mico_joint_1', 'mico_joint_2', 'mico_joint_3', 'mico_joint_4', 'mico_joint_5', 'mico_joint_6', 'mico_joint_finger_1', 'mico_joint_finger_2']
position: [-1.4918714173616245, -1.8046833895251533, -0.12993722748162864, -2.1717450183580356, 0.5688181856434458, 2.6786835908090505, -0.0008399999999999999, 0.0]

*/



bool g_caught_sigint = false;
std::vector<double> q_safe;
ros::ServiceClient movement_client;
ros::Publisher pub;
double inflationRad;
bool safe = false;
std_msgs::Bool pub_data; 
sensor_msgs::JointState js_cur;
ros::ServiceClient fkine_client;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};

bool checkIfSafe(){
    while(!fkine_client.exists()){
        ROS_INFO("Waiting for service");
        sleep(1.0);
    }

    /*
     * Forward Kinematic service call
     */

    moveit_msgs::GetPositionFK::Request fkine_request;
    moveit_msgs::GetPositionFK::Response fkine_response;

    sensor_msgs::JointState q_true = js_cur;

    //Load request with the desired link
    fkine_request.fk_link_names.push_back("mico_link_1");
    fkine_request.fk_link_names.push_back("mico_link_2");
    fkine_request.fk_link_names.push_back("mico_link_3");
    fkine_request.fk_link_names.push_back("mico_link_4");
    fkine_request.fk_link_names.push_back("mico_link_5");
    fkine_request.fk_link_names.push_back("mico_link_hand");
    fkine_request.fk_link_names.push_back("mico_link_finger_1");
    fkine_request.fk_link_names.push_back("mico_link_finger_2");


    ROS_INFO("Making FK call");
    if(fkine_client.call(fkine_request, fkine_response)){
        ros::spinOnce();
        ROS_INFO("Call successful.");
        ROS_INFO_STREAM(fkine_response);
    } else {
        ROS_INFO("Call failed. Terminating.");
        ros::shutdown();
        return false;
    }
    
    /*
     * Check if any joints are outside the radius
     */
    bool temp = true;
	for(int i = 0; i < fkine_response.pose_stamped.size(); i++){
		if(fkine_response.pose_stamped.at(i).pose.position.x > inflationRad ||
				fkine_response.pose_stamped.at(i).pose.position.y > inflationRad ||
				fkine_response.pose_stamped.at(i).pose.position.x < -inflationRad ||
				fkine_response.pose_stamped.at(i).pose.position.y < -inflationRad)
			temp = false;
	}
	safe = temp;
	pub_data.data = temp;
	pub.publish(pub_data);
	return temp;
}

void joint_state_cb(const sensor_msgs::JointStateConstPtr& js){
    if (js->position.size() > 4){ //Message from the base or the arm
        js_cur = *js;
        checkIfSafe();
    }
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
    ros::Subscriber sub_angles = nh.subscribe ("/joint_states", 10, joint_state_cb);
    pub = nh.advertise<std_msgs::Bool>("mico_nav_safe", 10);
    ros::ServiceServer srv = nh.advertiseService("mico_nav_safety", service_cb);
    fkine_client = nh.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");

    q_safe += -1.4918,-1.804,-0.1299,-2.1717,.5688,2.6787; //defines the 'go-to' safe position

    nh.param("inflation_radius", inflationRad, .2);

    ros::spin();
    return 0;
}

