#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>


//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
kinova_msgs::FingerPosition current_finger;


bool heardJoinstState;
bool heardPose;
bool heardFingers;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};

//Joint positions cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
    
    if (msg->position.size() == NUM_JOINTS){
        current_state = *msg;
        heardJoinstState = true;
    }
}

//tool pose cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
    current_pose = msg;
    heardPose = true;
}

//fingers state cb
void fingers_cb (const kinova_msgs::FingerPositionConstPtr& msg) {
    current_finger = *msg;
    heardFingers = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
    
    heardJoinstState = false;
    heardPose = false;
    heardFingers = false;
    
    ros::Rate r(40.0);
    
    while (ros::ok()){
        ros::spinOnce();    
        
        if (heardJoinstState && heardPose && heardFingers)
            return;
        
        r.sleep();
    }
}


// Blocking call for user input
void pressEnter(std::string message){
    std::cout << message;
    while (true){
        char c = std::cin.get();
        if (c == '\n')
            break;
        else if (c == 'q'){
            ros::shutdown();
            exit(1);
        }
        else {
            std::cout <<  message;
        }
    }
}



int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "ex1_subscribing_to_topics");
    
    ros::NodeHandle n;
    
    //create subscribers for arm topics
    
    //joint positions
    ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
    
    //cartesean tool position and orientation
    ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

    //finger positions
    ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
     
    //register ctrl-c
    signal(SIGINT, sig_handler);
        
    listenForArmData();

    pressEnter("Press enter...");

    geometry_msgs::PoseStamped p_target;
    // Pose with the end effector pointing straight up, with palm parallel with the frame of the robot
    p_target.header.frame_id = "m1n6s200_link_base";
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = -0.05;
    p_target.pose.position.z = 0.97;
    p_target.pose.orientation.x = 0.0;
    p_target.pose.orientation.y = 0.00;
    p_target.pose.orientation.z = -0.73;
    p_target.pose.orientation.w = 0.68;
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
    
    // Pose with end effector pointing straight left, with palm parallel with the frame of the robot
    pressEnter("Press enter...");
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = 0.40;
    p_target.pose.position.z = 0.50;
    p_target.pose.orientation.x = -0.50;
    p_target.pose.orientation.y = -0.50;
    p_target.pose.orientation.z = -0.50;
    p_target.pose.orientation.w = 0.5;
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
    
    // Pose with end effector pointing straight right, with palm parallel with the frame of the robot
    pressEnter("Press enter...");
    p_target.pose.position.x = 0.00;
    p_target.pose.position.y = -0.40;
    p_target.pose.position.z = 0.50;
    p_target.pose.orientation.x = 0.00;
    p_target.pose.orientation.y = 0.50;
    p_target.pose.orientation.z = -0.50;
    p_target.pose.orientation.w = 0.5;
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
    
    ROS_INFO("Moving to target x=%f, y=%f, z=%f", p_target.pose.position.x, p_target.pose.position.y, p_target.pose.position.z);
    
    ros::shutdown();
}
