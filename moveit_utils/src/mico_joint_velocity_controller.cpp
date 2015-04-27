#include <ros/ros.h>

//messages and types
#include <sensor_msgs/JointState.h>


ros::Publisher j_vel_pub;
bool g_caught_sigint=false;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("Caught sigint, shutting down...");
    ros::shutdown();
    exit(1);
}
void joint_state_cb(const sensor_msgs::JointStateConstPtr& js){

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_joint_velocity_controller");
    signal(SIGINT, sig_handler);
    ros::NodeHandle n;

    //subscriber for position check
    ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
    //publisher for velocity commands
    j_vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/joint_velocity", 10);
    
    ros::ServiceServer srv = n.advertiseService("mico_joint_velocity_controller_service");
    ROS_INFO("Mico joint velocity controller server started.");
    
    ros::spin();
    return 0;
}
