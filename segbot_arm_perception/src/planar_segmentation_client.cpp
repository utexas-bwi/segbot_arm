#include "ros/ros.h"
#include "segbot_arm_perception/PlanarSegmentation.h"
#include <vector>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PlanarSegmentationTesterClient");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<segbot_arm_perception::PlanarSegmentation>("PlanarSegmentation");
  segbot_arm_perception::PlanarSegmentation srv_msg;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("segmented_pc", 1);

  srv_msg.request.excludePlane = false;
  char in = ' ';
  while(in != 'q' && ros::ok()){
    std::cout << "Enter q to quit, any other character to call the service." << std::endl;
    std::cin >> in; 
    if (client.call(srv_msg))
    {
      ROS_INFO("Got a response. Publishing to /segmented_pc");
      std::vector<sensor_msgs::PointCloud2> res = srv_msg.response.clouds;
      for(int i = 0; i < res.size(); i++){
        pub.publish(res.at(i));
        std::cin >> in;    //cycle through all the pointclouds. Useful for visualizing in rviz
      }
    }
    else
    {
      ROS_ERROR("Failed to call service");
      return 1;
    }
  }

  return 0;
}