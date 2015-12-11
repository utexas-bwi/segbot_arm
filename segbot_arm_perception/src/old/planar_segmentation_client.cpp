#include "ros/ros.h"
#include "segbot_arm_perception/PlanarSegmentation.h"
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PlanarSegmentationTesterClient");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<segbot_arm_perception::PlanarSegmentation>("PlanarSegmentation");
  segbot_arm_perception::PlanarSegmentation srv_msg;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("segmented_pc", 1);

  //Test service requests
  srv_msg.request.excludePlane = false;
  srv_msg.request.numberOfPlanes = 0;
  char in = ' ';
  while(in != 'q' && ros::ok()){
    std::cout << "Enter q to quit, any other character to call the service." << std::endl;
    std::cin >> in; 
    if (client.call(srv_msg))
    {
      std::vector<sensor_msgs::PointCloud2> res = srv_msg.response.clouds;
      ROS_INFO("Got %lu clouds. Publishing to /segmented_pc", res.size());
      for(int i = 0; i < res.size(); i++){
        std::cin >> in;    //cycle through all the pointclouds. Useful for visualizing in rviz
        std::cout << "Coefficients for this cloud: xyzw " << srv_msg.response.coefficients.at(i).x << " " << srv_msg.response.coefficients.at(i).y;
        std::cout << " " << srv_msg.response.coefficients.at(i).z << " " << srv_msg.response.coefficients.at(i).w;
        pub.publish(res.at(i));
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
