#include <signal.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include "segbot_arm_perception/PlanarSegmentation.h"

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

int main (int argc, char** argv)
{
	ros::init (argc, argv, "approach_board");
	ros::NodeHandle n;
	
	//segmented cloud service client
	ros::ServiceClient ps_client = n.serviceClient<segbot_arm_perception::PlanarSegmentation>("PlanarSegmentation");
	//pose pub
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("approach_board/pose", 10);

	tf::TransformListener listener;
	signal(SIGINT, sig_handler);
	
	segbot_arm_perception::PlanarSegmentation srv;
	srv.request.excludePlane = false;
	srv.request.numberOfPlanes = 3;
	char input;
	
	while (ros::ok() && !g_caught_sigint) {
		std::cout << "Enter 1 to call the service and initiate the approach board action" << std::endl;
		std::cin >> input;
		if(input == '1'){
			if(ps_client.call(srv)){
				ros::spinOnce();
				
				std::vector<sensor_msgs::PointCloud2> res = srv.response.clouds;
				if(res.size() > 0){
					//conversion of pointcloud
					pcl::PCLPointCloud2 temp;
					pcl_conversions::toPCL(res.at(0),temp);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::fromPCLPointCloud2(temp,*cloud_plane);
					
					//cloud_plane = (res.at(0));
					Eigen::Vector4f centroid;
					pcl::compute3DCentroid(*cloud_plane, centroid);

					//transform pose to map
					geometry_msgs::Pose target;
					target.position.x=centroid(0);
					target.position.y=centroid(1);
					target.position.z=centroid(2);
					target.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);
					std::cout << target.position.x << " " << target.position.y << " " << target.position.z << std::endl;
					/*geometry_msgs::PoseStamped stampedPose;

					stampedPose.header.frame_id = cloud->header.frame_id;
					stampedPose.header.stamp = ros::Time(0);
					stampedPose.pose = pose_i;

					geometry_msgs::PoseStamped stampOut;
					listener.waitForTransform(cloud->header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
					listener.transformPose("mico_api_origin", stampedPose, stampOut);

					stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
					pose_pub.publish(stampOut);*/
				}
				else
					std::cout << "The service returned 0 point clouds." << std::endl;
			}
			else
				std::cout << "Failed to call the service. Is it running?" << std::endl;
			ros::spinOnce();
		}

	}
};
