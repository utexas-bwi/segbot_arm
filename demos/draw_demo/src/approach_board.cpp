#include <signal.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include "segbot_arm_perception/PlanarSegmentation.h"
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


/* Author: Maxwell Svetlik
 * Current state: finds centroid and transforms frame
 * Todo: find closest point to end effector, transform frame, approach
 * will require format of drawing demo code for drawing. 
 * /


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
					geometry_msgs::PoseStamped pose_in;

					pose_in.header.frame_id = cloud_plane->header.frame_id;
					pose_in.header.stamp = ros::Time(0);
					pose_in.pose = target;

					geometry_msgs::PoseStamped pose_out;
					listener.waitForTransform(cloud_plane->header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
					listener.transformPose("mico_api_origin", pose_in, pose_out);

					pose_out.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
					std::cout << pose_out.pose.position.x << " " << pose_out.pose.position.y << " " << pose_out.pose.position.z << std::endl;
					//pose_pub.publish(pose_out);
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
