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
#include <pcl/kdtree/kdtree_flann.h>


/* Author: Maxwell Svetlik
 * Current state: finds closest point to end effector through transforms and KdTree search
 * Todo: compute the transform of the orientation of the plane, so the EF can approach the plane normal
 * 
 * Write something (will require format of drawing demo code for drawing)
 */


double cur_x, cur_y, cur_z;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;



void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	geometry_msgs::PoseStamped current = msg;
	cur_x = current.pose.position.x;
	cur_y = current.pose.position.y;
	cur_z = current.pose.position.z;
}


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
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
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
					
					//Transform EF XYZ to /camera/depth_registered
					geometry_msgs::Pose ef_pose;
					ef_pose.position.x = cur_x;
					ef_pose.position.y = cur_y;
					ef_pose.position.z = cur_z;
					ef_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);
					geometry_msgs::PoseStamped ef_mico;

					ef_mico.header.frame_id = "mico_api_origin";
					ef_mico.header.stamp = ros::Time(0);
					ef_mico.pose = ef_pose;

					geometry_msgs::PoseStamped ef_camera;
					listener.waitForTransform("mico_api_origin", cloud_plane->header.frame_id, ros::Time(0), ros::Duration(3.0));
					listener.transformPose("mico_api_origin", ef_mico, ef_camera);
					
					
					//Find closest point to EF
					pcl::PointXYZ targetPoint;
					int K = 5; //K nearest neighbors
					std::vector<int> pointIdxNKNSearch(K);
					std::vector<float> pointNKNSquaredDistance(K);
					targetPoint.x = ef_camera.pose.position.x;
					targetPoint.y = ef_camera.pose.position.y;
					targetPoint.z = ef_camera.pose.position.z;
					
					pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
					kdtree.setInputCloud(cloud_plane);
					
					if(kdtree.nearestKSearch(targetPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
						//transform pose to mico
						geometry_msgs::Pose target;
						target.position.x = cloud_plane->points[pointIdxNKNSearch[0]].x;
						target.position.y = cloud_plane->points[pointIdxNKNSearch[0]].y;
						target.position.z = cloud_plane->points[pointIdxNKNSearch[0]].z;
						target.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);
						geometry_msgs::PoseStamped pose_in;

						pose_in.header.frame_id = cloud_plane->header.frame_id;
						pose_in.header.stamp = ros::Time(0);
						pose_in.pose = target;

						geometry_msgs::PoseStamped pose_out;
						listener.waitForTransform(cloud_plane->header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
						listener.transformPose("mico_api_origin", pose_in, pose_out);

						pose_out.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
						std::cout << "The target approach point in the arm frame: ";
						std::cout << pose_out.pose.position.x << " " << pose_out.pose.position.y << " " << pose_out.pose.position.z << std::endl;
						//pose_pub.publish(pose_out);
					}
					else
						std::cout << "Unable to find closest point." << std::endl;
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
