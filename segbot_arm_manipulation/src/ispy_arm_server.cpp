#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_manipulation/iSpyTouch.h"
#include "segbot_arm_manipulation/iSpyDetectTouch.h"

// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/distances.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>

#define NUM_JOINTS 8 //6+2 for the arm
#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

//some defines for behaviors
#define TOUCH_POSE_HEIGHT 0.045
#define TIMEOUT_THRESHOLD 30.0 //30 seconds

#define PI 3.14159265

//threshold used to determine if detect change indicates 
//a contact with the object
#define TOUCH_DISTANCE_THRESHOLD 0.05 //5 cm

using namespace std;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// robot state information
sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;
geometry_msgs::PoseStamped home_pose;


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//cloud representing change
PointCloudT::Ptr cloud_change (new PointCloudT);
boost::mutex change_cloud_mutex;
bool new_change_cloud_detected = false;

//clients
ros::ServiceClient client_start_change;
ros::ServiceClient client_stop_change;

//publishers
ros::Publisher pub_velocity;
ros::Publisher change_cloud_debug_pub; //publishes the filtered change cloud 
ros::Publisher detected_change_cloud_pub; //publishes the cluster detected to be close to the object
sensor_msgs::PointCloud2 cloud_ros;
pcl::PCLPointCloud2 pc_target;


/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		//ROS_INFO("Heard arm joint states!");
		current_state = *input;
		heardJoinstState = true;
	}
  //ROS_INFO_STREAM(current_state);
}

//Joint state cb
void joint_effort_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_effort = *input;
  //ROS_INFO_STREAM(current_effort);
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
 // ROS_INFO("Heard arm tool pose!");
  current_pose = msg;
  heardPose = true;
  //  ROS_INFO_STREAM(current_pose);
}

//Joint state cb
void fingers_cb (const jaco_msgs::FingerPosition msg) {
  current_finger = msg;
}


void listenForArmData(float rate){
	heardPose = false;
	heardJoinstState = false;
	ros::Rate r(rate);
	
	while (ros::ok()){
		ROS_INFO("Listening for arm data...");
		
		ros::spinOnce();
		
		if (heardPose && heardJoinstState)
			return;
		
		r.sleep();
	}
}


std::vector<PointCloudT::Ptr > computeClusters(PointCloudT::Ptr in){
	std::vector<PointCloudT::Ptr > clusters;

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (in->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
  }
  
  return clusters;
}

geometry_msgs::PoseStamped createTouchPose(PointCloudT::Ptr blob, Eigen::Vector4f plane_coefficients,std::string frame_id){
	//basically, find the point furthers away from the plane -- that's the top of the object
	double max_distance = -1000.0;
	int max_index = -1;
	
	//first, we find the point in the blob closest to the plane
	for (unsigned int i = 0; i < blob->points.size(); i++){
		pcl::PointXYZ p_i;
		p_i.x=blob->points.at(i).x;
		p_i.y=blob->points.at(i).y;
		p_i.z=blob->points.at(i).z;

		double distance = pcl::pointToPlaneDistance(p_i, plane_coefficients);
		

		if (distance > max_distance){
			max_distance = distance;
			max_index = i;
		}			
	}
	
	//now create the pose
	geometry_msgs::PoseStamped pose_st;
	pose_st.header.stamp = ros::Time(0);
	pose_st.header.frame_id =frame_id.c_str();
	
	pose_st.pose.position.x = blob->points.at(max_index).x;
	pose_st.pose.position.y = blob->points.at(max_index).y;
	pose_st.pose.position.z = blob->points.at(max_index).z;
	//this orientation here doesn't matter, it just has to be valid in order for the transform to work
	pose_st.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	
	//tranform pose into arm frame of reference
	tf::TransformListener listener;
	listener.waitForTransform(pose_st.header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
	listener.transformPose("mico_api_origin", pose_st, pose_st);
			
	//decide on orientation (horizontal palm)
	pose_st.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,PI/2);
	
	//add a bit of z 
	pose_st.pose.position.z+=TOUCH_POSE_HEIGHT;
	
	//ROS_INFO("Touch pose:");
	//ROS_INFO_STREAM(pose_st);
	
	return pose_st;
}


void moveToPoseCarteseanVelocity(geometry_msgs::PoseStamped pose_st){
	listenForArmData(30.0);
	
	int rateHertz = 40;
	geometry_msgs::TwistStamped velocityMsg;
	
	
	ros::Rate r(rateHertz);
	
	float theta = 0.05;
	
	float constant_m = 2.0;
	
	ROS_INFO("Starting movement...");
	
	while (ros::ok()){
		
		float dx = constant_m*( - current_pose.pose.position.x + pose_st.pose.position.x );
		float dy = constant_m*(- current_pose.pose.position.y + pose_st.pose.position.y);
		float dz = constant_m*(- current_pose.pose.position.z + pose_st.pose.position.z);
		
		if (fabs(dx) < theta && fabs(dy) < theta && fabs(dz) < theta){
			//we reached the position, exit
			break;
		}
		
		velocityMsg.twist.linear.x = dx;
		velocityMsg.twist.linear.y = dy;
		velocityMsg.twist.linear.z = dz;
		
		velocityMsg.twist.angular.x = 0.0;
		velocityMsg.twist.angular.y = 0.0;
		velocityMsg.twist.angular.z = 0.0;
		
		
		pub_velocity.publish(velocityMsg);
		ros::spinOnce();
		//ROS_INFO("Published cartesian vel. command");
		r.sleep();
	}
	
	ROS_INFO("Ending movement...");

}



void startChangeDetection(){
	
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;

	if(client_start_change.call(req, res)){
 		ROS_INFO("Call to change detect node successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call to change detect node failed. Terminating.");
 		//ros::shutdown();
 	}
	
}


void stopChangeDetection(){
	
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;

	if(client_stop_change.call(req, res)){
 		ROS_INFO("Call to change detect node successful. Response:");
 		ROS_INFO_STREAM(res);
 	} else {
 		ROS_ERROR("Call to change detect node failed. Terminating.");
 		//ros::shutdown();
 	}
	
}

void change_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	change_cloud_mutex.lock ();
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud_change);



	new_change_cloud_detected = true;
	
	change_cloud_mutex.unlock ();
}

bool detect_touch_cb(segbot_arm_manipulation::iSpyDetectTouch::Request &req, 
					segbot_arm_manipulation::iSpyDetectTouch::Response &res)
{
	
	//Step 1: extract perceptual information and compute locations at the top of the objects
	
	//convert object clouds to PCL format
	std::vector<PointCloudT::Ptr > detected_objects;
	for (unsigned int i = 0; i <req.objects.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(req.objects.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		detected_objects.push_back(object_i);
	}
	
	//get plane
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=req.cloud_plane_coef[i];
		
	//for each object, compute the touch pose; also, extract the highest point from the table
	std::vector<geometry_msgs::PoseStamped> touch_poses;
	double highest_z = 0.0;
	for (int i = 0; i < detected_objects.size(); i++){
		//generate touch pose for the object
		geometry_msgs::PoseStamped touch_pose_i = createTouchPose(detected_objects.at(i),plane_coef_vector,
												req.objects.at(i).header.frame_id);
			
		if (touch_pose_i.pose.position.z > highest_z){
			highest_z = touch_pose_i.pose.position.z ;
		}
		
		touch_poses.push_back(touch_pose_i);
	}
	
	//Step 2: start change detection server
	startChangeDetection();
	
	//Step 3: process each cloud
	//now detect change close to objects
	double rate = 10.0;
	ros::Rate r(rate);
	
	double elapsed_time = 0.0;
	
	while (ros::ok()){
		ros::spinOnce();
		
		elapsed_time += 1.0/rate;
		
		if (new_change_cloud_detected){
			
			//first, Z filter on the cloud
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (cloud_change);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.0, 1.15);
			pass.filter (*cloud_change);
			
			//next, perform statistical outlier filter
			pcl::StatisticalOutlierRemoval<PointT> sor;
			sor.setInputCloud (cloud_change);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud_change);
			
			//publish filtered cloud for debugging purposes
			//pcl::toPCLPointCloud2(*cloud_change,pc_target);
			//pcl_conversions::fromPCL(pc_target,cloud_ros);
			//change_cloud_debug_pub.publish(cloud_ros);
			
			
			//next, compute change clusters
			std::vector<PointCloudT::Ptr > change_clusters = computeClusters(cloud_change);
			
			//find largest cluster
			ROS_INFO("Found %i change clusters...",(int)change_clusters.size());
				
			if (change_clusters.size() > 0){
			
				int max_num_points = -1;
				int largest_index = -1;
				for (unsigned int i = 0; i < change_clusters.size(); i++){
					if ((int)change_clusters.at(i)->points.size() > max_num_points){
						max_num_points = (int)change_clusters.at(i)->points.size();
						largest_index = i;
					}
				}
				
				//publish filtered cloud for debugging purposes
				pcl::toPCLPointCloud2(*change_clusters.at(largest_index),pc_target);
				pcl_conversions::fromPCL(pc_target,cloud_ros);
				cloud_ros.header.frame_id = cloud_change->header.frame_id;
				change_cloud_debug_pub.publish(cloud_ros);
				
				
				
				
				
				//find the smallest distance between any cluster and any object top
				float min_distance = 1000.0;
				int min_object_index = -1;
				int min_change_cluster_index = -1;
				for (unsigned int i = 0; i < change_clusters.size(); i++){
					
					
					for (unsigned int j = 0; j < touch_poses.size();j++){
						Eigen::Vector4f touch_pos_j;
						touch_pos_j(0)=touch_poses.at(j).pose.position.x;
						touch_pos_j(1)=touch_poses.at(j).pose.position.y;
						touch_pos_j(2)=touch_poses.at(j).pose.position.z;
						touch_pos_j(3)=0.0;//unused

						//this variable will be set to the closest distance between
						//the touch pose (the top of the object) and any point in the 
						//j^th change cluster
						float distance_ij = 1000.0;
						
						for (unsigned int k = 0; k < change_clusters.at(i)->points.size(); k++){
							Eigen::Vector4f t;
							t(0) = change_clusters.at(i)->points.at(k).x;
							t(1) = change_clusters.at(i)->points.at(k).y;
							t(2) = change_clusters.at(i)->points.at(k).z;
							t(3) = 0.0;//unused
							
							float dist_jk = pcl::distances::l2(touch_pos_j,t);
							
							if (dist_jk < distance_ij){
								distance_ij = dist_jk;
							}
						}
						
						//now check if this is the min distance between a change cluster and a touch pose
						if (distance_ij < min_distance){
							min_distance = distance_ij;
							min_object_index = j;
							min_change_cluster_index = i;
						}
					}
				}
				
				ROS_INFO("[ispy_arm_server.cpp] min. distance = %f between object %i and change cluster %i",min_distance,min_object_index,min_change_cluster_index);
				
				//now check if the min_distance is below a threshold and if so, report the detection
				
				if (min_distance < TOUCH_DISTANCE_THRESHOLD){
					res.detected_touch_index = min_object_index;
					res.success = true;
					
					//publish the change cluster
					pcl::toPCLPointCloud2(*change_clusters.at(min_change_cluster_index),pc_target);
					pcl_conversions::fromPCL(pc_target,cloud_ros);
					detected_change_cloud_pub.publish(cloud_ros);
					
					
					return true;
				}
			}
			
			
			
			new_change_cloud_detected = false;
		}
		
		//timeout condition
		if (elapsed_time > TIMEOUT_THRESHOLD){
				res.detected_touch_index = -1;
				res.success = false;
				return true;
		}
		
		r.sleep();
	}
	
	return true;
}


bool touch_object_cb(segbot_arm_manipulation::iSpyTouch::Request &req, 
					segbot_arm_manipulation::iSpyTouch::Response &res)
{
	
	//convert object clouds to PCL format
	std::vector<PointCloudT::Ptr > detected_objects;
	for (unsigned int i = 0; i <req.objects.size(); i++){
		PointCloudT::Ptr object_i (new PointCloudT);
		pcl::PCLPointCloud2 pc_i;
		pcl_conversions::toPCL(req.objects.at(i),pc_i);
		pcl::fromPCLPointCloud2(pc_i,*object_i);
		detected_objects.push_back(object_i);
	}
	
	//get plane
	Eigen::Vector4f plane_coef_vector;
	for (int i = 0; i < 4; i ++)
		plane_coef_vector(i)=req.cloud_plane_coef[i];
	
	ROS_INFO("Received touch object request for object %i",req.touch_index);
	
	if (req.touch_index != -1){
		
		ROS_INFO("Computing touch poses for all objects...");
		
		//for each object, compute the touch pose; also, extract the highest point from the table
		std::vector<geometry_msgs::PoseStamped> touch_poses;
		double highest_z = 0.0;
		for (int i = 0; i < detected_objects.size(); i++){
			//generate touch pose for the object
			geometry_msgs::PoseStamped touch_pose_i = createTouchPose(detected_objects.at(i),plane_coef_vector,
													req.objects.at(i).header.frame_id);
			
			//if (touch_pose_i.pose.position.z > 0.05)
			//	touch_poses.push_back(touch_pose_i);
			
			if (touch_pose_i.pose.position.z > highest_z){
				highest_z = touch_pose_i.pose.position.z ;
			}
			
			touch_poses.push_back(touch_pose_i);
		}
		
		ROS_INFO("...done!");
		
		ROS_INFO("Waiting for arm data...");
		
		//store current pose
		/*listenForArmData(10.0);
		geometry_msgs::PoseStamped start_pose = current_pose;
		ROS_INFO("...done");*/
		
		
		
		geometry_msgs::PoseStamped touch_pose_i = touch_poses.at(req.touch_index);
			
		//before we get there, first go a bit above
		double z_above = highest_z+0.2;
			
		geometry_msgs::PoseStamped touch_approach = touch_pose_i;
		touch_approach.pose.position.z = z_above;
		
		ROS_INFO("Moving to approach pose...");	
		//first approach the object from the top
		moveToPoseCarteseanVelocity(touch_approach);
			
		//now touch it
		ROS_INFO("Moving to touch pose...");	
		moveToPoseCarteseanVelocity(touch_pose_i);
	}
	else { //retract
		//store current pose
		listenForArmData(10.0);
		
		geometry_msgs::PoseStamped touch_approach = current_pose;
		touch_approach.pose.position.z = current_pose.pose.position.z+0.07;
		
		moveToPoseCarteseanVelocity(touch_approach);
		moveToPoseCarteseanVelocity(home_pose);
		
	}
	
	ROS_INFO("Finished touch object request for object %i",req.touch_index);
	
	
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "ispy_arm_server");
	ros::NodeHandle n;
	
	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);

	//create subscriber to joint torques
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	 
	//subscriber for change cloud
	ros::Subscriber sub_change_cloud = n.subscribe("/segbot_arm_table_change_detector/cloud",1,change_cloud_cb);  
	 
	//publish velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	//cloud publisher
	change_cloud_debug_pub = n.advertise<sensor_msgs::PointCloud2>("ispy_arm_server/change_cloud_filtered", 10);
	detected_change_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("ispy_arm_server/detected_touch_cloud", 10);
	
	
	//declare service for touching objects
	ros::ServiceServer service_touch = n.advertiseService("ispy/touch_object_service", touch_object_cb);
	
	//service for detecting when a human touches an object
	ros::ServiceServer service_detect = n.advertiseService("ispy/human_detect_touch_object_service", detect_touch_cb);
	
	//clients
	client_start_change = n.serviceClient<std_srvs::Empty> ("/segbot_arm_table_change_detector/start");
	client_stop_change = n.serviceClient<std_srvs::Empty> ("/segbot_arm_table_change_detector/stop");
	
	
	
	//store the home arm pose
	listenForArmData(10.0);
	home_pose = current_pose;
	
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//refresh rate
	double ros_rate = 10.0;
	ros::Rate r(ros_rate);

	ros::spin();

	// Main loop:
	/*while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();

		r.sleep();

	}*/
};
