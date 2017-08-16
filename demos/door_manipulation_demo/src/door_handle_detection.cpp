#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>    
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/impl/transforms.hpp>
//including package services 
#include "door_manipulation_demo/door_perception.h"
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
//tf 
#include <tf/transform_listener.h>
#define PI 3.14159265

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_aggregated (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_centroid (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
PointCloudT::Ptr empty_cloud (new PointCloudT);
std::vector<PointCloudT::Ptr > clusters;
std::vector<PointCloudT::Ptr > clusters_on_plane;
geometry_msgs::Quaternion plane_coeff;

sensor_msgs::PointCloud2 cloud_ros;

// used to see if door has moved
sensor_msgs::JointState current_state;
Eigen::Vector4f plane_coefficients_compare;
bool firstLook = true;

Eigen::Vector4f centroid;

ros::Publisher cloud_pub;
ros::Publisher door_cloud_pub;
ros::Publisher move_point;
ros::Publisher goal_pub;
ros::Publisher plane_coeff_pub;
PointCloudT::Ptr cloud_costmap (new PointCloudT);


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

double plane_distance_tolerance = 0.08;
double plane_max_distance_tolerance = 0.03;
double cluster_extraction_tolerance = 0.05;

bool collecting_cloud = false;

// Check if a file exist or not
bool file_exist(std::string& name){
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

/* what happens when ctr-c is pressed */
void sig_handler(int sig){
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

// used to get current state to see if door has moved
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input){
    //NUM JOINTS IS 8
    if (input->position.size() == 8)
        current_state = *input;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
        cloud_mutex.lock ();
        //convert to PCL format
        pcl::fromROSMsg (*input, *cloud);
        //state that a new cloud is available
        new_cloud_available_flag = true;
         cloud_mutex.unlock ();
}

bool filter(PointCloudT::Ptr blob, Eigen::Vector4f plane_coefficients, double tolerance_min, double tolerance_max){
    
    double min_distance = 1000.0;
    double max_distance = -1000.0;
    
    //first, we find the point in the blob closest to the plane
    for (unsigned int i = 0; i < blob->points.size(); i++){
        pcl::PointXYZ p_i;
        p_i.x=blob->points.at(i).x;
        p_i.y=blob->points.at(i).y;
        p_i.z=blob->points.at(i).z;

        double distance = pcl::pointToPlaneDistance(p_i, plane_coefficients);
        
        if (distance < min_distance)
            min_distance = distance;
        
        if (distance > max_distance)
            max_distance = distance;
        
    }
    if (min_distance > tolerance_min){
        return false;
    }
    else if (max_distance < 0.8*tolerance_max){
        return false;   
    }
    
    ROS_INFO("\nMin Distance to plane for cluster with %i points: %f",(int)blob->points.size(),min_distance);
    ROS_INFO("Max Distance to plane for cluster with %i points: %f",(int)blob->points.size(),max_distance);
    return true;
}

void computeClusters(PointCloudT::Ptr in, double tolerance){
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (tolerance); // 2cm
    //filter by door handle size assumed facing door head on and only have points of this size
    ec.setMinClusterSize (000);
    ec.setMaxClusterSize (800);
    ec.setSearchMethod (tree);
    ec.setInputCloud (in);
    ec.extract (cluster_indices);
    clusters.clear();
    
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        PointCloudT::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (in->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }
}

void waitForCloud(){
    ros::Rate r(30);
    collecting_cloud = true;
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
        if (new_cloud_available_flag){
            new_cloud_available_flag = false;
            break;
        }
    }
    collecting_cloud = false;
}

/* collects a cloud by aggregating k successive frames */
void waitForCloudK(int k){
    ros::Rate r(30);
    cloud_aggregated->clear();
    int counter = 0;
    collecting_cloud = true;
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
        if (new_cloud_available_flag){
            *cloud_aggregated+=*cloud;
            new_cloud_available_flag = false;
            counter ++;
            if (counter >= k){
                cloud_aggregated->header = cloud->header;
                break;
            }
        }
    }
    collecting_cloud = false;
}

bool seg_cb(door_manipulation_demo::door_perception::Request &req, door_manipulation_demo::door_perception::Response &res){
    //listener for transforrms
    tf::TransformListener listener;
    
    ROS_INFO("entered seg_cb");
    //get the point cloud by aggregating k successive input clouds
    waitForCloudK(15);
    cloud = cloud_aggregated;
    ROS_INFO("got cloud");
    double filter_z = 1.15;
    
    // Apply z filter -- we don't care for anything X m away in the z direction
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, filter_z);
    pass.filter (*cloud);
    ROS_INFO("before segmentation");
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.0025f, 0.0025f, 0.0025f);
    vg.filter (*cloud_filtered);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    // Extract the plane
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    //extract everything else
    extract.setNegative (true);
    extract.filter (*cloud_blobs);
    //get the plane coefficients
    Eigen::Vector4f plane_coefficients;
    plane_coefficients(0)=coefficients->values[0];
    plane_coefficients(1)=coefficients->values[1];
    plane_coefficients(2)=coefficients->values[2];
    plane_coefficients(3)=coefficients->values[3];
    
    ROS_INFO("Planar coefficients: %f, %f, %f, %f",
        plane_coefficients(0),plane_coefficients(1),plane_coefficients(2),  plane_coefficients(3));
    plane_coeff.x = plane_coefficients(0);
    plane_coeff.y = plane_coefficients(1);
    plane_coeff.z = plane_coefficients(2);
    plane_coeff.w = plane_coefficients(3);
    plane_coeff_pub.publish(plane_coeff);
    
    bool plane_is_not_vertical = plane_coefficients(1) > -.5 && plane_coefficients(3) > 0;
    //Step 3: Eucledian Cluster Extraction
    computeClusters(cloud_blobs,cluster_extraction_tolerance);
    ROS_INFO("Found %i clusters eucldian.",(int)clusters.size());
    clusters_on_plane.clear();
    for (unsigned int i = 0; i < clusters.size(); i++){
        if (filter(clusters.at(i),plane_coefficients,plane_distance_tolerance,plane_max_distance_tolerance)){
            clusters_on_plane.push_back(clusters.at(i));
        }
    }
    
    ROS_INFO("Found %i clusters on the plane after restrains.",(int)clusters_on_plane.size());
    //fill in response
    //plane cloud and coefficient
    pcl::toROSMsg(*cloud_plane,res.cloud_plane);
    res.cloud_plane.header.frame_id = cloud->header.frame_id;
    for (int i = 0; i < 4; i ++){
        res.cloud_plane_coef[i] = plane_coefficients(i);
    }
    //check if valid information
    if(clusters_on_plane.size() < 1 ){
        ROS_INFO("Found 0 clusters or plane isn't vertical did not continue");  
    } else {
        ROS_INFO("Picked largest cluster");
        pcl::toROSMsg(*clusters_on_plane.at(0),cloud_ros);
        cloud_ros.header.frame_id = cloud->header.frame_id;
        res.cloud_cluster = cloud_ros;
        //TO DO: this may not always be the case
        res.is_plane_found = true;
        ROS_INFO("passed second filer");
        //for debugging purposes
        //now, put the clouds in cluster_on_plane in one cloud and publish it
        cloud_blobs->clear();
    }
    for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
        *cloud_blobs += *clusters_on_plane.at(i);
    }
    
    //To::DO figure out values of 180
    ROS_INFO("Publishing debug cloud...");
    //get centroid and move it up .1 m 
    //used to get goal xyz
    pcl::compute3DCentroid(*cloud_blobs,centroid);
    //get pose
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = centroid.x();
    goal.pose.position.y = centroid.y();
    goal.pose.position.z = centroid.z();
    //get it flat (180 degress)
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    goal.header.frame_id = cloud_ros.header.frame_id;
    //mext transform pose into arm frame of reference and set orientation
    try{
        listener.waitForTransform(cloud_ros.header.frame_id,  "mico_link_origin",  ros::Time(0), ros::Duration(5.0) );
        listener.transformPose("mico_api_origin", goal, goal);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    //set orientation after transforming into arm frame of reference
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.54,0,1.54);
    //publish the two goals to get it to push the goor
    goal_pub.publish(goal);
    return true;
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "segbot_arm_door_handle_detector");
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    std::string param_topic = "/xtion_camera/depth_registered/points";
    ros::Subscriber sub = n.subscribe (param_topic, 1, cloud_cb);
    
    //create subscriber to joint angles
    ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
    
    //debugging publisher
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("door_handle_detection/cloud", 1);
    door_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("door_handle_detection/plane_cloud", 1);
    plane_coeff_pub = n.advertise<geometry_msgs::Quaternion>("plane_coeff", 1);
    
    //service
    ros::ServiceServer service = n.advertiseService("door_handle_detection/door_perception", seg_cb);
    ros::ServiceClient client = n.serviceClient<door_manipulation_demo::door_perception>("door_perception");
    
    //move_point = n.advertise<PCLCloudXYZ> ("point_to_send/cloud", 1);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_to_go", 1);
    
    //refresh rate
    double ros_rate = 3.0;
    ros::Rate r(ros_rate);
    while (!g_caught_sigint && ros::ok()){
        //collect messages
        ros::spinOnce();
        r.sleep();
    }
};
