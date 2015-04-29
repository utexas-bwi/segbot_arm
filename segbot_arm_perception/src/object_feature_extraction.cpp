#include <iostream>
#include <fstream>
#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include "segbot_arm_perception/TableDetectionObjectExtraction.h"
#include "segbot_arm_perception/FeatureExtraction.h"

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum Label {
    RED,
    GREEN,
    BLUE
};



// Select mode
const bool kCaptureScene = false;
const bool kLoadScene = false;
const double kROSRate = 1.0; 	//refresh rate (Hz)
const std::string kFeatureDataFile = "train1";
const Label kLabel = RED;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);

sensor_msgs::PointCloud2 cloud_ros;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// Check if a file exist or not
bool file_exist(std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
    g_caught_sigint = true;
    ROS_INFO("Caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
}


void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	cloud_mutex.lock();

	//convert to PCL format
	pcl::fromROSMsg(*input, *cloud);

	//state that a new cloud is available
    new_cloud_available_flag = true;

	cloud_mutex.unlock();
}


int main(int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_button_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

    tf::TransformListener listener;

	//register ctrl-c
	signal(SIGINT, sig_handler);

	ros::Rate r(kROSRate);

	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_feature_extraction/cloud", 10);

    // Setup service client for table detection
    ros::ServiceClient table_srv_client = nh.serviceClient<segbot_arm_perception::TableDetectionObjectExtraction>("/segbot_arm_perception/table_detection_object_extraction_server");
    ros::ServiceClient feature_srv_client = nh.serviceClient<segbot_arm_perception::FeatureExtraction>("/segbot_arm_perception/feature_extraction_server");

	// Main loop:
	while (!g_caught_sigint && ros::ok()) {
		//collect messages
		ros::spinOnce();

        // TODO hack job for loading from disk
        if (kLoadScene) {
            new_cloud_available_flag = true;
        }
        if (new_cloud_available_flag) {
            new_cloud_available_flag = false;

            // Save pointcloud to disk
            if (kCaptureScene) {
                static int scene_count = 0;
                char input_char;

                toROSMsg(*cloud, cloud_ros);
                cloud_ros.header.frame_id = cloud->header.frame_id;
                ROS_INFO("Publishing cloud clusters...");
                cloud_pub.publish(cloud_ros);

                ROS_INFO("Save current button point cloud? [y/n]");
                std::cin >> input_char;
                if(input_char == 'y') {
                    pcl::PCDWriter writer;
                    std::stringstream ss;
                    ss << "scene_" << scene_count <<  ".pcd";
                    std::string pathname_write = ros::package::getPath("segbot_arm_perception") + "/pcd/" + ss.str();
                    // Changing file name until it doesn't overlap with existing point clouds
                    while (file_exist(pathname_write)) {
                        scene_count++;
                        ss.str("");
                        ss << "red_button_" << scene_count <<  ".pcd";
                        pathname_write = ros::package::getPath("segbot_arm_perception") + "/pcd/" + ss.str();
                    }

                    ROS_INFO("Saving %s...", ss.str().c_str());
                    writer.write<PointT>(pathname_write, *cloud, false);
                    scene_count++;
                }
                continue;
            }

            // Load pointcloud from disk
            if (kLoadScene) {
                pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT>);
                std::string pathNameRead = ros::package::getPath("segbot_arm_perception") + "/pcd/" + "scene_0.pcd";
                if (pcl::io::loadPCDFile<PointT> (pathNameRead, *cloud_temp) == -1) {  //* load the file
                    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                    return (-1);
                }
                cloud_temp->header.frame_id = cloud->header.frame_id;
                cloud = cloud_temp;
                toROSMsg(*cloud, cloud_ros);
                cloud_ros.header.frame_id = cloud->header.frame_id;
                ROS_INFO("Publishing cloud clusters...");
                cloud_pub.publish(cloud_ros);
            }

            // Prepare service call message
            segbot_arm_perception::TableDetectionObjectExtraction table_srv;
            // Pack service request
            toROSMsg(*cloud, table_srv.request.cloud);
            ROS_INFO("Calling table detection clusters extraction service...");
            if (table_srv_client.call(table_srv) && table_srv.response.is_plane_found) {
                PointCloudT::Ptr cloud_plane(new PointCloudT);
                std::vector<PointCloudT::Ptr > clusters_on_plane;
                float cloud_plane_coef[4];
                ROS_INFO("Table found");
                // Retrieve values
                for (int i = 0; i < table_srv.response.cloud_plane_coef.size(); i++) {
                    cloud_plane_coef[i] = table_srv.response.cloud_plane_coef[i];
                }
                fromROSMsg(table_srv.response.cloud_plane, *cloud_plane);
                // Find the closest cluster to the center
                int object_index = 0;
                float min_y_value = std::numeric_limits<float>::max();
                for (int i = 0; i < table_srv.response.cloud_clusters.size(); i++) {
                    Eigen::Vector4f centroid_i;
                    PointCloudT::Ptr temp_ptr(new PointCloudT);
                    fromROSMsg(table_srv.response.cloud_clusters[i], *temp_ptr);
                    clusters_on_plane.push_back(temp_ptr);
                    // Find min Y coordinate
                    pcl::compute3DCentroid(*temp_ptr, centroid_i);
                    if (fabs(centroid_i(0)) < min_y_value) {
                        min_y_value = fabs(centroid_i(0));
                        object_index = i;
                    }
                }

                segbot_arm_perception::FeatureExtraction feature_srv;
                if (clusters_on_plane.size() > 0) {
                    // Pack service request
                    toROSMsg(*clusters_on_plane[object_index], feature_srv.request.cloud);
                    ROS_INFO("Calling feature extraction service...");
                    if (feature_srv_client.call(feature_srv)) {
                        ROS_INFO("Feature vector received");
                        std::vector<double> feature_vector = feature_srv.response.feature_vector;
                        std::string pathname_write = ros::package::getPath("segbot_arm_perception") + "/feature_data/" + kFeatureDataFile;
                        std::ofstream write_feature_file;
                        write_feature_file.open(pathname_write.c_str());
                        char input_char;
                        std::cin >> input_char;
                        ROS_INFO("PRESS 'y' TO SAVE. PRESS '0' TO QUIT. PRESS ANY OTHER KEY TO SKIP");
                        if (input_char == 'y') {
                            write_feature_file << kLabel << " ";
                            for (int i = 0; i < feature_vector.size(); i++) {
                                write_feature_file << i << ":" << feature_vector[i] << " ";
                            }
                            write_feature_file << "\n";
                        } else if (input_char == '0') {
                            ROS_INFO("Saving");
                            write_feature_file.close();
                        } else {
                            ROS_INFO("Skipping");
                        }
                    } else {
                        ROS_INFO("Error: No feature vector");
                    }

                    // ROS_INFO("Publishing cloud clusters...");
                    // toROSMsg(*cloud_plane, cloud_ros);
                    // cloud_pub.publish(cloud_ros);
                } else {
                    ROS_INFO("No clusters found");
                }
            }
        }
		r.sleep();
	}
}
