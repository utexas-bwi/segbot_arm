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
    YELLOW = 1,
    RED,
    GREEN,
    BLUE
};

// Select mode
const bool kCaptureScene = true;
const bool kLoadScene = !kCaptureScene;
const double kROSRate = 1.0; 	//refresh rate (Hz)
const Label kLabel = YELLOW;

// Mutex: //
boost::mutex cloud_mutex;

// Global variables
bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
std::string data_file_name = "";
std::string pcd_dir_name = "";

ros::Publisher cloud_pub;
sensor_msgs::PointCloud2 cloud_ros;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// Check if a file exist or not
bool file_exist(std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("Caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	cloud_mutex.lock();

	//convert to PCL format
	pcl::fromROSMsg(*input, *cloud);

	//state that a new cloud is available
    new_cloud_available_flag = true;

	cloud_mutex.unlock();
}

void set_pcd_dir_name() {
    while (pcd_dir_name.empty()) {
        ROS_INFO("Output object id (dir) [e.g. \"01\"]: ");
        std::getline (std::cin, pcd_dir_name);
        pcd_dir_name += "/";
        ROS_INFO("pcd_dir_name = %s", pcd_dir_name.c_str());
    }
}

// Load cloud with cloud stored in pcd_path
bool load_cloud_from_disk(PointCloudT::Ptr cloud) {
    static int pcd_count = 0;
    set_pcd_dir_name();

    std::stringstream ss;
    ss << pcd_count <<  ".pcd";
    std::string pathname_read = ros::package::getPath("segbot_arm_perception") + "/pcd/" + pcd_dir_name + ss.str();
    if (pcl::io::loadPCDFile<PointT> (pathname_read, *cloud) == -1) {  // load the file
        return false;
    }
    pcd_count++;

    return true;
}


bool save_cloud_to_disk(PointCloudT::Ptr cloud) {
    static int scene_count = 0;
    char input_char;
    // Set directory for each run of the program. Directory must already exist.
    set_pcd_dir_name();

    ROS_INFO("Save current cloud (size %d)? [y/n]", (int)cloud->points.size());
    std::cin >> input_char;
    if(input_char == 'y') {
        pcl::PCDWriter writer;
        std::stringstream ss;
        ss << scene_count <<  ".pcd";
        std::string pathname_write = ros::package::getPath("segbot_arm_perception") + "/pcd/" + pcd_dir_name + ss.str();
        // Changing file name until it doesn't overlap with existing point clouds
        while (file_exist(pathname_write)) {
            scene_count++;
            ss.str("");
            ss << scene_count <<  ".pcd";
            pathname_write = ros::package::getPath("segbot_arm_perception") + "/pcd/" + pcd_dir_name + ss.str();
        }

        ROS_INFO("Saving %s...", pathname_write.c_str());
        writer.write<PointT>(pathname_write, *cloud, false);
        scene_count++;
    }
    return true;
}


// Write the feature vector to disk in libSVM format. See kFeatureDataFile for file name
bool write_feature_to_disk(std::vector<double>& feature_vector) {
    // Set filename for each run of the program
    while (data_file_name.empty()) {
        ROS_INFO("Output file name: ");
        std::getline (std::cin, data_file_name);
        ROS_INFO("data_file_name = %s", data_file_name.c_str());
    }
    ROS_INFO("PRESS 'y' TO SAVE. PRESS '0' TO QUIT. PRESS ANY OTHER KEYS TO SKIP");
    char input_char;
    std::cin >> input_char;
    if (input_char == 'y') {
        bool positive_example;
        do {
            ROS_INFO("Positive 'p' or negative 'n' example?");
            std::cin >> input_char;
            if (input_char == 'p') {
                positive_example = true;
            } else {
                positive_example = false;
            }
        } while (input_char != 'p' && input_char != 'n');

        std::string pathname_write = ros::package::getPath("segbot_arm_perception") + "/feature_data/" + data_file_name;
        ROS_INFO("Path = %s", pathname_write.c_str());
        std::ofstream write_feature_file;
        write_feature_file.open(pathname_write.c_str(), std::ios::out | std::ios::app);
        if (write_feature_file.is_open()) {
            ROS_INFO("Writing to file...");
            if (positive_example) {
                write_feature_file << "+";
            } else {
                write_feature_file << "-";
            }
            write_feature_file << kLabel << " ";
            for (int i = 0; i < feature_vector.size(); i++) {
                write_feature_file << i << ":" << feature_vector[i] << " ";
            }
            write_feature_file << "\n";
            write_feature_file.close();
        } else {
            ROS_ERROR("Cannot open file");
        }

    } else if (input_char == '0') {
        ROS_INFO("Shutting down...");
        ros::shutdown();
        exit(1);
    } else {
        ROS_INFO("Skipping");
        return false;
    }

    return true;
}


int main(int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_button_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

    tf::TransformListener listener;
    int frame_count = 0;
	//register ctrl-c
	signal(SIGINT, sig_handler);

	ros::Rate r(kROSRate);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_feature_extraction/cloud", 10);

    // Setup service client for table detection
    ros::ServiceClient table_srv_client = nh.serviceClient<segbot_arm_perception::TableDetectionObjectExtraction>("/segbot_arm_perception/table_detection_object_extraction_server");
    ros::ServiceClient feature_srv_client = nh.serviceClient<segbot_arm_perception::FeatureExtraction>("/segbot_arm_perception/feature_extraction_server");

	// Main loop:
	while (!g_caught_sigint && ros::ok()) {
		//collect messages
		ros::spinOnce();

        // Process new cloud if there is one
        if (new_cloud_available_flag) {
            new_cloud_available_flag = false;
            // Prepare table service call message
            segbot_arm_perception::TableDetectionObjectExtraction table_srv;
            toROSMsg(*cloud, table_srv.request.cloud);
            ROS_INFO("Calling table detection clusters extraction service...");
            if (table_srv_client.call(table_srv) && table_srv.response.is_plane_found) {
                ROS_INFO("Table found");
                PointCloudT::Ptr cloud_plane(new PointCloudT);
                std::vector<PointCloudT::Ptr> clusters_on_plane;
                float cloud_plane_coef[4];
                // Retrieve values
                for (int i = 0; i < table_srv.response.cloud_plane_coef.size(); i++) {
                    cloud_plane_coef[i] = table_srv.response.cloud_plane_coef[i];
                }
                fromROSMsg(table_srv.response.cloud_plane, *cloud_plane);
                int object_index = 0;
                float min_x_value = std::numeric_limits<float>::max();
                for (int i = 0; i < table_srv.response.cloud_clusters.size(); i++) {
                    Eigen::Vector4f centroid_i;
                    PointCloudT::Ptr temp_ptr(new PointCloudT);
                    fromROSMsg(table_srv.response.cloud_clusters[i], *temp_ptr);
                    clusters_on_plane.push_back(temp_ptr);
                    // Find the closest cluster to the center (horizontal, x)
                    pcl::compute3DCentroid(*temp_ptr, centroid_i);
                    if (fabs(centroid_i(0)) < min_x_value) {
                        min_x_value = fabs(centroid_i(0));
                        object_index = i;
                    }
                }

                // Burn a few frames
                if (frame_count < 5) {
                    frame_count++;
                    continue;
                }

                // TODO: better integration. right now it skips over the last loop
                // Save center ponit cloud
                if (kCaptureScene && !kLoadScene && clusters_on_plane.size() > 0) {
                    ROS_WARN("Capture Scene mode");
                    PointCloudT::Ptr temp_cloud = clusters_on_plane[object_index];
                    ROS_INFO("Publishing cloud clusters size %d...", (int)temp_cloud->points.size());
                    toROSMsg(*temp_cloud, cloud_ros);
                    cloud_pub.publish(cloud_ros);
                    save_cloud_to_disk(clusters_on_plane[object_index]);
                    continue;
                }

                // Load pointcloud from disk
                if (kLoadScene && !kCaptureScene) {
                    ROS_WARN("Load Scene mode");
                    PointCloudT::Ptr temp_cloud(new PointCloudT);
                    if (!load_cloud_from_disk(temp_cloud)) {
                        ROS_WARN("Last file in dir reached");
                        while(1);
                    }

                    ROS_INFO("Publishing cloud clusters size %d...", (int)temp_cloud->points.size());
                    temp_cloud->header.frame_id = cloud->header.frame_id;
                    toROSMsg(*temp_cloud, cloud_ros);
                    cloud_pub.publish(cloud_ros);
                    ROS_INFO("Press ENTER to continue");
                    std::cin.ignore();
                    continue;
                }
////////////////////

                // Prepare feature service call message
                segbot_arm_perception::FeatureExtraction feature_srv;
                toROSMsg(*clusters_on_plane[object_index], feature_srv.request.cloud);
                ROS_INFO("Calling feature extraction service...");
                // Find feature if there is more than 1 cluster on table
                if (clusters_on_plane.size() > 0 && feature_srv_client.call(feature_srv)) {
                    ROS_INFO("Feature vector received");

                    std::vector<double> feature_vector = feature_srv.response.feature_vector;
                    // Write feature to disk
                    write_feature_to_disk(feature_vector);

                    // ROS_INFO("Publishing cloud clusters...");
                    // toROSMsg(*cloud_plane, cloud_ros);
                    // cloud_pub.publish(cloud_ros);
                } else {
                    ROS_WARN("Error: No clusters found or failed to retrieve feature vector");
                }
            }
            frame_count++;
        }
		r.sleep();
	}
}
