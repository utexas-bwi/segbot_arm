#include <iostream>
#include <fstream>
#include <math.h>
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
#include <pcl/impl/point_types.hpp>

// PCL specific includes
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
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
const bool kCaptureScene = false;
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


// what happens when ctr-c is pressed
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
        ROS_INFO("Output object id (dir) [e.g. \"01\"or ""]: ");
        std::getline (std::cin, pcd_dir_name);
        pcd_dir_name += "/";
        ROS_INFO("pcd_dir_name = %s", pcd_dir_name.c_str());
    }
}


// Retrieve all pcb file path name from pcb/object_dir recurisvely
// Leave object_dir as blank to get all files for pcb/
std::vector<boost::filesystem::path> get_pcd_path_list (std::string current_dir) {
    using namespace boost::filesystem;
    std::vector<path> pcd_path_list;
    path current_path(current_dir.c_str());

    if (!exists(current_path)) {
        ROS_WARN("Path %s doesn't exist.", current_dir.c_str());
        return pcd_path_list;
    }
    directory_iterator end_itr; // default construction yields past-the-end
    for (directory_iterator itr(current_path); itr != end_itr; itr++) {
        if (is_regular_file(itr->status())) {
            pcd_path_list.push_back(itr->path());
        } else if (is_directory(itr->status())) {
            std::vector<path> temp_path_list = get_pcd_path_list(itr->path().string());
            // Concatentate vectors
            pcd_path_list.insert(pcd_path_list.end(), temp_path_list.begin(), temp_path_list.end());
        }
    }

    std::sort(pcd_path_list.begin(), pcd_path_list.end());

    return pcd_path_list;
}

// Load cloud with cloud stored in pcd_path
bool load_cloud_from_disk(PointCloudT::Ptr cloud, std::string cloud_path) {
    if (pcl::io::loadPCDFile<PointT>(cloud_path, *cloud) == -1) {  // load the file
        return false;
    }
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
bool write_feature_to_disk_libSVM(std::vector<double>& feature_vector) {
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


// Write the feature vector to disk in libSVM format. See kFeatureDataFile for file name
// It also writes the object id corresponding to the feature to another file
bool write_feature_to_disk_csv(std::vector<double>& feature_vector,
                               std::string feature_object_index) {
    // Set filename for each run of the program
    while (data_file_name.empty()) {
        ROS_INFO("Output file name: ");
        std::getline (std::cin, data_file_name);
    }

    std::string pathname_write = ros::package::getPath("segbot_arm_perception") + "/feature_data_csv/" + data_file_name + ".csv";
    std::string pathname_object_index_write = ros::package::getPath("segbot_arm_perception") + "/feature_data_csv/" + data_file_name + "_object_index.csv";
    ROS_INFO("Write path = %s", pathname_write.c_str());
    std::ofstream write_feature_file;
    std::ofstream write_feature_object_index_file;
    write_feature_file.open(pathname_write.c_str(), std::ios::out | std::ios::app);
    write_feature_object_index_file.open(pathname_object_index_write.c_str(), std::ios::out | std::ios::app);
    if (write_feature_file.is_open() && write_feature_object_index_file.is_open()) {
        // write_feature_file << kLabel << " ";
        for (int i = 0; i < feature_vector.size(); i++) {
            write_feature_file << feature_vector[i];
            if (i + 1 < feature_vector.size()) {
                write_feature_file << ",";
            }

        }
        write_feature_file << "\n";
        write_feature_file.close();
        write_feature_object_index_file << feature_object_index << "\n";
    } else {
        ROS_ERROR("Cannot open file");
    }

    return true;
}


void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
{
    float min, max, delta;

    min = std::min(std::min(r,g),b);
    max = std::max(std::max(r,g),b);

    *v = max;// v
    delta = max - min;
    if( max != 0 )
        *s = delta / max;// s
    else {
        // r = g = b = 0// s = 0, v is undefined
        *s = 0;
        *h = -1;
        return;
    }
    if( r == max )
        *h = ( g - b ) / delta;// between yellow & magenta
    else if( g == max )
        *h = 2 + ( b - r ) / delta;// between cyan & yellow
    else
        *h = 4 + ( r - g ) / delta;// between magenta & cyan
    *h *= 60;// degrees
    if( *h < 0 )
        *h += 360;
}
void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
{
    int i;
    float f, p, q, t;
    if( s == 0 ) {
        // achromatic (grey)
        *r = *g = *b = v;
        return;
    }
    h /= 60;// sector 0 to 5
    i = floor( h );
    f = h - i;// factorial part of h
    p = v * ( 1 - s );
    q = v * ( 1 - s * f );
    t = v * ( 1 - s * ( 1 - f ) );
    switch( i ) {
    case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
    case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
    case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
    case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
    case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
    default:// case 5:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
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
                    set_pcd_dir_name();
                    PointCloudT::Ptr temp_cloud(new PointCloudT);
                    std::string pcd_root_dir = ros::package::getPath("segbot_arm_perception") + "/pcd/" + pcd_dir_name;
                    // Grab paths for each pointcloud
                    std::vector<boost::filesystem::path> pcd_path_list = get_pcd_path_list(pcd_root_dir);
                    // Extract parent dir (object id) name
                    std::vector<std::string> feature_object_index_vector;
                    for (int i = 0; i < pcd_path_list.size(); i++) {
                        feature_object_index_vector.push_back(pcd_path_list[i].parent_path().filename().string().c_str());
                    }

                    for (int i = 0; i < pcd_path_list.size(); i++) {
                        ROS_INFO("%s", pcd_path_list[i].string().c_str());
                        if (!load_cloud_from_disk(temp_cloud, pcd_path_list[i].string())) {
                            ROS_WARN("Last file reached");
                            ros::shutdown();
                            exit(0);
                        }
                        ROS_INFO("Publishing cloud clusters size %d...", (int)temp_cloud->points.size());

                        temp_cloud->header.frame_id = cloud->header.frame_id;
                        toROSMsg(*temp_cloud, cloud_ros);
                        cloud_pub.publish(cloud_ros);

                        ROS_INFO("Press ENTER to continue");
                        std::cin.ignore();

                        pcl::PointCloud<pcl::PointXYZHSV>::Ptr temp_cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
                        // PointCloudT::Ptr temp_cloud_2(new PointCloudT)
                        pcl::PointCloudXYZRGBtoXYZHSV(*temp_cloud, *temp_cloud_hsv);
                        PointT temp_point;
                        pcl::PointXYZHSV temp_point_hsv;
                        for (int poi = 0; poi < temp_cloud->points.size(); poi++) {
                            pcl::PointXYZRGBtoXYZHSV(temp_cloud->points[poi], temp_point_hsv);
                            // ROS_INFO("before: (%f, %f, %f)", temp_cloud->points[poi].x, temp_cloud->points[poi].y, temp_cloud->points[poi].z);

                            // pcl::PointXYZRGBtoXYZHSV(temp_cloud->points[poi], temp_cloud_hsv->points[i]);
                            // ROS_INFO("before: (%f, %f, %f)", temp_cloud_hsv->points[poi].h, temp_cloud_hsv->points[poi].s, temp_cloud_hsv->points[poi].v);

                            // temp_point_hsv.s *= 0.8;
                            float r, g, b, h, s, v;
                            r = temp_cloud->points[poi].r;
                            g= temp_cloud->points[poi].g;
                            b = temp_cloud->points[poi].b;
                            RGBtoHSV( r, g, b, &h, &s, &v );
                            h *= 0.8;
                            HSVtoRGB( &r,  &g, &b,  h, s, v );
                            temp_cloud->points[poi].r = r;
                            temp_cloud->points[poi].g = g;
                            temp_cloud->points[poi].b = b;

                            // ros_INFO("after: (%f, %f, %f)", temp_cloud_hsv->points[poi].h, temp_cloud_hsv->points[poi].s, temp_cloud_hsv->points[poi].v);
                            // pcl::PointXYZHSVtoXYZRGB(temp_point_hsv,
                            //                          temp_point);
                            // temp_cloud->points[poi].r = temp_point.r;
                            // temp_cloud->points[poi].g = temp_point.g;
                            // temp_cloud->points[poi].b = temp_point.b;
                            // ROS_INFO("after: (%f, %f, %f)", temp_cloud->points[poi].x, temp_cloud->points[poi].y, temp_cloud->points[poi].z);
                        }

                        temp_cloud->header.frame_id = cloud->header.frame_id;
                        toROSMsg(*temp_cloud, cloud_ros);
                        cloud_pub.publish(cloud_ros);

                        ROS_INFO("Press ENTER to continue");
                        std::cin.ignore();
                        continue;

/////////

                        segbot_arm_perception::FeatureExtraction feature_srv;
                        toROSMsg(*temp_cloud, feature_srv.request.cloud);
                        ROS_INFO("Calling feature extraction service...");
                        // Find feature if there is more than 1 cluster on table
                        if (feature_srv_client.call(feature_srv)) {
                            ROS_INFO("Feature vector received");
                            std::vector<double> feature_vector = feature_srv.response.feature_vector;
                            // Write feature to disk
                            write_feature_to_disk_csv(feature_vector, feature_object_index_vector[i]);

                            // ROS_INFO("Publishing cloud clusters...");
                            // toROSMsg(*cloud_plane, cloud_ros);
                            // cloud_pub.publish(cloud_ros);
                        } else {
                            ROS_WARN("Error: No clusters found or failed to retrieve feature vector");
                        }

                        // ROS_INFO("Press ENTER to continue");
                        // std::cin.ignore();
                        // continue;
                    }
                    while (1);
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
                    write_feature_to_disk_libSVM(feature_vector);

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
