#include <signal.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const double length_off_table = 0.15;

ros::Publisher cloud_clusters_pub;
ros::Publisher cloud_plane_pub;

typedef struct ColorRGB {
    Eigen::Vector4f centroid;
    bool taken;
    int r;
    int g;
    int b;
} ColorRGB;

std::vector<ColorRGB> color_vector;

//true if Ctrl-C is pressed
bool g_caught_sigint = false;

/* what happens when ctr-c is pressed */
void sigint_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("Caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
}

bool pointCloudSizeComp (PointCloudT* cloud_1, PointCloudT* cloud_2) {
    return cloud_1->points.size() > cloud_2->points.size();
}

bool colorPointCloud(PointCloudT &in, int index) {
    ColorRGB* color;
    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(in, centroid);
    // Check if we already have this color
    if (index < color_vector.size()) {
        int color_index = -1;
        double min_distance = std::numeric_limits<int>::max();
        for (int i = 0; i < color_vector.size(); i++) {
            if (!color_vector[i].taken) {
                double distance_i = sqrt(pow(centroid(0) - color_vector[i].centroid(0), 2) +
                                         pow(centroid(1) -color_vector[i].centroid(1), 2) +
                                         pow(centroid(2) - color_vector[i].centroid(2), 2));
                if (distance_i < min_distance) {
                    min_distance = distance_i;
                    color_index = i;
                }
            }
        }
        if (color_index < 0) {
            ROS_INFO("ERROR, all colors taken, something went wrong~!");
            return false;
        }
        ROS_INFO("color_index = %d", color_index);
        color = &color_vector[color_index];
        for (int i = 0; i < in.points.size(); i++) {
            in.points[i].r = color->r;
            in.points[i].g = color->g;
            in.points[i].b = color->b;
        }
        color->taken = true;
        color->centroid = centroid;

    } else {
        color = new ColorRGB;
        color->r = rand() % 255;
        color->g = rand() % 255;
        color->b = rand() % 255;
        for (int i = 0; i < in.points.size(); i++) {
            in.points[i].r = color->r;
            in.points[i].g = color->g;
            in.points[i].b = color->b;
        }
        color->centroid = centroid;
        color_vector.push_back(*color);
    }

    return true;
}

bool computeClusters(PointCloudT::Ptr in, std::vector<PointCloudT> &cloud_clusters,
                     double tolerance) {
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

    // Separate pointcloud into blobs (cluster_indices)
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (tolerance);
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
         it != cluster_indices.end (); ++it) {
		PointCloudT cloud_cluster;
        // Get all points
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
            cloud_cluster.points.push_back(in->points[*pit]);
        }
		cloud_cluster.width = cloud_cluster.points.size ();
		cloud_cluster.height = 1;
		cloud_cluster.is_dense = true;
        // Add cluster to cluster vector
		cloud_clusters.push_back(cloud_cluster);
    }

    return true;
}


bool table_detection_object_extraction_cb(
    segbot_arm_perception::TableDetectionObjectExtraction::Request &req,
    segbot_arm_perception::TableDetectionObjectExtraction::Response &res) {

    PointCloudT::Ptr cloud(new PointCloudT), cloud_filtered(new PointCloudT),
        cloud_blob(new PointCloudT), cloud_plane(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);

    // Step 1: z and voxel filters
    // Create the filtering object
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.15);
    pass.filter (*cloud);

    ROS_INFO("Before voxel grid filter: %i points",(int)cloud->points.size());

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*cloud_filtered);

    ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());

    // Step 2: plane separating
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
    seg.setDistanceThreshold (0.01); // 1 cm plane

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
    extract.filter (*cloud_blob);

    //get the plane coefficients
    Eigen::Vector4f plane_coefficients;
    for (int i = 0; i < coefficients->values.size(); i++) {
        res.cloud_plane_coef[i] = coefficients->values[i];
        plane_coefficients(i) = coefficients->values[i];
    }

    // TODO give a good condition for plane detection or not
    res.is_plane_found = true;

    //Step 3: Eucledian Cluster Extraction
    std::vector<PointCloudT> cloud_clusters;
    std::vector<int> remove_indices;

    // Extract clusters
    computeClusters(cloud_blob, cloud_clusters, 0.04);

    ROS_INFO("cloud_clusters.size() = %d", (int)cloud_clusters.size());
    ROS_INFO("%f x + %f y + %f z + %f = 0", plane_coefficients(0), plane_coefficients(1),
             plane_coefficients(2), plane_coefficients(3));
    std::vector<PointCloudT*> cloud_clusters_on_plane;
    // Only use the clusters with centroid close to the plane
    for (unsigned int i = 0; i < cloud_clusters.size(); i++) {
        Eigen::Vector4f centroid_i;
        pcl::compute3DCentroid(cloud_clusters[i], centroid_i);
        PointT center;
        center.x = centroid_i(0);
        center.y = centroid_i(1);
        center.z = centroid_i(2);

        double distance = pcl::pointToPlaneDistance(center, plane_coefficients);
        double is_above_table = center.x * plane_coefficients(0) +
                                center.y * plane_coefficients(1) +
                                center.z + plane_coefficients(2) + plane_coefficients(3);
        if ((distance < length_off_table) && (is_above_table > 0)) {
            ///////////////////
            // Make pointcloud of centroids
            // cloud_clusters[i].width = 1;
            // cloud_clusters[i].height = 1;
            // cloud_clusters[i].points.resize(1);
            // cloud_clusters[i].points[0] = center;
            /////////////////
            cloud_clusters_on_plane.push_back(&cloud_clusters[i]);
            ROS_INFO("distance = %f", distance);
            ROS_INFO("(%f, %f, %f)", center.x, center.y, center.z);
        }
    }

    ROS_INFO("Number of clusters = %d", (int)cloud_clusters_on_plane.size());

    // Reset color taken vales
    for (int i = 0; i < color_vector.size(); i++) {
        color_vector[i].taken = false;
    }

    // Sort cloud_clouter_on_plane ascending
    std::sort(cloud_clusters_on_plane.begin(), cloud_clusters_on_plane.end(), pointCloudSizeComp);
    for (int i = 0; i < cloud_clusters_on_plane.size(); i++) {
        ROS_INFO("cloudClusterSize = %d", (int)cloud_clusters_on_plane[i]->points.size());
    }

    // Copy ponitcloud header
    pcl::toROSMsg(*cloud_plane, res.cloud_plane);
    res.cloud_plane.header.frame_id = req.cloud.header.frame_id;
    res.cloud_plane.header.stamp = ros::Time::now();
    PointCloudT cloud_debug;
    for (unsigned int i = 0; i < cloud_clusters_on_plane.size(); i++) {
        sensor_msgs::PointCloud2 cloud_cluster_ros;
        pcl::toROSMsg(*(cloud_clusters_on_plane[i]), cloud_cluster_ros);
        cloud_cluster_ros.header.frame_id = req.cloud.header.frame_id;
        cloud_cluster_ros.header.stamp = ros::Time::now();
        res.cloud_clusters.push_back(cloud_cluster_ros);
        // For debug purpose
        PointCloudT cloud_temp(*cloud_clusters_on_plane[i]);
        colorPointCloud(cloud_temp, i);
        cloud_debug += cloud_temp;
    }

    // Publish cloud result
    ROS_INFO("Publishing pointcloud clusters...");
    sensor_msgs::PointCloud2 cloud_debug_ros;
    toROSMsg(cloud_debug, cloud_debug_ros);
    cloud_debug_ros.header.frame_id = res.cloud_plane.header.frame_id;
    cloud_clusters_pub.publish(cloud_debug_ros);
    cloud_plane_pub.publish(res.cloud_plane);

    return true;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "cluster_extraction_server");
    ros::NodeHandle nh;

    signal(SIGINT, sigint_handler);

    // Debugging publisher
    cloud_clusters_pub = nh.advertise<sensor_msgs::PointCloud2>("/table_detection_object_extraction/cloud", 10);
    cloud_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("/table_detection_object_extraction/cloud_plane", 10);

    ros::ServiceServer service = nh.advertiseService("/segbot_arm_perception/table_detection_object_extraction_server", table_detection_object_extraction_cb);

    ROS_INFO("Table detection object extraction server ready");
    ros::spin();
    return 0;
}
