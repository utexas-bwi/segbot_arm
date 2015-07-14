#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

using namespace std;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const double length_off_table = 0.15;

// General point cloud to store the whole image
PointCloudT::Ptr image_cloud (new PointCloudT), cloud_blob(new PointCloudT), cloud_plane(new PointCloudT);

// function to compute the clusters
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

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "extract_object_color");
	ros::NodeHandle nh;
	
	ROS_INFO("Ready to extract colour from .pcd files...");
	
	// Fill in the cloud data
    pcl::PCDReader reader;
   
	// The general file path
    std::string generalFilePath = "/home/bwi/priyanka_image_data/pointClouds/test0_1436751414.0383992.pcd"
	//TODO add file traversal so that all .pcd files in the above path are accessed consecutively
	std::string filePath = generalFilePath + "";
	reader.read(filePath, *image_cloud);
	
	//Plane separating
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
    
    //Eucledian Cluster Extraction
    std::vector<PointCloudT> cloud_clusters;
    std::vector<int> remove_indices;
    
    // Extract clusters
    computeClusters(cloud_blob, cloud_clusters, 0.04);
    
    // Only use the clusters with centroid close to the plane as they will belong to the object
    std::vector<PointCloudT*> cloud_clusters_on_plane;
    
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
			 cloud_clusters_on_plane.push_back(&cloud_clusters[i]);
        }
    }
	
	ROS_INFO("Number of clusters = %d", (int)cloud_clusters_on_plane.size());
	    
	return 0;
}


