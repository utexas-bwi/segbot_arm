#include <iostream>

#include "ros/ros.h"

#include "segbot_arm_perception/TableDetection.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// TableDetection.srv has req.cloud, res.cloud_plane, res.cloud_blobs
bool remove_plane_cb(segbot_arm_perception::TableDetection::Request &req,
                    segbot_arm_perception::TableDetection::Response &res) {
    PointCloudT::Ptr cloud(new PointCloudT), cloud_blobs(new PointCloudT),
        cloud_plane(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);

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
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    // Extract the plane
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);

    //extract everything else
    extract.setNegative (true);
    extract.filter (*cloud_blobs);

    //get the plane coefficients
    // TODO put this in .srv as well
    Eigen::Vector4f plane_coefficients;
    plane_coefficients(0)=coefficients->values[0];
    plane_coefficients(1)=coefficients->values[1];
    plane_coefficients(2)=coefficients->values[2];
    plane_coefficients(3)=coefficients->values[3];

    pcl::toROSMsg(*cloud_plane, res.cloud_plane);
    pcl::toROSMsg(*cloud_blobs, res.cloud_blobs);

    return true;
}


int main (int argc, char** argv) {
    ros::init(argc, argv, "table_segmentation_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/table_segmentation_node/remove_plane", remove_plane_cb);

    ROS_INFO("Table segmentation ready");
    ros::spin();
    return 0;
}
