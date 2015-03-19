/*
This ROS service analyzes pointcloud data on the camera/depth topic
Current implementation:
requests: boolean exludePlane -: true if the returned clouds should be all points exclude the planes (useful for object recognition on a plane, for instance).
                                      if false, only clouds containing planes will be returned
TODO: 
requests: plane orientation (vertical, horizontal)
          plane size
          number of planes returned


Filter out clouds that are sparsely populated and very spread out. Likely not a plane.
*/


#include <ros/ros.h>
#include <vector>
#include "segbot_arm_perception/PlanarSegmentation.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

void depth_cb(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::fromROSMsg (*input, *cur_cloud);
}
bool seg_cb (segbot_arm_perception::PlanarSegmentation::Request &req, segbot_arm_perception::PlanarSegmentation::Response &res)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  std::vector<sensor_msgs::PointCloud2> cloud_cont;
 // Do data processing here...
  //output = *input;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  ROS_INFO("PointCloud before filtering has: %lu data points.",cur_cloud->points.size());

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cur_cloud);
  vg.setFilterFieldName("z");
  vg.setFilterLimits(0.01, 1.5);
  vg.setLeafSize (0.015f, 0.015f, 0.015f);
  vg.filter (*cloud_filtered);
  ROS_INFO("PointCloud after filtering has: %lu data points.",cloud_filtered->points.size());

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_plane;// (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);


  //This codeblock is useful if breaking up the pointcloud based on size or something iterative
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () != 0)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_INFO("Could not estimate a planar model for the given dataset.");
      break;
    }
    else if(inliers->indices.size() <= 5){ //added due to high number of low count planes
      ROS_INFO("Plane size below threshold. Skipping.");
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    //Convert to ros sensor msg, add to vector
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_plane, output );
    cloud_cont.push_back(output);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
    res.clouds = cloud_cont;
}

  int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "planar_segmentation_service");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe ("/camera/depth_registered/points", 3, depth_cb);
  ros::ServiceServer service = n.advertiseService("PlanarSegmentation", seg_cb);
  ros::spin ();
}
