#include <signal.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include "segbot_arm_perception/FeatureExtraction.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef unsigned int uint;

class ColorHistogram {
  private:
    std::vector<std::vector<std::vector<uint> > > hist3;
    int dim;
  public:
    ColorHistogram(int dim):dim(dim) {
        hist3.resize(dim);
        for (int i = 0; i < dim; i++) {
            hist3[i].resize(dim);
            for (int j = 0; j < dim; j++) {
                hist3[i][j].resize(dim);
                std::fill( hist3[i][j].begin(), hist3[i][j].end(), 0 );
            }
        }
    }

    void computeHistogram(PointCloudT &cloud) {
        for (int i = 0; i < cloud.points.size(); i++) {
            int r = (int)cloud.points[i].r / dim;
            int g = (int)cloud.points[i].g / dim;
            int b = (int)cloud.points[i].b / dim;
            hist3[r][g][b]++;
        }
    }

};


// Point Feature Histograms (PFH) descriptors
// pointclouds.org/documentation/tutorials/pfh_estimation.php
bool computePfh(PointCloudT::Ptr &cloud) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    ROS_INFO("Hi");
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ROS_INFO("Hi");
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
    ROS_INFO("Hi");
    // Check for undefined values
    for (int i = 0; i < cloud_normals->points.size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i]))
        {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
    }
    // Visualize
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->addPointCloud<pcl::Normal> (cloud_normals, "normal cloud");
    ROS_INFO("Hi");
    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod (tree2);
    ROS_INFO("Hi2");
    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.05);
    ROS_INFO("Hi");
    // Compute the features
    pfh.compute (*pfhs);
    ROS_INFO("Hi");
return true;
}

bool feature_extraction_cb(
    segbot_arm_perception::FeatureExtraction::Request &req,
    segbot_arm_perception::FeatureExtraction::Response &res) {
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);

    // Pack return feature vector
    res.feature_vector;
    return true;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "object_feature_detection_server");
    ros::NodeHandle nh;

ros::ServiceServer service = nh.advertiseService("/segbot_arm_perception/feature_extraction_server", feature_extraction_cb);

    ROS_INFO("Feature extraction server ready");
    ros::spin();
    return 0;
}
