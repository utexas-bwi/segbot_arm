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
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>b
#include <pcl/visualization/pcl_plotter.h>

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
            // Max value of 255
            double round = 256 / dim;
            int r = (int)((double)(cloud.points[i].r) / round);
            int g = (int)((double)(cloud.points[i].g) / round);
            int b = (int)((double)(cloud.points[i].b) / round);
            hist3[r][g][b]++;
        }
    }
    uint get(int r, int g, int b) {
        return hist3[r][g][b];
    }
    // Breaks program, don't use
    std::string rosPrintHist() {
        for (int i = 0; i < dim; i++) {
            ROS_INFO("i = %d", i);
            ROS_INFO("[");
            for (int j = 0; j < dim; j++) {
                std::string output;
                for (int k = 0; k < dim; k++) {
                    output += boost::lexical_cast<std::string>(hist3[i][j][k]) + ", ";
                }
                ROS_INFO("%s", output.c_str());
            }
            ROS_INFO("]");
        }
    }
    std::vector<double> toDoubleVector() {
        int i_offset = dim * dim;
        int j_offset = dim;
        std::vector<double> hist3_double_vector (dim * dim * dim, 0);
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                for (int k = 0; k < dim; k++) {
                    hist3_double_vector[i * i_offset + j * j_offset + k] = hist3[i][j][k];
                }
            }
        }

        return hist3_double_vector;
    }
};


pcl::PointCloud<pcl::Normal>::Ptr computeNormals(PointCloudT::Ptr &cloud) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
    // Check for undefined values
    for (int i = 0; i < cloud_normals->points.size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i]))
        {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
    }

    return cloud_normals;
}



pcl::PointCloud<pcl::VFHSignature308>::Ptr computeCVFH(PointCloudT::Ptr &cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud);

    // Create the CVFH estimation class, and pass the input dataset+normals to it
    pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud (cloud);
    cvfh.setInputNormals (cloud_normals);

// Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    cvfh.setSearchMethod (tree);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    cvfh.compute(*vfhs);

    return vfhs;
}


// http://pointclouds.org/documentation/tutorials/vfh_estimation.php#vfh-estimation
pcl::PointCloud<pcl::VFHSignature308>::Ptr computeVFH(PointCloudT::Ptr &cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud);

    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);

    // vfhs->points.size () should be of size 1*
    return vfhs;
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(PointCloudT::Ptr &cloud) {
    // Find normal
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud);

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());


    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);

    // Compute the features
    fpfh.compute (*fpfhs);

    // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
    return fpfhs;
}


// Point Feature Histograms (PFH) descriptors
// pointclouds.org/documentation/tutorials/pfh_estimation.php
pcl::PointCloud<pcl::PFHSignature125>::Ptr computePFH(PointCloudT::Ptr &cloud) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
    // Check for undefined values
    for (int i = 0; i < cloud_normals->points.size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i]))
        {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
    }

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
    pfh.setSearchMethod (tree2);
    ROS_INFO("Hi2");
    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.05);
    // Compute the features
    pfh.compute (*pfhs);
}


bool feature_extraction_cb(
    segbot_arm_perception::FeatureExtraction::Request &req,
    segbot_arm_perception::FeatureExtraction::Response &res) {
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);
    const int kColorHistBins = 8;

    // Features:
    // Color
    ColorHistogram ch(kColorHistBins);
    ch.computeHistogram(*cloud);

    // CVFH
    computeFPFH(cloud);

    //defining a plotter
    // pcl::visualization::PCLPlotter * plotter = new PCLPlotter ();

    // //defining the polynomial function, y = x^2. Index of x^2 is 1, rest is 0
    // vector<double> func1 (3,0);
    // func1[2] = 1;

    // //adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
    // plotter->addPlotData (func1, -10, 10, "y = x^2");

    // //display the plot, DONE!
    // plotter->plot ();

    // vfhs->points[i].histogram[j]
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
