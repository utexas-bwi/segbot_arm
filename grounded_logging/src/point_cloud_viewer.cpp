#include "ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{	
	pcl::PCLPointCloud2::Ptr image_cloud (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Fill in the cloud data
    pcl::PCDReader reader;
    
    // Change the name of the file to the one you want to view
    std::string pathName = "/home/bwi/priyanka_image_data/test/test223_1436561502.3271854.pcd";
    reader.read (pathName, *image_cloud);
    
    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*image_cloud, *cloud);
    
    //Start the cloud viewer
    std::cerr << "Start Cloud Viewer..." << std::endl;
    pcl::visualization::CloudViewer viewer("Filtered cloud image");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ());
    return 0;
    
    return(0);
    
}
