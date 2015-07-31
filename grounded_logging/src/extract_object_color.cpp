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
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
// For traversing the filesystem
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp> 

using namespace std;
using namespace boost::filesystem;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const double length_off_table = 0.10;

//Name of the vision folder to search for
const std::string & vision_folder = "vision_data";

//Name of the look behaviour folder to search for
const std::string & look_folder = "look";

//Total number of objects and trials for ease of file traversal
int total_objects = 32, total_trials = 6;

// General point cloud to store the whole image
PointCloudT::Ptr image_cloud (new PointCloudT), cloud_blob(new PointCloudT), cloud_plane(new PointCloudT), convexHull(new PointCloudT);

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
    std::string generalFilePath = "/home/users/pkhante/grounded_learning_experiments/";
	//File traversal so that all .pcd files in the above path are accessed consecutively
	for(int object_num = 1; object_num <= total_objects; object_num++){
		for(int trial_num = 1; trial_num <= total_trials; trial_num++){
			std::stringstream convert1, convert2;
			convert1 << object_num;
			convert2 << trial_num;
			string filePath = generalFilePath + "obj_" + convert1.str() + "/trial_" + convert2.str();
			path dir_path(filePath);
			if(!exists(dir_path))
				return 1;
			
			directory_iterator end_itr;       // default construction yields past-the-end
			for(directory_iterator itr(dir_path); itr != end_itr; ++itr){
					if(itr->path().filename().string() == look_folder){    // if the folder is the look behaviour then get the pcd file
					directory_iterator end_itr2;       // default construction yields past-the-end
					for(directory_iterator itr2(itr->path()); itr2 != end_itr2; ++itr2){
						if(itr2->path().filename().string() == vision_folder){
							//Go through all the files and find the .pcd files
							directory_iterator itr3(itr2->path()), eod;
							BOOST_FOREACH(path const &p, std::make_pair(itr3, eod)){ 
								if(is_regular_file(p)){
									if(p.extension() == ".pcd"){
										//cout << p.stem();
										string filePath = itr2->path().string() + "/" + p.filename().string();
										ROS_INFO("Filepath: %s", filePath.c_str());
 										reader.read(filePath, *image_cloud);
 										//ROS_INFO("Size: %ld", image_cloud->points.size());
										
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
										seg.setDistanceThreshold (0.022); // 2.2 cm plane

										// Create the filtering object
										pcl::ExtractIndices<PointT> extract;

										// Segment the largest planar component from the remaining cloud
										seg.setInputCloud (image_cloud);
										seg.segment (*inliers, *coefficients);

										// Extract the plane
										extract.setInputCloud (image_cloud);
										extract.setIndices (inliers);
										extract.setNegative (false);
										extract.filter (*cloud_plane);
										//ROS_INFO("Cloud_plane Size: %ld", cloud_plane->points.size());
										
										// Retrieve the convex hull.
										pcl::ConvexHull<PointT> hull;
										hull.setInputCloud(cloud_plane);
										// Make sure that the resulting hull is bidimensional.
										hull.setDimension(2);
										hull.reconstruct(*convexHull);
										
										if (hull.getDimension() == 2)
										{
											// Prism object.
											pcl::ExtractPolygonalPrismData<PointT> prism;
											prism.setInputCloud(image_cloud);
											prism.setInputPlanarHull(convexHull);
											// First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
											// Second parameter: maximum Z value. Tune it according to the height of the objects you expect.
											prism.setHeightLimits(0.02f, 0.25f);
											pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
								 
											prism.segment(*objectIndices);
								 
											// Get and show all points retrieved by the hull.
											extract.setIndices(objectIndices);
											extract.filter(*cloud_blob);
											//ROS_INFO("Cloud_blob Size: %ld", cloud_blob->points.size());
											
											//use visualizer only for debugging purposes. Code seg faults after some time.
											pcl::visualization::CloudViewer viewerObjects("Objects on table");
											viewerObjects.showCloud(cloud_blob);
											while (!viewerObjects.wasStopped())
											{
												// Do nothing but wait.
											}
										}
										else std::cout << "The chosen hull is not planar." << std::endl;
										
										image_cloud->clear();
										cloud_plane->clear();
										cloud_blob->clear();
										convexHull->clear();
										
										//extract everything else
										/*extract.setNegative (true);
										extract.filter (*cloud_blob);
										ROS_INFO("Cloud_blob Size: %ld", cloud_blob->points.size());

										//get the plane coefficients
										Eigen::Vector4f plane_coefficients;
										plane_coefficients(0)=coefficients->values[0];
										plane_coefficients(1)=coefficients->values[1];
										plane_coefficients(2)=coefficients->values[2];
										plane_coefficients(3)=coefficients->values[3];
										
										//Eucledian Cluster Extraction
										std::vector<PointCloudT> cloud_clusters;
										std::vector<int> remove_indices;
										
										// Extract clusters
										computeClusters(cloud_blob, cloud_clusters, 0.04);
										ROS_INFO("Size of cloud_clusters: %ld", cloud_clusters.size());
										
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
											ROS_INFO("is_above_table: %f", is_above_table);
											
											if ((distance < length_off_table) && (is_above_table > 0)) {
												 cloud_clusters_on_plane.push_back(&cloud_clusters[i]);
											}
										}
										
										ROS_INFO("Size of cloud_clusters_on_plane: %ld", cloud_clusters_on_plane.size());
										
										ROS_INFO("Number of clusters = %d", (int)cloud_clusters_on_plane.size());
										
										// Start visualizing the current plane
										std::cerr << "Start Cloud Viewer..." << std::endl;
										pcl::visualization::CloudViewer viewer ("plane segmentation");
										viewer.showCloud (cloud_blob);
										while (!viewer.wasStopped ());
										
										image_cloud->clear();
										cloud_plane->clear();
										cloud_blob->clear();
										convexHull->clear();*/
										
										/*pcl::PCDWriter writer;
										std::stringstream ss;
										ss << "object_extraction.pcd";
										ROS_INFO("Saving the .csv file");
										writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_blob, false);
										
										//TODO Write it to a .csv file
										string csvFilePath = itr2->path.string() + "/pointCloudData.csv";
										std::ofstream outputCsvFile(csvFilePath.c_str());
										if(!csvFilePath.is_open()){
											std::cout<< "Could not open the file to store\n";
											return -1;
										}*/
									}
								} 
							}
						}
					}
				}
			}
		}
	}
	return 0;
}
