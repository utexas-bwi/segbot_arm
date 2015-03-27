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
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

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

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Select mode
const bool save_pl_mode = false;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
PointCloudT::Ptr empty_cloud (new PointCloudT);
std::vector<PointCloudT::Ptr > clusters;
std::vector<PointCloudT::Ptr > clusters_on_plane;

sensor_msgs::PointCloud2 cloud_ros;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// Check if a file exist or not
bool file_exist(std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{


	cloud_mutex.lock ();

	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;

	cloud_mutex.unlock ();
}

double computeAvgRedValue(PointCloudT::Ptr in){
	double total_red = 0;

	for (unsigned int i = 0; i < in->points.size(); i++){
		total_red += in->points.at(i).r;

	}

	total_red /= (in->points.size());
	return total_red;
}

int countRedVoxels(PointCloudT::Ptr in) {
    int total_red = 0;

    for (int i = 0; i < in->points.size(); i++) {
        unsigned int r, g, b;
        r = in->points[i].r;
        g = in->points[i].g;
        b = in->points[i].b;
        // Look for mostly red values points
        // These values are hand-picked using lab lighting samples
        if (r > 200 && (g + b) < 50) {
            total_red++;
        }
    }

    return total_red;
}


void computeClusters(PointCloudT::Ptr in, double tolerance){
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (tolerance); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	clusters.clear();

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (in->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
  }
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_button_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

	//debugging publisher
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("button_detection_node/cloud", 10);

	//button position publisher
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("button_detection_node/pose", 10);

	 tf::TransformListener listener;

	//register ctrl-c
	signal(SIGINT, sig_handler);

	//refresh rate
	double ros_rate = 3.0;
	ros::Rate r(ros_rate);

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();

		r.sleep();

		if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
		{
			new_cloud_available_flag = false;

			//Step 1: z-filter and voxel filter

			// Create the filtering object
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (cloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.0, 1.15);
			pass.filter (*cloud);

			ROS_INFO("Before voxel grid filter: %i points",(int)cloud->points.size());

			 // Create the filtering object: downsample the dataset using a leaf size of 1cm
			pcl::VoxelGrid<PointT> vg;
			pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
			vg.setInputCloud (cloud);
			vg.setLeafSize (0.005f, 0.005f, 0.005f);
			vg.filter (*cloud_filtered);

			ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());

			//pcl::toROSMsg(*cloud_filtered,cloud_ros);
			//cloud_ros.header.frame_id = cloud->header.frame_id;
			//cloud_pub.publish(cloud_ros);


			//Step 2: plane fitting

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
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);

			// Extract the plane
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);
			extract.filter (*cloud_plane);

			//extract everything else
			extract.setNegative (true);
			extract.filter (*cloud_blobs);

			//get the plane coefficients
			Eigen::Vector4f plane_coefficients;
			plane_coefficients(0)=coefficients->values[0];
			plane_coefficients(1)=coefficients->values[1];
			plane_coefficients(2)=coefficients->values[2];
			plane_coefficients(3)=coefficients->values[3];


			//Step 3: Eucledian Cluster Extraction
			computeClusters(cloud_blobs,0.04);

			clusters_on_plane.clear();

			for (unsigned int i = 0; i < clusters.size(); i++){
				Eigen::Vector4f centroid_i;
				pcl::compute3DCentroid(*clusters.at(i), centroid_i);
				pcl::PointXYZ center;
				center.x=centroid_i(0);center.y=centroid_i(1);center.z=centroid_i(2);

				double distance = pcl::pointToPlaneDistance(center, plane_coefficients);
				if (distance < 0.1 /*&& clusters.at(i).get()->points.size() >*/ ){
					clusters_on_plane.push_back(clusters.at(i));

				}
			}


			//Step 4: detect the button among the remaining clusters
			int max_index = -1;
/*
			double max_red = 0.0;
            // Find the max red value
			for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
				double red_i = computeAvgRedValue(clusters_on_plane.at(i));
				//ROS_INFO("Cluster %i: %i points, red_value = %f",i,(int)clusters_on_plane.at(i)->points.size(),red_i);

				if (red_i > max_red){
					max_red = red_i;
					max_index = i;
				}
			}
*/

            int max_num_red = 0;
            // Alternatively, find the blob with most red voxels
			for (unsigned int i = 0; i < clusters_on_plane.size(); i++) {
                int red_num_i = countRedVoxels(clusters_on_plane.at(i));
                if (red_num_i > max_num_red) {
                    max_num_red = red_num_i;
                    max_index = i;
                }
            }


			PointT min;
			PointT max;
			pcl::getMinMax3D(*clusters_on_plane.at(max_index),min,max);

			double volume = (max.x-min.x)*(max.y-min.y)*(max.z-min.z);

			//float area = pcl::calculatePolygonArea(*clusters_on_plane.at(max_index));
//			ROS_INFO("Button found: %i points with red_value = %f, volume = %f",(int)clusters_on_plane.at(max_index)->points.size(),max_red,volume);


			//publish  cloud if we think it's a button
			/*max_red > 170 && max_red < 250 && */
            if ((clusters_on_plane.at(max_index)->points.size()) < 360 &&
                (clusters_on_plane.at(max_index)->points.size()) > 80 &&
                (max_index >= 0)) {

				pcl::toROSMsg(*clusters_on_plane.at(max_index),cloud_ros);
				cloud_ros.header.frame_id = cloud->header.frame_id;
				cloud_pub.publish(cloud_ros);

				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*clusters_on_plane.at(max_index), centroid);

				//transforms the pose into /map frame
				geometry_msgs::Pose pose_i;
				pose_i.position.x=centroid(0);
				pose_i.position.y=centroid(1);
				pose_i.position.z=centroid(2);
				pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);

				geometry_msgs::PoseStamped stampedPose;

				stampedPose.header.frame_id = cloud->header.frame_id;
				stampedPose.header.stamp = ros::Time(0);
				stampedPose.pose = pose_i;

				geometry_msgs::PoseStamped stampOut;
				listener.waitForTransform(cloud->header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
				listener.transformPose("mico_api_origin", stampedPose, stampOut);

				stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
				pose_pub.publish(stampOut);

                // Save button pointcloud if mode is on
                if (save_pl_mode) {
                    static int button_cloud_count = 0;
                    char input_char;
                    ROS_INFO("Save current button point cloud? [y/n]");
                    std::cin >> input_char;
                    if(input_char == 'y') {
                        pcl::PCDWriter writer;
                        std::stringstream ss;
                        ss << "red_button_" << button_cloud_count <<  ".pcd";
                        std::string pathNameWrite = ros::package::getPath("segbot_arm_perception") + "/pcd/" + ss.str();
                        // Changing file name until it doesn't overlap with existing point clouds
                        while (file_exist(pathNameWrite)) {
                            button_cloud_count++;
                            ss.str("");
                            ss << "red_button_" << button_cloud_count <<  ".pcd";
                            pathNameWrite = ros::package::getPath("segbot_arm_perception") + "/pcd/" + ss.str();
                        }

                        ROS_INFO("Saving %s...", ss.str().c_str());
                        writer.write<PointT>(pathNameWrite, *clusters_on_plane.at(max_index), false);
                        button_cloud_count++;
                    }
                }
			}
			else {
				pcl::toROSMsg(*empty_cloud,cloud_ros);
				cloud_ros.header.frame_id = cloud->header.frame_id;
				cloud_pub.publish(cloud_ros);
			}


			//unlock mutex
			cloud_mutex.unlock ();
		}

	}
};
