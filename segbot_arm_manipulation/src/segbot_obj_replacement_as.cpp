#include <ros/ros.h>
#include <ros/package.h>

#include <cstdlib>

#include <geometry_msgs/TwistStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl_ros/impl/transforms.hpp>

//the action definition
#include "segbot_arm_manipulation/ObjReplacementAction.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ObjReplacementActionServer
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<segbot_arm_manipulation::ObjReplacementAction> as_; 
	std::string action_name_; 

	segbot_arm_manipulation::ObjReplacementFeedback feedback_; 
	segbot_arm_manipulation::ObjReplacementResult result_; 

	//holds set of predefined positions
	ArmPositionDB *posDB;

public:
	ObjReplacementActionServer(std::string name) :
		as_(nh_, name, boost::bind(&ObjReplacementActionServer::executeCB, this, _1), false),
    		action_name_(name)
    {
    	//load database of joint- and tool-space positions
		std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
		std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";

    	posDB = new ArmPositionDB(j_pos_filename, c_pos_filename);

    	ROS_INFO("Starting replacement grasp action server..."); 
    	as_.start(); 
    }

	~ObjReplacementActionServer(void)
	{
	}

		void executeCB(const segbot_arm_manipulation::ObjReplacementGoalConstPtr  &goal)
		{
			posDB->print(); 
			//home the arm
			segbot_arm_manipulation::homeArm(nh_);
			segbot_arm_manipulation::closeHand();

			// Step 1: Get center point of table (assumes empty table)

			// Move arm out of view of camera 
			if (posDB->hasCarteseanPosition("side_view")){
				ROS_INFO("Moving out of the way...");
				geometry_msgs::PoseStamped out_of_view_pose = posDB->getToolPositionStamped("side_view","/mico_link_base");
				
				//now go to the pose
				segbot_arm_manipulation::moveToPoseMoveIt(nh_,out_of_view_pose);
			}
			else {
				ROS_ERROR("[segbot_table_approach_as.cpp] Cannot move arm out of view!");
			}

			// Get table scene 
			ros::ServiceClient client_tabletop_perception = nh_.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
			segbot_arm_perception::TabletopPerception srv;

			srv.request.override_filter_z = false;

			if (client_tabletop_perception.call(srv))
			{
				ROS_INFO("Received Response from tabletop_object_detection_service");
			}
			else
			{
				ROS_ERROR("Failed to call perception service");
				result_.success = false;
				result_.error_msg = "cannot_call_tabletop_perception";
				as_.setSucceeded(result_);
				return;
			}

			// Check for table 
			if (srv.response.is_plane_found == false){
				ROS_ERROR("[segbot_obj_replacement_as.cpp] Table not found. The end.");
				result_.success = false;
				result_.error_msg = "table_not_found";
				as_.setAborted(result_);
				return;
			}

			sensor_msgs::PointCloud2 table_cloud_ros = srv.response.cloud_plane;
			PointCloudT::Ptr table_cloud (new PointCloudT); 
			Eigen::Vector4f table_centroid;

			//convert to PCL format and take centroid
			pcl::fromROSMsg (table_cloud_ros, *table_cloud);
			pcl::compute3DCentroid (*table_cloud, table_centroid);
		
ROS_INFO("[segbot_obj_replacement_as.cpp] Table xyz: %f, %f, %f",table_centroid(0),table_centroid(1),table_centroid(2));
		}
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_obj_replacement_as");

  ObjReplacementActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}