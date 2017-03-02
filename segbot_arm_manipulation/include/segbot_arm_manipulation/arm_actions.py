"""Function calls for various arm actions"""
import roslib
roslib.load_manifest('segbot_arm_manipulation')
import rospy

import actionlib
import actionlib_msgs.msg

from geometry_msgs.msg import *
import std_msgs.msg
from jaco_msgs.msg import *
import sensor_msgs.msg
from segbot_arm_manipulation.msg import *
from segbot_arm_perception.msg import *

def get_table_scene():
	

def get_largest_object(table_scene):
	for num in table_scene.cloud_clusters.size():
		goal.cloud_clusters.push_back(table_scene.cloud_clusters(num))
		

def arm_grasp(table_scene):
	"""Send goal to grasp the largest object on the table"""
	action_address = 'segbot_tabletop_grasp_as'
	client = actionlib.SimpleActionClient(action_address, segbot_arm_manipulation.msg.TabletopGraspAction)
	client.wait_for_server()
	
	goal = TabletopGraspGoal()
	goal.action_name = segbot_arm_manipulation.msg.TabletopGraspGoal.GRASP
	goal.grasp_selection_method = segbot_arm_manipulation.msg.TabletopGraspGoal.CLOSEST_JOINTSPACE_SELECTION
	goal.cloud_plane = table_scene.cloud_plane
	goal.cloud_plane_coef = table_scene.cloud_plane_coef
	
	
	goal.target_object_cluster_index = 
	
