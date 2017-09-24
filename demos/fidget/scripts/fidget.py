#!/usr/bin/env python
import roslib; roslib.load_manifest('characterize_power')
import rospy
from jaco_msgs.msg import JointCurrents
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from moveit_utils.srv import MicoMoveitCartesianPose
import time

#rosservice call /mico_cartesianpose_service '{target: { pose: {position: {x: 0.2,y: 0.0,z: 0.0},orientation:{x: 2.0}}}}'

cartesian_client = None
last_timestamp = None
trials = []
current_trial = 0

class Trial:
	def __init__(start_stamp):
		self.observations = []
		self.start_stamp = start_stamp
		self.end_stamp = end_stamp

def callback(data):
    new_timestamp = time.time()
    total = sum([abs(reading) for reading in data.currents])
    global current_trial
    global trials
    global last_timestamp
    if current_trial >= len(trials):
	trials.append(Trial(new_timestamp))
    if last_timestamp:
	diff = new_timestamp - last_timestamp 
	# We'll integrate the power estimate as we go. This measurement has a unit of watt/seconds
	trials[current_trial].observations.append(diff)
    last_timestamp = time.time()

def execute_trajectory(trajectory, client):
    for point in trajectory:
	pose = PoseStamped()
	pose.position.x = point[0]
	pose.position.y = point[1]
	pose.position.z = point[2]
	client(pose)

	

def main():
    global cartesian_client
    global s
    rospy.init_node('fidget', anonymous=True)
   
    cartesian_client = rospy.ServiceProxy("mico_cartesianpose_service", MicoMoveitCartesianPose)
    global current_trial
    while not rospy.is_shutdown():
	execute_trajectory([(0,0,0),(1,1,1)], cartesian_client)
        current_trial += 1
		
	

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
