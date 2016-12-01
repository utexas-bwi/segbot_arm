#!/bin/bash
echo Recording point cloud. Press Ctrl-C when recording is completed
cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/point_cloud/
rosbag record /xtion_camera/depth_registered/points
read -rsp $'Press any key to continue...\n' -n1 key
for i in $(ls *.bag); do rosrun pcl_ros bag_to_pcd $i /xtion_camera/depth_registered/points /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/point_cloud/point_cloud_files; done
