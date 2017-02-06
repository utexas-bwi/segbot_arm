#!/bin/bash
echo Recording data. Press Ctrl-C when recording is completed

(cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/finger_position && rosbag record mico_arm_driver/out/finger_position) &
(cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/joint_efforts && rosbag record mico_arm_driver/out/joint_efforts) &
(cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/joint_states && rosbag record joint_states) &
(cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/tool_position && rosbag record mico_arm_driver/out/tool_position) &
(cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/point_cloud && rosbag record /xtion_camera/depth_registered/points) & 
(cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/xtion_camera && rosrun image_view image_saver image:=/xtion_camera/rgb/image_raw)


read -rsp $'Press any key to continue...\n' -n1 key
echo Converting...
# for finger_position
cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/finger_position
for i in $(ls *.bag); do rostopic echo -b $i -p mico_arm_driver/out/finger_position > $i.csv; done

# joint_efforts
cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/joint_efforts
for i in $(ls *.bag); do rostopic echo -b $i -p mico_arm_driver/out/joint_efforts > $i.csv; done

# joint_states
cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/joint_states
for i in $(ls *.bag); do rostopic echo -b $i -p joint_states > $i.csv; done

# tool_position
cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/tool_position
for i in $(ls *.bag); do rostopic echo -b $i -p mico_arm_driver/out/tool_position > $i.csv; done

# point cloud
cd /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/point_cloud
for i in $(ls *.bag); do rosrun pcl_ros bag_to_pcd $i /xtion_camera/depth_registered/points /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/point_cloud; done

echo Converting completed.
