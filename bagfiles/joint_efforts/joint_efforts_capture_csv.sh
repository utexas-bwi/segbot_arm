#!/bin/bash
echo Recording joint_efforts. Press Ctrl-C when recording is completed
cd /home/users/fri/lafd_workspace/src/joystick_bwi/bagfiles/joint_efforts
rosbag record mico_arm_driver/out/joint_efforts
read -rsp $'Press any key to continue...\n' -n1 key
for i in $(ls *.bag); do rostopic echo -b $i -p mico_arm_driver/out/joint_efforts > $i.csv; done
