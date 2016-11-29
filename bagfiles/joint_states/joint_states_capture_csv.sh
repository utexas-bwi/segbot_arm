#!/bin/bash
echo Recording joint_states. Press Ctrl-C when recording is completed
cd joint_states
rosbag record joint_states
read -rsp $'Press any key to continue...\n' -n1 key
for i in $(ls *.bag); do rostopic echo -b $i -p joint_states > $i.csv; done
