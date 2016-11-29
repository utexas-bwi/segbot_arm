#!/bin/bash
echo Recording tool_position. Press Ctrl-C when recording is completed
cd tool_position
rosbag record mico_arm_driver/out/tool_position
read -rsp $'Press any key to continue...\n' -n1 key
for i in $(ls *.bag); do rostopic echo -b $i -p mico_arm_driver/out/tool_position > $i.csv; done
