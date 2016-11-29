#!/bin/bash
echo Recording joy. Press Ctrl-C when recording is completed
rosbag record joy
read -rsp $'Press any key to continue...\n' -n1 key
for i in $(ls *.bag); do rostopic echo -b $i -p joy > $i.csv; done
