#!/bin/bash
echo Recording joy
rosbag record joy
read -rsp $'Press any key when data recording is completed...\n' -n1 key
rostopic echo -b *.bag -p /joy
