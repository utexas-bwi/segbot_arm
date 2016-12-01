echo Recording Data. Press Ctrl-C when recording is completed

(cd /home/krishna/catkin_ws/src/joystick_bwi/bagfiles/joy && rosbag record joy) &
(cd /home/krishna/catkin_ws/src/joystick_bwi/bagfiles/joint_states && rosbag record joint_states) 

echo Hello
read -rsp $'Press any key to continue...\n' -n1 key
cd /home/krishna/catkin_ws/src/joystick_bwi/bagfiles/joy 
for i in $(ls *.bag); do rostopic echo -b $i -p joy > $i.csv; done
cd /home/krishna/catkin_ws/src/joystick_bwi/bagfiles/joint_states
for i in $(ls *.bag); do rostopic echo -b $i -p joint_states > $i.csv; done