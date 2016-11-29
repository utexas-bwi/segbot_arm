#!/bin/bash
x-terminal-emulator -e /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/joint_states/joint_states_capture_csv.sh &
x-terminal-emulator -e /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/tool_position/tool_position_capture_csv.sh &
x-terminal-emulator -e /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/joint_efforts/joint_efforts_capture_csv.sh &
x-terminal-emulator -e /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/finger_position/finger_position_capture_csv.sh &
x-terminal-emulator -e /home/users/fri/lafd_ws/src/joystick_bwi/bagfiles/point_cloud/point_cloud_capture_pcd.sh 
