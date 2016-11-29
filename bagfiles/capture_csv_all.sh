#!/bin/bash
x-terminal-emulator -e /home/bzsinger/catkin_ws/src/joystick_bwi/bagfiles/joint_states/joint_states_capture_csv.sh &
x-terminal-emulator -e /home/bzsinger/catkin_ws/src/joystick_bwi/bagfiles/joint_states/tool_position_capture_csv.sh &
x-terminal-emulator -e /home/bzsinger/catkin_ws/src/joystick_bwi/bagfiles/joint_states/joint_efforts_capture_csv.sh &
x-terminal-emulator -e /home/bzsinger/catkin_ws/src/joystick_bwi/bagfiles/joint_states/finger_position_capture_csv.sh
