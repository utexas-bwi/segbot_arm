<<<<<<< HEAD
# segbot_arm

This holds the all projects for use by the BWI segbot's arm manipulators.
Currently this includes: Kinova Mico

To run these packages, you must first install the utexas-bwi fork of the jaco ros drivers. You can do that by going into catkin_ws/src and then cloning the repository with

$ git clone https://github.com/utexas-bwi/jaco-ros.git

and compile with catkin.

Before compiling, install the following dependencies:

Install MoveIt! by typing:

$ sudo apt-get install ros-indigo-moveit-*

Install libsndfile by typing:

$ sudo apt-get install libsndfile1-dev

Install libfft3:

$ sudo apt-get install libfftw3-dev

Instal Lapack:

$ sudo apt-get install liblapack-dev

Install our fork of agile_grasp ROS indigo package by cloning from:

https://github.com/jsinapov/agile_grasp

=======
# Controller for BWI Kinova arm, Segway base

Uses the "joy" ROS package. 

## Arm controls:  
Linear velocity x - Left axis stick left/right  
Linear velocity y - Left axis stick up/down  
Linear velocity z - Left trigger/Right trigger  
  
Angular velocity x - Right axis stick left/right  
Angular velocity y - Right axis stick up/down  
Angular velocity z - Left back button/Right back button  
  
### Fingers:  
Open slowly - 4-direction pad LEFT  
Close slowly - 4-direction pad RIGHT  
Open fully - 4-direction pad UP  
Close fully - 4-direction pad DOWN  
  
Home arm - Center button  
Switch modes - Back + Start buttons  

## Segway base controls:  
Forward/Backward - Left axis stick up/down  
Turn - Right axis stick left/right  

Increase/Decrease speed - Y/A  
Increase/Decrease turn speed - X/B  
>>>>>>> b167ee15d0ab8d9598d9ce6f1347011a358675da
