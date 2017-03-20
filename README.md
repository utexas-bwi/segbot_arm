# segbot_arm [![Build Status](https://travis-ci.org/utexas-bwi/segbot_arm.svg?branch=master)](https://travis-ci.org/utexas-bwi/segbot_arm)

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
