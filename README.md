# segbot_arm [![Build Status](https://travis-ci.org/utexas-bwi/segbot_arm.svg?branch=master)](https://travis-ci.org/utexas-bwi/segbot_arm)

This holds the all projects for use by the BWI segbot's arm manipulators.
Currently this is just the Mico2.

## Installation

To run these packages, you must first install the utexas-bwi fork of the jaco ros drivers. You can do that by going into catkin_ws/src and then cloning the repository with

    git clone https://github.com/utexas-bwi/kinova-ros.git

and compile with catkin.

Before compiling, install the following dependencies:

Install MoveIt!, TRAC-IK, libsndfile, Lapack and libfftw with

    sudo apt-get install ros-indigo-moveit-* libsndfile1-dev libfftw3-dev liblapack-dev ros-indigo-trac-ik

Install our fork of agile_grasp ROS indigo package by cloning from:

    git clone https://github.com/jsinapov/agile_grasp.git
