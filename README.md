# segbot_arm

This holds the all projects for use by the BWI segbot's arm manipulators.
Currently this includes: Kinova Mico

To run these packages, you must first install the utexas-bwi fork of the jaco ros drivers. You can do that with

$ git clone https://github.com/utexas-bwi/jaco-ros.git

and compile with catkin.

Before compiling, install the following dependencies:

1. Install MoveIt! by typing:

$ sudo apt-get install ros-indigo-moveit-*

2. Install libsndfile by typing:

$ sudo apt-get install libsndfile1-dev

3. Install libfft3:

$ sudo apt-get install libfftw3-dev



