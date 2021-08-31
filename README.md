# Turtlebot3_mapping
Using Turtlebot3 + RealSense D435i for mapping both in 2D and 3D. 

In this repository you can find the launch files and some scripts to run the mapping smoothly. In order to work you first needs to install the packages. 

All this software is run under ROS Melodic. Follow the guide in https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/ (Quick Start Guide) to get you started with Turtlebot3 and configure all the basics packages. Also you need to set the IP data in the PC and the Turtlebot. 

First you need to create a catkin_ws to put all the packages: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To install Hector SLAM:

```
cd ~/catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam

```

To install RealSense D435i drivers and packages: https://github.com/IntelRealSense/realsense-ros
