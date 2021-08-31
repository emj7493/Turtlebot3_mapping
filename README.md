# Turtlebot3_mapping
Using Turtlebot3 + RealSense D435i for mapping both in 2D and 3D. 

In this repository you can find the launch files and some scripts to run the mapping smoothly. In order to work you first needs to install the packages. 

## Tutorial

### Basic settings 
All this software is run under ROS Melodic. Follow the guide in https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/ (Quick Start Guide) to get you started with Turtlebot3 and configure all the basics packages. Also you need to set the IP data in the PC and the Turtlebot. 

First you need to create a catkin_ws to put all the packages: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

### Hector SLAM
To install Hector SLAM:

```
cd ~/catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam
```

### RealSense D435i
To install RealSense D435i drivers and packages: https://github.com/IntelRealSense/realsense-ros

### Packages from this repository
Once everything is configured you need to do the following with the code provided in this repo:

- Copy the launch file for Open_source_tracking in the launch foulder in RealSense ROS package.
- Copy HOG_detector inside src directory in your catkin workspae.
- Copy Hector_SLAM foulder in your src directory in your catkin workspace and perform _catkin_build_

With this you should be good to go.

## 2D mapping

You should execute the following:

### Turtlebot3:
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
### PC:
```
roscore
roslaunch hector_slam_launch SLAM_turtlebot.launch
```

## 3D mapping

You should execute the following:

### Turtlebot3:
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch realsense2_camera rs_camera.launch
```
### PC:
```
roscore
roslaunch realsense2_camera opensource_tracking_turtlebot3.launch
```


## People detection

You should execute the following:

### Turtlebot3:
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch realsense2_camera rs_camera.launch
```
### PC:
```
roscore
roslaunch realsense2_camera opensource_tracking_turtlebot3.launch
roslaunch hog_detector hog_detector.launch 
```
