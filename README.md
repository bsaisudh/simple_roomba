[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/bsaisudh/simple_roomba/blob/master/LICENSE)
# ENPM 808F Wandering Robot

## Overview

A simple robot that will move in the simulated gazebo space by avoiding the obstacles.

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/bsaisudh/simple_roomba/blob/master/LICENSE) file for details

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* catkin
* Ubuntu 16.04
* Gazebo
* turtlebot_gazebo

## ROS installation

ROS Kinetic is needed for this tutorial. Visit the [ROS Kinetic installation](http://wiki.ros.org/kinetic/Installation) page and follow the steps.

## ROS package dependencies

cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
geometry_msgs
sensor_msgs

Note: Most of the packages are by default installed with ROS full development installation

## Installing turtlebot_gazebo

Run the below command to intall the package

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

## Steps for building package

* Install catkin
```
sudo apt-get install ros-kinetic-catkin
```
* Setup Catkin Workspace
```
mkdir path_to_catkin_workspace
cd path_to_catkin_workspace
mkdir src
cd src
```
* Clone github
```
cd path_to_catkin_workspace/src
git clone -b teleop-wander https://github.com/bsaisudh/simple_roomba.git
```
* Build package and install using catkin
```
cd path_to_catkin_workspace
catkin_make install
source ./devel/setup.bash
(source ./devel/setup.zsh  // For zsh shell)
```
