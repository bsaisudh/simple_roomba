[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/bsaisudh/simple_roomba/blob/master/LICENSE)
# ENPM 808X simple_roomba

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
* Clone beginner_tutorials repository
```
cd path_to_catkin_workspace/src
git clone --recursive https://github.com/bsaisudh/simple_roomba.git
```
* Build package and install using catkin
```
cd path_to_catkin_workspace
catkin_make install
source ./devel/setup.bash
(source ./devel/setup.zsh  // For zsh shell)
```
## Running with launch file

* Both Gazebo environment and the simple_romba navigator node can be launched at the same time using launch file using below command 
<br>Argument - bagrecord : The argument 'record' whether to record rosbag files or not. It is optional and if not specified the default value is false. The messages will be recorded for a duration of 30 seconds. Bag files will be created with unique time stamps.
```
roslaunch simple_roomba simple_roomba.launch bagrecord:=1
```
* The recored bag file will be found below folder
```
<package_folder>/rosbag
```
### Playing rosbag file
Run the following commands in separate termina.
```
roslaunch turtlebot_gazebo turtlebot_world.launch
rosbag play <path_to_Package>/rosbag/<rosbar_file_name>.bag
```

### Tutorial Output running from launch file

</p>
<p align="center">
<img src="/readme_images/Launch_execution.png">
</p>
</p>


## ROS Graph

</p>
<p align="center">
<img src="/readme_images/rosgraph.png">
</p>
</p>

## Working with tf Frames
* Run the simple_romba launch file. The nodes will be broadcasting the different types of transformations.
* viewing the frames
```
rosrun tf view_frames
```
sample output
```
Listening to /tf for 5.000000 seconds
Done Listening
dot - graphviz version 2.38.0 (20140413.2041)

Detected dot version 2.38
frames.pdf generated
```
* Alternatively the same information can be for using rqt_tf_tree
```
rosrun rqt_tf_tree rqt_tf_tree
```
</p>
<p align="center">
<img src="/readme_images/frames.png">
</p>
</p>


### Visuvalizing using RVIZ

* Run the simple_romba launch file
* Open rviz using following command
```
roslaunch turtlebot_teleop keyboard_teleop.launch
```
* load the rviz config file from file menu. The config file can be found in <package_src_folder>/rviz/Romba_rviz.rviz

</p>
<p align="center">
<img src="/readme_images/RvizOutput.png">
</p>
</p>

