# Gazebo SAUVC environment

Gazebo SAUVC environment integrated with ROS

[![](https://img.youtube.com/vi/jII8SlZvBcM/0.jpg)](https://www.youtube.com/watch?v=jII8SlZvBcM)

## Prerequisites

- [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

Add the following lines to ~/.bashrc

```sh
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.bash
```

## Installation

Create ROS workspace and clone the repo:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:NCTU-AUV/sauvc_sim.git
```

Build the code

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Getting started

Launch the environment

```sh
cd ~/catkin_ws
# lauch the SAUVC arena
roslaunch sauvc_sim sauvc.world
# lauch an empty world with an auv inside
roslaunch sauvc_sim empty_world.world
```

Launch keyboard controller

```sh
rosrun sauvc_sim teleop.py
```
