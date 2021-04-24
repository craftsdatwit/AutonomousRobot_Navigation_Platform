# Pallet Project

## Introduction 

Using ROS and Gazebo, this project enables a robot to navigate a warehouse environment with path-following and collision avoidance algorithms.

## Features
1.) Collision avoidance

2.) Object Detection

3.) Path Following

## Getting Started

### Installation

1.) Install Ubuntu 20.04 LTS (or later) on a virtual machine or stand alone machine [VirtualBox](https://www.virtualbox.org/).

2.) Upon installation, visit [ROS Install](http://wiki.ros.org/Installation/Ubuntu) and enter the following commands on a sudo terminal session. 

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-noetic-desktop-full #Full installation of ROS Noetic / Gazebo
```
4.) Following the tutorial here: [ROS Tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) create a catkin_ws folder in your home directory.

5.) Upon creation of your ~/catkin_ws folder, navigate to this website: [Automatic Addison](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/) and follow the steps to clone the turtlebot3 repo into your ~/catkin_ws folder. *Note: Follow the instructions for the Waffle Pi Turtlebot3 robot.

### Running the Simulation

```bash
~/catkin_ws 

source devel/setup.bash

rosrun turtlebot3

```

## Demo Video

## Road Map
* **Release 1.0** 
    * Establish a system that enables robot to navigate from point A to B with a lightweight version of the collision avoidance algorithms. This will enable the robot to come to a complete stop and wait until detected object is removed before continuing to its destination. 
* **Release 1.5**
    * Upgrade from release 1.0 to a more robust version of the collision avoidance software. In this release the robot would be able to make its way around the object wihtout having the obstruction to be removed.
* **Release 2.0**
    * This release would have the ability to add a second robot to the simulation.

## Contributors

- David Crafts (craftsd@wit.edu), Team Lead / Collision Avoidance Developer
- Jake Sousa (sousaj8@wit.edu), Collision Avoidance Developer
- Nathan Robson
- William
