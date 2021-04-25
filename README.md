# Pallet Project

## Introduction 

Using ROS and Gazebo, this project enables a robot to navigate a warehouse environment with path-following and collision avoidance algorithms.

## Features
1.) Collision avoidance / Object Detection

The robot control class 'Logic_Robot_Control_Class.py', located in Pallet_Project/controller/src/ contains all of the necessary functions for full collision avoidance / object detection. While not all of the functions were implemented into the path following algorithm, they are ready to be implemented within the robot control class. Originally the functions were in a standard file, however, they were then broken out into a class file for easier implementation and integration with any algorithm necessary. Any robot algorithm developed in this manner can use our robot control class to implement full collision avoidance / object detection with ease. 

2.) Path Following

The path following script, Logic_Robot_Path_Following.py located in Pallet_Project/controller/src/, is what enables the robot to calculate the fastest path of any two points on the known map and drive autonomously from its current poisition to the new calculated point. This navigation system takes advantage of the A* algorithm, a graph traversal and path search algorithm, which is known for its optimality. This algorithm requires the creation of nodes, which coincide with the points of the graph in the search. The starting node is the first location, which then branches to all surrounding nodes, and calculating what is called the cost, or the numerical value of one node to another. those connected nodes are considered the 'children' nodes of the starting 'parent' node. The children then branch there own children nodes, and add that cost to the total cost of that path. This continues branching until the end point is reached, and the path of nodes with the least overall cost is the shortest path. The only real issue with this algorithm, the node data is saved locally, so on a very expansive map memory issues could occur, however on grids of smaller sixe this is not an issue.

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
- William McLellan (mclellanw@wit.edu), Autonomous Navigation Developer
