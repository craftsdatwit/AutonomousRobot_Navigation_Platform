# Autonomous Robot Navigation Platform

## Introduction 

Using ROS and Gazebo, this platform enables robots to navigate a warehouse environment, following a unique path and avoiding collisions with objects

## Features
1.) Collision avoidance / Object Detection

The robot control class 'Robot_Control_Class.py', located in Pallet_Project/controller/src/ contains all of the necessary functions for full collision avoidance / object detection. While not all of the functions were implemented into the path following algorithm, they are ready to be implemented within the robot control class. Originally the functions were in a standard file, however, they were then broken out into a class file for easier implementation and integration with any algorithm necessary. Any robot algorithm developed in this manner can use our robot control class to implement full collision avoidance / object detection with ease. 

2.) Path Following

The path following script, Robot_Path_Following.py located in Pallet_Project/controller/src/, is what enables the robot to calculate the fastest path of any two points on the known map and drive autonomously from its current poisition to the new calculated point. This navigation system takes advantage of the A* algorithm, a graph traversal and path search algorithm, which is known for its optimality. This algorithm requires the creation of nodes, which coincide with the points of the graph in the search. The starting node is the first location, which then branches to all surrounding nodes, and calculating what is called the cost, or the numerical value of one node to another. those connected nodes are considered the 'children' nodes of the starting 'parent' node. The children then branch there own children nodes, and add that cost to the total cost of that path. This continues branching until the end point is reached, and the path of nodes with the least overall cost is the shortest path. The only real issue with this algorithm, the node data is saved locally, so on a very expansive map memory issues could occur, however on grids of smaller sixe this is not an issue.

3.) Multi-Robot Support

New in version 2.0 is the support for multiple robots with their own unqiue path following and collision avoidance. This allows for testing scalability of the platform across multiple robots given a multitude of test scenarios. 

### Getting Started / Running the Code

## Installation

1.) Install Ubuntu 20.04 LTS (or later) on a virtual machine [VirtualBox](https://www.virtualbox.org/) or stand alone machine.

2.) Upon installation, visit [ROS Install](http://wiki.ros.org/Installation/Ubuntu) and enter the following commands on a sudo terminal session. 

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-noetic-desktop-full #Full installation of ROS Noetic / Gazebo
```
3.) Following the tutorial here: [ROS Tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) create a catkin_ws folder in your home directory.

4.) Upon creation of your ~/catkin_ws folder, navigate to this website: [Automatic Addison](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/) and follow the steps to clone the turtlebot3 repo into your ~/catkin_ws folder. *Note: Follow the instructions for the Waffle Pi Turtlebot3 robot.

5.) For multi-robot support please follow the guide here: https://mywentworth-my.sharepoint.com/:w:/g/personal/craftsd_wit_edu/ERLpnHlVpF5Mv47ig-ZlgO0BGA20GwfDFfZiV_AzffQYKw?e=Zmn8Jd

6.) To get the demo world installed, please follow the readme instructions within the Warehouse directory.

## Running the Code

Once all other steps have been followed, from ~/catkin_ws run a source devel/setup.bash then run 

```bash

roslaunch multi_robot warehousemultirobot.launch

```

This should launch the demo world with 5 robots.

8.) To get the robots moving, each robot has a corresponding python file within the /controller/src directory that can be run by browing to the directory then entering 

```bash

python3 Robot1.py

```

This loads in the robots unique path map found within the Warehouse Map files directory under /controller/src. The @ represents the start symbol and the $ represnets the end goal.

9.) Each robot will need it's own terminal tab and will need to be luanched like in step 8 to be run cocurrently. The large blocks of 1's within each map are predefined "no-go zones" that can be easily changed to 0's to represent free spaces.

10.) These maps can be modified but are to scale of the enviornment, feel free to drag in dynamic obstacles from gazebo or play around with different no go zones. This is a sanbox envionrnment so have fun!

## Demo Video

https://youtu.be/JdTmplTQyB0

The demo video highlights the integration of both path-following and collision avoidance algorithms with multi-robot support. This was run within the demo enviornment located in the Warehouse Files directory with the large rectangles being no go zones and the smaller boxes being dynamic obstacles for the robots to avoid. This run was sped up as it took over 10 minutes to run and highlights the core functionaly of the platform.

https://youtu.be/kYpcAFGEUJY

This video is a lighthearted and comical demo video showcasing two robots with voiceover commentary about what is occuring during the video. This video was recorded for Wentworth's Senior Project showcase

## Road Map
* **Release 2.5** 
    * Migrate the current implementation off of Gazebo to a more robust graphical application such as Unity.
* **Release 3.0**
    * Implement a reverse function and other minor code refinements to improve the experience.
* **Release 4.0**
    * Implement multi-robot awareness and dynamic path editing for a more robust navigation system allowing robots to be aware of other robots current locations, marking them as no go zones. Also allowing each robot to dynamically edit it's own path, recalcuating based on given stimuli. 

## Contributors

- David Crafts (craftsd@wit.edu), Team Lead / Collision Avoidance Developer
- Jake Sousa (sousaj8@wit.edu), Collision Avoidance Developer
- - William McLellan (mclellanw@wit.edu), Autonomous Navigation Developer
- Nathan Robson (robsonn@wit.edu), Autonomous Navigation Developer
