# Guided policy search on the Franka-Emika Panda Robot

## Overview

This repository contains three sub-repositories.
1. franka-ros : contains ros-packages to operate the franka robot, both in the real world and in gazebo.
2. gps : This is a copy of Chelsea Finn's implementation and demonstrations of Sergey Levine and Vladlen Koltun's guided policy search algorithm, with the addition of a ROS controller and GPS agent and demonstrations for the KUKA LWR 4+ by Jack White. We have added an interface for the Franka-Emika panda robot using Jack White's additions.

3. kuka-lwr : contains ros-packages to operate the KUKA LWR 4+, both in the real world and in gazebo.

Jack White's modifications : Unlike the demonstration ROS controller for the PR2 robot which comes with Finn's code, the GPS controller uses `ros_control` and should be easily modified for other robots with standard `ros_control` hardware interfaces.
This code is a reimplementation of the guided policy search algorithm and LQG-based trajectory optimization, meant to help others understand, reuse, and build upon existing work.

For full documentation, see [rll.berkeley.edu/gps](http://rll.berkeley.edu/gps).

The code base is **a work in progress**. See the [FAQ](http://rll.berkeley.edu/gps/faq.html) for information on planned future additions to the code.



- [Guided Policy Search](http://proceedings.mlr.press/v28/levine13.html)
- [End-to-End Training of Deep Visuomotor Policies](https://arxiv.org/abs/1504.00702), the paper on which Chelsea Finn's software is based
- [Chelsea Finn's source](https://github.com/cbfinn/gps)
- [Documentation](http://rll.berkeley.edu/gps/)
- [Jack White's modified GPS source](https://bitbucket.org/JackNWhite/gps/src/master/)
- [Franka-ROS package](https://github.com/frankaemika/franka_ros)
- [LWR hardware interface](https://github.com/CentroEPiaggio/kuka-lwr) Centro di ricerca Enrico Piaggio's KUKA  for ROS

## Use

### Requirements

- [ROS Melodic](http://www.ros.org)
- [Caffe](http://caffe.berkeleyvision.org/) deep-learning framework (v1)
- This Git repository
- Python 2.7

#### Brief instructions

- Install ROS and set up a Catkin workspace
- Build and install Caffe in accordance with the instructions on its web site, making sure to build the distribution files
- Clone the franka_ros and kuka-lwr packages into `catkin_workspace/src` and build with `catkin_make`
- Clone GPS into `catkin_workspace/src`
- Build the GPS controllers using `cmake .` and then `make` from `catkin_workspace/src/gps/src/gps_agent_pkg`
- Set up paths by adding the following to your ~/.bashrc file:

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/gps:/path/to/gps/src/gps_agent_pkg`
- Run Gazebo simulation and load the controller with `roslaunch gps_agent_pkg franka_gazebo.launch`
- Run the GPS backend and start the example experiment from `catkin_workspace/src/gps` using `python python/gps/gps_main.py gazebo_franka_example`

For more detailed instructions, see the ROS and Caffe web sites and the Franka-ROS hardware, original GPS and Jack White's GPS repositories.

--------------------------------------------------------------------------------
