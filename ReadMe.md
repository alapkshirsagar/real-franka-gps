# Guided policy search on the Franka-Emika Panda Robot

## Overview

This repository contains three sub-repositories.
1. franka-ros : contains ros-packages to operate the franka robot, both in the real world and in gazebo.
2. gps : This is a copy of Chelsea Finn's implementation and demonstrations of Sergey Levine and Vladlen Koltun's guided policy search algorithm, with the addition of a ROS controller and GPS agent and demonstrations for the KUKA LWR 4+ by Jack White. We have added an interface for the Franka-Emika panda robot using Jack White's additions.

3. kuka-lwr : contains ros-packages to operate the KUKA LWR 4+, both in the real world and in gazebo.

Jack White's modifications : Unlike the demonstration ROS controller for the PR2 robot which comes with Finn's code, the GPS controller uses `ros_control` and should be easily modified for other robots with standard `ros_control` hardware interfaces.
This code is a reimplementation of the guided policy search algorithm and LQG-based trajectory optimization, meant to help others understand, reuse, and build upon existing work.

Our modifications: We used Franka hardware interface instead of Kuka interface that was used in White's code. We connected OptiTrack motion tracking system to feed live positions of the robot and the target to the GPS algorithm. We replaced the realtime trial data publisher (publishes the data acquired in each trial) with a non realtime publisher. Other small modifications include tuning parameters of the initial local controllers, tuning parameters of the initial dynamics model, and clamping the joint torques within certain limits before they are sent to the robot. 

For full documentation, see [rll.berkeley.edu/gps](http://rll.berkeley.edu/gps).

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
```
cmake . -DUSE_CAFFE=1 -DCAFFE_INCLUDE_PATH=/home/pascal/caffe/distribute/include -DCAFFE_LIBRARY_PATH=/home/pascal/caffe/build/lib
```
- Clone the franka_ros and kuka-lwr packages into `catkin_workspace/src` and build with `catkin_make`
- Clone GPS into `catkin_workspace/src`
- Install ROS packages: "convex-decomposition", "ivcon", "pr2_description", "pr2_controllers"
- Build the GPS controllers using `cmake .` and then `make` from `catkin_workspace/src/gps/src/gps_agent_pkg`
- Set up paths by adding the following to your ~/.bashrc file:

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/gps:/path/to/gps/src/gps_agent_pkg`


Gazebo Model:
- Run Gazebo simulation and load the controller with `roslaunch gps_agent_pkg franka_gazebo.launch`
- Run the GPS backend and start the example experiment from `catkin_workspace/src/gps` using `python python/gps/gps_main.py gazebo_franka_example`

Real Robot:
- on the Franka computer, run: `roslaunch gps_agent_pkg franka_real.launch robot_ip:=172.16.0.2`
- Run the GPS backend and start the example experiment from `catkin_workspace/src/gps` using `python python/gps/gps_main.py gazebo_franka_example`

in case we have error:
rostopic pub /panda1/franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "header:

For more detailed instructions, see the ROS and Caffe web sites and the Franka-ROS hardware, original GPS and Jack White's GPS repositories.

--------------------------------------------------------------------------------
## GPS PC
# Terminal 1
export ROS_MASTER_URI=http://132.72.96.38:11311
export ROS_IP=132.72.96.38
source devel/setup.bash
roslaunch mocap_optitrack mocap.launch

## GPS PC
# Terminal 2
export ROS_MASTER_URI=http://132.72.96.38:11311
export ROS_IP=132.72.96.38
source devel/setup.bash
rosrun gps_agent_pkg optitrack_publisher.py


## Franka PC
## Terminal 1
export ROS_MASTER_URI=http://132.72.96.38:11311
export ROS_IP=132.72.96.57
source devel/setup.bash
roslaunch gps_agent_pkg franka_real.launch robot_ip:=172.16.1.2

## GPS PC
# Terminal 3 (for training real robot)
export ROS_MASTER_URI=http://132.72.96.38:11311
export ROS_IP=132.72.96.38
source devel/setup.bash
python src/real-franka-gps/gps/python/gps/gps_main.py real_franka_badmm_example

# Terminal 3 (for testing sim-to-real transfer)
export ROS_MASTER_URI=http://132.72.96.38:11311
export ROS_IP=132.72.96.38
source devel/setup.bash
cd src/real-franka-gps/gps/
python python/gps/gps_main.py real_franka_badmm_sim2real -p 1 10


MuJoCo to RealRobot - parameters to change in hyperparams file: 1. agent type 2. conditions 3. init_var 4. stiffness 5. stiffness_vel
                                           in cost_fk    : 1. jx_full
