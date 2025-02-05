## 2 Wheel Robot POC
Goal
Teleoperated robot capable to cut small weeds using a joystic and camera

Background
This robot is build on top of robotmow skelaton and motors


This package contain the robot and sensors defintions
Robot description xacro files
gazebo related files
lanuch files 
launch scripts

How to run
create new work spaces in both dev and robot linux machines 
gaebo mode require only dev machine
robot mode require both - dev macnine holds joystic package and camera (TBD)

dev machine:
mkdir dev_ws
mkdir dev_ws/src 
cd dev_ws/src
clone the following repos
1. This repo (poc_2W_Robot)
2. teleop_twist_joy

robot machine:
Prepare robot_ws in robot linux system

2 Build stage:

cd src
colcon build
source install/setup.bash
cd ..
colcon build
source install/setup.bash








