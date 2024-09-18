"Temp hack" - to be fixed later...
Note this is the base repo for robot_ws directory
clone this repo to a new dir name robot_ws
crteate src dir -  clone other required repos under src: <br>
robot_orientation/<br>
teleop_twist_joy/<br> 
diffdrive_arduino/<br>
joy_to_array_topic/<br>
mpu6050/<br>
robot_control_pkg/<br>
serial/<br>



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


colcon build --symlink-install
source install/setup.bash






