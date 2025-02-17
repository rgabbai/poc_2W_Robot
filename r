cd src
colcon build
source install/setup.bash
 
cd ..

# Activate Robot ROS control
ros2 launch poc_2W_Robot launch_robot2.launch.py &

# Activate robot_control_pkg (Android app backend)
ros2 run robot_ctrl_pkg android_pub &

# Activate MPU6050
#======================
## MPU node
#ros2 run mpu6050 imu_publisher_node &
## Calman filter 
#ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p gain:=0.2 -r /imu/data_raw:=/mpu6050/imu/data &
## sensor offset correction and rpy srv  
#ros2 run mpu6050 imu_offset_corrector & 

### TODO Relpace below code with ROS only code - auto mode is old Rust code 
### Is currently used for Robot  phone app 

cd ..
cp libonnxruntime.so.1.15.1 /usr/lib/.

# Activate top level  robot control Rust program 
cd tracking_node
./target/release/auto_mode &


