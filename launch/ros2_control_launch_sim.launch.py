import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
# for adding delay mechansim 
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
 
def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='poc_2W_Robot' #<--- CHANGE ME
 
    #rsp = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','rsp.launch.py'
    #            )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)

    pkg_path = os.path.join(get_package_share_directory('poc_2W_Robot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    #print("*************************************************xacro: "+str(xacro_file)+"\n")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_raw = robot_description_config.toxml()
    #print ("************************************************* raw:"+str(robot_description_raw)+"\n")

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_raw , 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo_params_path = os.path.join(
                  get_package_share_directory(package_name),'config','gazebo_params.yaml')
 
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
             )
 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'poc_W2_Robot'],
                        output='screen')
 

    diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=["diff_cont"],
    )
    delayed_diff_drive_spawner= TimerAction(period=3.0,actions=[diff_drive_spawner])


    joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner= TimerAction(period=3.0,actions=[joint_broad_spawner])


 
    # Launch them all!
    return LaunchDescription([
        #rsp,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])