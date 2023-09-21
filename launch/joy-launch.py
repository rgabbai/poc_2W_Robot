import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.substitutions import Command
# for adding delay mechansim 
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import ExecuteProcess 
def generate_launch_description():
  
    package_name='dev_ws'

    teleop_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource('src/teleop_twist_joy/launch/ros2_control_teleop-launch.py'
                ), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joy_to_topic_node = Node(
            package="joy_to_array_topic",
            executable="joy_to_array_node",
            name="joy_to_array_node",
            remappings={('/custom_array', '/joint_whacker/commands')},
    )

    # Launch them all!
    return LaunchDescription([
        teleop_node,
        joy_to_topic_node
    ])
