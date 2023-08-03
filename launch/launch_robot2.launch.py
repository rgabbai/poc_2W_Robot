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
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='poc_2W_Robot' #<--- CHANGE ME
 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp2.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

   
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params = os.path.join(get_package_share_directory(package_name),'config','my_controllers2.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                controller_params],
        )
    delayed_controller_manager = TimerAction(period=2.0,actions=[controller_manager])
 
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        parameters=[{"log-level": "DEBUG"}]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    joint_whacker_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_whacker"],
        parameters=[{"log-level": "DEBUG"}]
    )

    delayed_joint_whacker_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_whacker_spawner],
        )
    )

    lidar = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                #'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
    )

     # Create a node to execute the service call
    service_call_node = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/stop_motor', 'std_srvs/srv/Empty', '{}'],
        output='screen'
    )
 
    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_joint_whacker_spawner,
        #lidar,
        service_call_node
    ])
