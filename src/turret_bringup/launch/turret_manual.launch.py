from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turret_bringup'),
                'launch',
                'ros2_control.launch.py'
            )
        )
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    turret_teleop_node = Node(
        package='turret_teleop',
        executable='turret_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        ros2_control_launch,
        joy_node,
        turret_teleop_node
    ])
