from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('turret_description'), 'urdf', 'turret.urdf')
    controllers_file = os.path.join(get_package_share_directory('turret_hardware'), 'config', 'controllers.yaml')
    
    with open(urdf_file, 'r') as file:
        urdf = file.read()

    # Just to publish the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
    )

    ros2_controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_file]
    )

    forward_position_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '--controller-manager-timeout',
            '300',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_controller_manager_node,
        forward_position_controller_node
    ])
