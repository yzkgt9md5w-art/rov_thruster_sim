import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rov_thruster_sim'),
        'config',
        'thruster_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='rov_thruster_sim',
            executable='thruster_bridge_node',
            name='rov_thruster_sim_node',
            output='screen',
            parameters=[config]
        )
    ])