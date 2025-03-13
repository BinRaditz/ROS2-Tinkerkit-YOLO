from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tinkerkit_controller',
            executable='robotarm_node',
            name='robotarm_node',
            parameters=['config/tinkerkit_params.yaml']
        ),
    ])