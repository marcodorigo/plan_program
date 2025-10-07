from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the path converter node
        Node(
            package='plan_program',
            executable='path_converter_node',
            name='path_converter_node',
            output='screen',
            parameters=[]
        ),
    ])