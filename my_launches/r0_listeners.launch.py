from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
          
    return LaunchDescription([
        Node(package='r0', executable='listener_rgb', output='screen',),
        Node(package='r0', executable='listener_caminfo', output='screen',),
        Node(package='r0', executable='listener_depth', output='screen',),
    ])
