from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    rgb = Node(
        package='r0',
        executable='provider_rgb',
        name='provider_rgb')
    depth = Node(
        package='r0',
        executable='provider_depth',
        name='provider_depth')
    """
    depth = Node(
        package='r0',
        executable='provider_depth_via_estimation',
        name='provider_depth') 
    """
    caminfo = Node(
        package='r0',
        executable='provider_caminfo',
        name='provider_caminfo')
    return LaunchDescription([
        caminfo,
        depth,
        rgb
    ])
