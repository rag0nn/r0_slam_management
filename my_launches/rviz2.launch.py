from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RViz config dosyasının yolunu alıyoruz
    package_name = 'r0'
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'my_config.rviz'
    )

    # RViz2 düğüm tanımı
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],  # Config dosyasını belirtmek için '-d' kullanılır
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
