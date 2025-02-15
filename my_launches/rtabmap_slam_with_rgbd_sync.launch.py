from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    topic_rgb,topic_depth,topic_caminfo ="camera/rgb/image_color","camera/depth/image","camera/rgb/camera_info" 

    parameters=[{
          'frame_id':'kinect',
          'subscribe_depth':True,
          'subscribe_rgbd':True,
          'subscribe_odom_info':True,
          'sync_queue_size':25,
          #'approx_sync_max_interval':0.15,
          #'approx_sync':False,

          # RTAB-Map's parameters should all be string type:
          'Odom/Strategy':'0',
          'Odom/ResetCountdown':'15',
          'Odom/GuessSmoothingDelay':'0',
          'Rtabmap/StartNewMapOnLoopClosure':'true',
          'RGBD/CreateOccupancyGrid':'false',
          'Rtabmap/CreateIntermediateNodes':'true',
          'RGBD/LinearUpdate':'0',
          'RGBD/AngularUpdate':'0'}]
          
    remappings=[
          ('rgb/image', topic_rgb),
          ('rgb/camera_info', topic_caminfo),
          ('depth/image', topic_depth)]
          

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=remappings
        ),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),

    ])

