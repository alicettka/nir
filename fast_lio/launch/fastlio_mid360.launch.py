from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            remappings=[
                ('/points_raw', '/livox/lidar/points/points'),
                ('/imu/data', '/imu_sensor')
            ]
        )

    ])
