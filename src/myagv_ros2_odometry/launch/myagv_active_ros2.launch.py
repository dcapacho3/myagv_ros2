from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myagv_ros2_odometry',
            executable='myagv_odom',
            name='odometry_node',
            output='screen'
        ),
        Node(
            package='myagv_ros2_odometry',
            executable='myagv_bringup',
            name='bringup_node',
            output='screen'
        )
    ])
