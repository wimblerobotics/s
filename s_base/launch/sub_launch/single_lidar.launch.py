from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='serial_port_right_rear', 
            default_value='/dev/lidar_right_rear',
            description='LD06 Serial Port for right_rear device'
        ),
        DeclareLaunchArgument(
            name='range_threshold', 
            default_value='0.0',
            description='Range Threshold'
        ),
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar_right_rear',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            parameters=[
                {'serial_port': LaunchConfiguration("serial_port_right_rear")},
                {'topic_name': "scan"},
                {'lidar_frame': "scan"},
                {'range_threshold': LaunchConfiguration("range_threshold")}
            ]
        ),

    ])
