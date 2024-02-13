import os
import sys
import xacro
import yaml

# import launch
import launch_ros.actions
# from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


s_sub_launch_path = os.path.join(get_package_share_directory('s_base'),
                                    'launch', 'sub_launch')
sys.path.append(s_sub_launch_path)
import common

def generate_launch_description():

    # Bring up the robot description (URDF).
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            common.s_description_directory_path, '/launch/description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': common.use_sim_time
        }.items()
    )
    common.ld.add_action(description_launch)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
              'world': os.path.join(common.s_description_directory_path, 'worlds', 'home.world')
            }.items()
        )
    common.ld.add_action(gazebo)

    spawn_entity = launch_ros.actions.Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'my_bot',
                    '-x' , '8', 
                    '-y', '3'],
        output='screen')
    common.ld.add_action(spawn_entity)

    return common.ld
