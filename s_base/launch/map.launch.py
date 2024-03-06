import os
import sys
import xacro
import yaml

# import launch
import launch_ros.actions
# from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


s_sub_launch_path = os.path.join(get_package_share_directory('s_base'),
                                    'launch', 'sub_launch')
sys.path.append(s_sub_launch_path)
import common

def generate_launch_description():

    use_sim_time = False

    base_launch_path = PathJoinSubstitution(
        [FindPackageShare('s_base'), 'launch', 'sub_launch', 'base.launch.py']
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        # launch_arguments={
        #     'map': default_map_path,
        #     'use_sim_time': str(use_sim_time),
        #     'params_file': nav2_config_path
        # }.items()
    )
    common.ld.add_action(base_launch)

    # Bring up SLAM.
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = os.path.join(common.s_base_directory_path, 'config', 'slam.yaml')        
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'slam_params_file': slam_config_path
        }.items()
    )
    common.ld.add_action(slam_launch)

    return common.ld
