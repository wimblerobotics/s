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

    # Bring up the navigation stack.
    default_map_path = PathJoinSubstitution(
        # [FindPackageShare('s_base'), 'maps', 'map6.yaml']
        [FindPackageShare('s_base'), 'maps', 'map5.yaml']
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    nav2_config_path =os.path.join(common.s_base_directory_path, 'config', 'navigation.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'map': default_map_path,
            'use_sim_time': str(use_sim_time),
            'params_file': nav2_config_path
        }.items()
    )
    common.ld.add_action(nav2_launch)

    return common.ld
