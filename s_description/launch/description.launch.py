import os
import xacro
import yaml

import launch
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()
    s_description_directory_path = get_package_share_directory('s_description')
    urdf_path = os.path.join(s_description_directory_path, 'urdf/base.urdf.xacro')
    
    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    joint_state_configFilePath = os.path.join(
        s_description_directory_path,
        'config',
        'joint_state_publisher.yaml'
    )

    with open(joint_state_configFilePath, 'r') as file:
        joint_state_configParams = yaml.safe_load(file)['joint_state_publisher']['ros__parameters']   

    robot_description_raw = xacro.process_file(urdf_path).toxml()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': use_sim_time
                     }]
    )
    ld.add_action(robot_state_publisher_node)

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[joint_state_configParams],
        condition=launch.conditions.UnlessCondition(use_sim_time)
    )
    ld.add_action(joint_state_publisher_node)

    return ld
