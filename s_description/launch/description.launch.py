import os
import sys
import xacro
import yaml

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory

s_sub_launch_path = os.path.join(get_package_share_directory('s_base'),
                                    'launch', 'sub_launch')
sys.path.append(s_sub_launch_path)
import common

def generate_launch_description():
    # do_rviz = os.getenv('DO_RVIZ', default=True) in (
    #     'True', 'true', '1', 't', 'T')
    # print(f'DO_RVIZ: {do_rviz}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_state_pub_gui = LaunchConfiguration('use_state_pub_gui')

    my_package_name = 's_description'
    pkg_share = get_package_share_directory(my_package_name)
    default_model_path = os.path.join(pkg_share, 'urdf', 'base.urdf.xacro')

    joint_state_configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'joint_state_publisher.yaml'
    )

    with open(joint_state_configFilePath, 'r') as file:
        joint_state_configParams = yaml.safe_load(file)['joint_state_publisher']['ros__parameters']   

    robot_description_raw = xacro.process_file(default_model_path).toxml()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': use_sim_time
                     }]
    )
    common.ld.add_action(robot_state_publisher_node)

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[joint_state_configParams],
        condition=launch.conditions.UnlessCondition(use_state_pub_gui)
    )
    common.ld.add_action(joint_state_publisher_node)

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(use_state_pub_gui)
    )
    common.ld.add_action(joint_state_publisher_gui_node)

    # Launch rviz2
    rviz_config_dir = os.path.join(
        common.rviz_directory_path, 'config', 'default_config.rviz')
    if True:
        rviz_node = launch_ros.actions.Node(package='rviz2',
                                            executable='rviz2',
                                            name='rviz2',
                                            output='screen',
                                            arguments=['-d', rviz_config_dir],
                                            parameters=[{
                                                'use_sim_time': common.use_sim_time
                                            }])
        common.ld.add_action(rviz_node)
        
    return common.ld
