import os
import xacro
import yaml

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    def get_urdf_path(context, *args, **kwargs):
        urdf_leaf_path = LaunchConfiguration('urdf_path_leaf_name').perform(context)
        print(f'urdf_leaf_path: {urdf_leaf_path}')
        ld = []
        joint_state_configFilePath = os.path.join(
            s_description_directory_path,
            'config',
            'joint_state_publisher.yaml'
        )

        urdf_path = os.path.join(s_description_directory_path, 'urdf/', urdf_leaf_path)
        
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
        ld.append(robot_state_publisher_node)

        joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[joint_state_configParams],
            condition=IfCondition(publish_joints)
        )
        ld.append(joint_state_publisher_node)
        return ld

    
    
    urdf_path_leaf_name  = LaunchConfiguration('urdf_path_leaf_name')
    publish_joints = LaunchConfiguration('publish_joints')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()
    s_description_directory_path = get_package_share_directory('s_description')

    ld.add_action(DeclareLaunchArgument(
        name='publish_joints', 
        default_value='true',
        description='Launch joint_states_publisher if true'))

    ld.add_action(DeclareLaunchArgument(
        name='urdf_path_leaf_name', 
        default_value='base.urdf.xacro',
        description='Leaf name of urdf'))

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(OpaqueFunction(function=get_urdf_path))

    # joint_state_configFilePath = os.path.join(
    #     s_description_directory_path,
    #     'config',
    #     'joint_state_publisher.yaml'
    # )

    # urdf_path = os.path.join(s_description_directory_path, 'urdf/', urdf_leaf_path)
    
    # with open(joint_state_configFilePath, 'r') as file:
    #     joint_state_configParams = yaml.safe_load(file)['joint_state_publisher']['ros__parameters']   

    # robot_description_raw = xacro.process_file(urdf_path).toxml()

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description_raw,
    #                  'use_sim_time': use_sim_time
    #                  }]
    # )
    # ld.add_action(robot_state_publisher_node)

    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[joint_state_configParams],
    #     condition=IfCondition(publish_joints)
    # )
    # ld.add_action(joint_state_publisher_node)

    return ld
