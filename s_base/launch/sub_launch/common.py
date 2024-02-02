from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
import os

# joystick_directory_path = get_package_share_directory('s_bluetooth_joystick')
# multiplexer_directory_path = get_package_share_directory('twist_multiplexer');
s_base_directory_path = get_package_share_directory('s_base')
s_description_directory_path = get_package_share_directory('s_description')
rviz_directory_path = get_package_share_directory('rviz')
urdf_path = os.path.join(s_description_directory_path, 'urdf/s.urdf.xacro')
use_sim_time = LaunchConfiguration('use_sim_time')
use_state_pub_gui = LaunchConfiguration('use_state_pub_gui')

# ekf_config_path = os.path.join(s_base_directory_path, 'config/ekf.yaml')

ld = LaunchDescription()

# ld.add_action(DeclareLaunchArgument(
#     name= 'map', 
#     default_value='map5.yaml',
#     description='map file to use for navigation'))

# ld.add_action(DeclareLaunchArgument(
#     name= 'mapping', 
#     default_value='True',
#     description='set to true to run mapping mode'))

ld.add_action(DeclareLaunchArgument(
    name='model', 
    default_value=urdf_path,
    description='Absolute path to robot urdf file'))

ld.add_action(DeclareLaunchArgument(
    name='use_sim_time', 
    default_value='false',
    description='Use simulation (Gazebo) clock if true'))

ld.add_action(DeclareLaunchArgument(
    name='use_state_pub_gui', 
    default_value='false',
    description='Use state publisher GUI'))

ld.add_action(DeclareLaunchArgument(
    name= 'world', 
    default_value='simbot2.world',
    description='If using simulation - gazebo world to launch'))


# map = LaunchConfiguration('map')
# map_path = os.path.join(s_base_directory_path,'maps','')
# map_file_name=PathJoinSubstitution([TextSubstitution(text=map_path), map])
