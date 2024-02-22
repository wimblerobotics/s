from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
import os

# joystick_directory_path = get_package_share_directory('s_bluetooth_joystick')
multiplexer_directory_path = get_package_share_directory('twist_multiplexer');
s_base_directory_path = get_package_share_directory('s_base')
s_description_directory_path = get_package_share_directory('s_description')
# rviz_directory_path = get_package_share_directory('rviz')
urdf_path = os.path.join(s_description_directory_path, 'urdf/base.urdf.xacro')
use_sim_time = LaunchConfiguration('use_sim_time')
ekf_config_path = os.path.join(s_base_directory_path, 'config/ekf.yaml')

ld = LaunchDescription()

ld.add_action(DeclareLaunchArgument(
    name= 'map', 
    default_value='home_map1.yaml',
    description='map file to use for navigation'))

ld.add_action(DeclareLaunchArgument(
    name='use_sim_time', 
    default_value='true',
    description='Use simulation (Gazebo) clock if true'))

map = LaunchConfiguration('map')
map_path = os.path.join(s_base_directory_path, 'maps', 'home_map1.yaml')
