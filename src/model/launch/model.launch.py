import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
  # Set the path to this package.

  pkg_share = os.path.join(os.path.dirname(__file__), "..")
  # pkg_share = FindPackageShare(package='model').find('model')
 
  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')
 
  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/model_test.urdf')

  with open(default_urdf_model_path, 'r', encoding="utf-8") as infp:
    robot_desc = infp.read()
 
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  urdf_model = LaunchConfiguration('urdf_model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
 
#   # Declare the launch arguments  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
     
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
   
   
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': True, 
    'robot_description': robot_desc}])
 
  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
   
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
 
#   # Add any actions
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
 
  return ld