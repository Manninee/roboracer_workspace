from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    vesc_config = os.path.join(current_dir, 'configs', 'vesc.yaml')
    mux_config = os.path.join(current_dir, 'configs', 'mux.yaml')
    depthimage_conversion_config = os.path.join(current_dir, 'configs', 'depthimage_to_laserscan.yaml')
    realsense2_camera_config = os.path.join(current_dir, 'configs', 'realsense2_camera.yaml')
    twist_to_ackermann_config = os.path.join(current_dir, 'configs', 'twist_to_ackermann_config.yaml')

    pkg_share = FindPackageShare(package='model').find('model')
    urdf_model_path = os.path.join(pkg_share, 'urdf/model.urdf')
    with open(urdf_model_path, 'r', encoding="utf-8") as infp:
        robot_desc = infp.read()

    ld = LaunchDescription([])

    nodes = [
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[vesc_config]
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[vesc_config]
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[vesc_config]
        ),
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[mux_config],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')]
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[realsense2_camera_config]
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth','/depth/image_rect_raw'),
                        ('depth_camera_info', '/depth/camera_info')],
            parameters=[depthimage_conversion_config]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "1.57", "-1.57", "0", "depth_camera_link", "camera_link"]
        ),
        Node(
            package='twist_to_ackermann',
            executable='twist_to_ackermann_node',
            parameters=[twist_to_ackermann_config]
        )
    ]

    for node in nodes:
        ld.add_action(node)

    return ld
