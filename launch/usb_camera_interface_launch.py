from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    camera_info = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/usb_camera_interface', 'wide03_1080_info.yaml'
    ])
    camera_params = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/usb_camera_interface', 'params.yaml'
    ])
    tf_cam_config = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/tf_cam', 'f330_arguments.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('camera_info', default_value=camera_info),
        DeclareLaunchArgument('camera_params', default_value=camera_params),
        DeclareLaunchArgument('tf_cam_config', default_value=tf_cam_config),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='usb_camera_interface',
            executable='usb_camera_interface_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[LaunchConfiguration('camera_params'),
                        LaunchConfiguration('camera_info'),
                        LaunchConfiguration('tf_cam_config')],
            output='screen',
            emulate_tty=True,
        ),

        # OpaqueFunction(function=staticTransformNode)
    ])
