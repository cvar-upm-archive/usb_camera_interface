from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml


def staticTransformNode(context, *args, **kwargs):
    tf_cam_config = LaunchConfiguration('tf_cam_config').perform(context)

    with open(tf_cam_config, 'r') as config_file:
        tf_arguments = yaml.load(config_file, Loader=yaml.FullLoader)

    ns = LaunchConfiguration('drone_id').perform(context)

    base_link = ns + '/' + tf_arguments['frame_id']
    camera_link = ns + '/' + tf_arguments['child_frame_id']
    x = tf_arguments['x']
    y = tf_arguments['y']
    z = tf_arguments['z']
    roll = tf_arguments['roll']
    pitch = tf_arguments['pitch']
    yaw = tf_arguments['yaw']

    static_transform_publisher_node = Node(
        # Tf from baselink to RGB cam
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf',
        namespace=LaunchConfiguration('drone_id'),
        arguments=[f'{x:.2f}', f'{y:.2f}', f'{z:.2f}', f'{roll:.2f}', f'{pitch:.2f}', f'{yaw:.2f}',
                   f'{base_link}', f'{camera_link}'],
        output='screen',
        emulate_tty=True,
    )

    return [static_transform_publisher_node]


def generate_launch_description():
    camera_info = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/usb_camera_interface', 'new_camera_info.yaml'
    ])
    camera_params = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/usb_camera_interface', 'params.yaml'
    ])
    tf_cam_config = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/tf_cam', 'real_arguments.yaml'
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
            parameters=[LaunchConfiguration('camera_params'), LaunchConfiguration('camera_info')],
            output='screen',
            emulate_tty=True,
        ),

        OpaqueFunction(function=staticTransformNode)
    ])
