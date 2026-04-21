import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import UnlessCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _maybe_remove_existing_bag(context, bags_dir):
    no_bag = _parse_bool(LaunchConfiguration('no_bag').perform(context))
    overwrite_bag = _parse_bool(LaunchConfiguration('overwrite_bag').perform(context))
    bag_name = LaunchConfiguration('bag_name').perform(context)
    bag_path = os.path.join(bags_dir, bag_name)

    if no_bag or not overwrite_bag or not os.path.exists(bag_path):
        return []

    if os.path.isdir(bag_path):
        shutil.rmtree(bag_path)
    else:
        os.remove(bag_path)

    return [LogInfo(msg=f'Removed existing bag path: {bag_path}')]


def _find_workspace_root():
    # Prefer the current shell directory, since `ros2 launch` usually inherits it.
    # Fall back to the source-tree relative path when launched from an installed copy.
    candidates = [
        os.getcwd(),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')),
    ]

    for candidate in candidates:
        if os.path.isdir(os.path.join(candidate, 'bags')) or os.path.isdir(os.path.join(candidate, 'src')):
            return candidate

    return os.getcwd()


def generate_launch_description():
    # 1. Ensure the target bags directory exists so rosbag doesn't fail
    workspace_root = _find_workspace_root()
    bags_dir = os.path.join(workspace_root, 'bags')
    os.makedirs(bags_dir, exist_ok=True)
    usb_cam_params_path = os.path.join(
        get_package_share_directory('sensehub_bringup'),
        'config',
        'ov9782_params.yaml'
    )

    # 2. Declare the configurable parameter for the bag name
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='sync_session',
        description='Name of the rosbag folder to save in <workspace_root>/bags'
    )

    no_bag_arg = DeclareLaunchArgument(
        'no_bag',
        default_value='false',
        description='If true, skip rosbag recording'
    )

    overwrite_bag_arg = DeclareLaunchArgument(
        'overwrite_bag',
        default_value='false',
        description='If true, remove existing target bag path before recording'
    )

    # 3. Unitree lidar via the package's launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('unitree_lidar_ros2'),
                'launch',
                'l2.launch.py'
            ])
        )
    )

    # 4. Camera node via usb_cam
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[usb_cam_params_path],
        output='screen'
    )
    # 5. Static Transform Publisher (LiDAR to Camera)
    # Note: ROS requires meters, so 119.38mm -> 0.11938m, -88.9mm -> -0.0889m
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_tf',
        arguments=[
            '--x', '0', 
            '--y', '-0.0889', 
            '--z', '0.11938', 
            '--roll', '0', 
            '--pitch', '0', 
            '--yaw', '0',
            '--frame-id', 'unilidar_lidar', 
            '--child-frame-id', 'camera_link'
        ],
        output='screen'
    )
    # 6. Process for Rosbag Recording (Specific Topics)
    pre_bag_cleanup = OpaqueFunction(function=_maybe_remove_existing_bag, kwargs={'bags_dir': bags_dir})
    bag_output_path = PathJoinSubstitution([bags_dir, LaunchConfiguration('bag_name')])

    bag_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', 
            bag_output_path,
            '/unilidar/cloud',
            '/unilidar/imu',
            '/tf',
            '/image_raw',
            '/camera_info',
            '/tf_static'
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('no_bag'))
    )

    # 7. Shut down entire launch if rosbag exits (e.g., output folder already exists)
    bag_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_process,
            on_exit=[
                LogInfo(msg='ros2 bag record exited; shutting down launch.'),
                EmitEvent(event=Shutdown(reason='rosbag recording stopped or failed to start')),
            ]
        )
        ,
        condition=UnlessCondition(LaunchConfiguration('no_bag'))
    )

    return LaunchDescription([
        bag_name_arg,
        no_bag_arg,
        overwrite_bag_arg,
        lidar_launch,
        camera_node,
        static_tf_node,
        pre_bag_cleanup,
        bag_process,
        bag_exit_handler
    ])
