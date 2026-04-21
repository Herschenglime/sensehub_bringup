import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import UnlessCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _maybe_remove_existing_bag(context):
    no_bag = _parse_bool(LaunchConfiguration('no_bag').perform(context))
    overwrite_bag = _parse_bool(LaunchConfiguration('overwrite_bag').perform(context))
    bag_name = LaunchConfiguration('bag_name').perform(context)
    bag_path = os.path.join(os.path.expanduser('~/bags'), bag_name)

    if no_bag or not overwrite_bag or not os.path.exists(bag_path):
        return []

    if os.path.isdir(bag_path):
        shutil.rmtree(bag_path)
    else:
        os.remove(bag_path)

    return [LogInfo(msg=f'Removed existing bag path: {bag_path}')]


def generate_launch_description():
    # 1. Ensure the target bags directory exists so rosbag doesn't fail
    bags_dir = os.path.expanduser('~/bags')
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
        description='Name of the rosbag folder to save in ~/bags'
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

    # 3. Process for Unitree Lidar
    lidar_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source /home/team/dev/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash && ros2 launch unitree_lidar_ros2 launch.py'
        ],
        output='screen'
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
    pre_bag_cleanup = OpaqueFunction(function=_maybe_remove_existing_bag)

    bag_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', 
            [f'{bags_dir}/', LaunchConfiguration('bag_name')],
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
        lidar_process,
        camera_node,
        static_tf_node,
        pre_bag_cleanup,
        bag_process,
        bag_exit_handler
    ])
