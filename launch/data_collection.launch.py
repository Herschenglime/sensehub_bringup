import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Ensure the target bags directory exists so rosbag doesn't fail
    bags_dir = os.path.expanduser('~/bags')
    os.makedirs(bags_dir, exist_ok=True)

    # 2. Declare the configurable parameter for the bag name
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='sync_session',
        description='Name of the rosbag folder to save in ~/bags'
    )

    # 3. Process for Unitree Lidar
    lidar_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source /home/team/dev/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash && ros2 launch unitree_lidar_ros2 launch.py'
        ],
        output='screen'
    )

    # 4. Process for Camera Trigger Sync
    sync_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source /home/team/workspaces/timing_ws/install/setup.bash && ros2 run ov9782_trig_sync trig_sync'
        ],
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
            '--y', '0.0889', 
            '--z', '-0.11938', 
            '--roll', '0', 
            '--pitch', '0', 
            '--yaw', '0',
            '--frame-id', 'unilidar_lidar', 
            '--child-frame-id', 'camera_frame'
        ],
        output='screen'
    )
    # 6. Process for Rosbag Recording (Specific Topics)
    bag_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', 
            [f'{bags_dir}/', LaunchConfiguration('bag_name')],
            '/unilidar/cloud',
            '/unilidar/imu',
            '/tf',
            '/trig/image_raw',
            '/trig/camera_info',
            '/tf_static'
        ],
        output='screen'
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
    )

    return LaunchDescription([
        bag_name_arg,
        lidar_process,
        sync_process,
        static_tf_node,
        bag_process,
        bag_exit_handler
    ])
