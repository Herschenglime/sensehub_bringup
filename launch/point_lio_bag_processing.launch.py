import os
import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _find_workspace_root():
    candidates = [
        os.getcwd(),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')),
    ]

    for candidate in candidates:
        if os.path.isdir(os.path.join(candidate, 'bags')) or os.path.isdir(os.path.join(candidate, 'src')):
            return candidate

    return os.getcwd()


def _resolve_input_bag_path(context, bags_dir):
    input_bag = LaunchConfiguration('input_bag').perform(context)
    if os.path.isabs(input_bag):
        return input_bag
    return os.path.join(bags_dir, input_bag)


def _maybe_remove_existing_output_bag(context, bags_dir):
    overwrite_output_bag = _parse_bool(LaunchConfiguration('overwrite_output_bag').perform(context))
    output_bag_name = LaunchConfiguration('output_bag_name').perform(context)
    output_bag_path = os.path.join(bags_dir, output_bag_name)

    if not overwrite_output_bag or not os.path.exists(output_bag_path):
        return []

    if os.path.isdir(output_bag_path):
        shutil.rmtree(output_bag_path)
    else:
        os.remove(output_bag_path)

    return [LogInfo(msg=f'Removed existing output bag path: {output_bag_path}')]


def _validate_input_bag_path(context, bags_dir):
    input_bag_path = _resolve_input_bag_path(context, bags_dir)
    if os.path.exists(input_bag_path):
        return [
            SetLaunchConfiguration('resolved_input_bag', input_bag_path),
            LogInfo(msg=f'Using input bag: {input_bag_path}'),
        ]

    return [
        LogInfo(msg=f'Input bag does not exist: {input_bag_path}'),
        EmitEvent(event=Shutdown(reason='input bag path not found')),
    ]


def generate_launch_description():
    workspace_root = _find_workspace_root()
    bags_dir = os.path.join(workspace_root, 'bags')
    os.makedirs(bags_dir, exist_ok=True)

    input_bag_arg = DeclareLaunchArgument(
        'input_bag',
        description='Input rosbag path (absolute or relative to <workspace_root>/bags)'
    )

    output_bag_name_arg = DeclareLaunchArgument(
        'output_bag_name',
        default_value='point_lio_processed',
        description='Name of output rosbag folder in <workspace_root>/bags'
    )

    overwrite_output_bag_arg = DeclareLaunchArgument(
        'overwrite_output_bag',
        default_value='false',
        description='If true, remove an existing output bag path before recording'
    )

    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'launch',
                'mapping_unilidar_l2.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    body_to_aft_mapped_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_aft_mapped_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'aft_mapped',
            '--child-frame-id', 'body',
        ],
        output='screen',
    )

    body_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_camera_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0',
            '--y', '-0.0889',
            '--z', '0.11938',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'body',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    pre_output_cleanup = OpaqueFunction(
        function=_maybe_remove_existing_output_bag,
        kwargs={'bags_dir': bags_dir},
    )

    validate_input_bag = OpaqueFunction(
        function=_validate_input_bag_path,
        kwargs={'bags_dir': bags_dir},
    )

    output_bag_path = PathJoinSubstitution([bags_dir, LaunchConfiguration('output_bag_name')])

    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--use-sim-time',
            '-o', output_bag_path,
            '/image_raw',
            '/camera_info',
            '/cloud_registered_body',
            '/tf',
            '/tf_static',
        ],
        output='screen',
    )

    play_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('resolved_input_bag'),
            '--clock',
            '--topics',
            '/unilidar/cloud',
            '/unilidar/imu',
            '/image_raw',
            '/camera_info',
        ],
        output='screen',
    )

    start_play_on_record_start = RegisterEventHandler(
        OnProcessStart(
            target_action=record_process,
            on_start=[
                LogInfo(msg='Recorder started; launching rosbag play with --clock.'),
                play_process,
            ],
        )
    )

    shutdown_after_play = RegisterEventHandler(
        OnProcessExit(
            target_action=play_process,
            on_exit=[
                LogInfo(msg='ros2 bag play finished; shutting down in 1 second.'),
                TimerAction(
                    period=1.0,
                    actions=[EmitEvent(event=Shutdown(reason='rosbag play completed'))],
                ),
            ],
        )
    )

    return LaunchDescription([
        input_bag_arg,
        output_bag_name_arg,
        overwrite_output_bag_arg,
        point_lio_launch,
        body_to_aft_mapped_tf,
        body_to_camera_tf,
        pre_output_cleanup,
        validate_input_bag,
        record_process,
        start_play_on_record_start,
        shutdown_after_play,
    ])
