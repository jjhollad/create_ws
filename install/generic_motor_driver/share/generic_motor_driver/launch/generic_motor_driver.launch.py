#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyUSB0',
        description='Serial device path'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Serial baud rate'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.3',
        description='Distance between left and right wheels (meters)'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
        description='Wheel radius (meters)'
    )
    
    motor_gear_ratio_arg = DeclareLaunchArgument(
        'motor_gear_ratio',
        default_value='1.0',
        description='Motor internal gear ratio (e.g., 50.0 for 1:50 reduction)'
    )
    
    belt_drive_ratio_arg = DeclareLaunchArgument(
        'belt_drive_ratio',
        default_value='1.0',
        description='Belt drive ratio (e.g., 2.0 for 1:2 reduction)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    loop_hz_arg = DeclareLaunchArgument(
        'loop_hz',
        default_value='10.0',
        description='Control loop frequency (Hz)'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    latch_cmd_duration_arg = DeclareLaunchArgument(
        'latch_cmd_duration',
        default_value='0.2',
        description='Duration to latch velocity commands (seconds)'
    )

    # Generic motor driver node
    generic_motor_driver_node = Node(
        package='generic_motor_driver',
        executable='generic_motor_driver',
        name='generic_motor_driver',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'baud': LaunchConfiguration('baud'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'motor_gear_ratio': LaunchConfiguration('motor_gear_ratio'),
            'belt_drive_ratio': LaunchConfiguration('belt_drive_ratio'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'loop_hz': LaunchConfiguration('loop_hz'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'latch_cmd_duration': LaunchConfiguration('latch_cmd_duration'),
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
            ('joint_states', 'joint_states'),
        ]
    )

    return LaunchDescription([
        dev_arg,
        baud_arg,
        wheel_base_arg,
        wheel_radius_arg,
        motor_gear_ratio_arg,
        belt_drive_ratio_arg,
        base_frame_arg,
        odom_frame_arg,
        loop_hz_arg,
        publish_tf_arg,
        latch_cmd_duration_arg,
        generic_motor_driver_node,
    ])

