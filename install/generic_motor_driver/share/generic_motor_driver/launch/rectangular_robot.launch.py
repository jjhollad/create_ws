#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyUSB0',
        description='Serial device path for motor controller'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Serial baud rate'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open('/home/user/test_ws/src/create_robot/create_driver/urdf/rectangular_robot.urdf').read()
        }]
    )

    # Joint state publisher (for manual testing)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Generic motor driver with rectangular robot parameters
    motor_driver_node = Node(
        package='generic_motor_driver',
        executable='generic_motor_driver',
        name='generic_motor_driver',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'baud': LaunchConfiguration('baud'),
            'wheel_base': 0.57,  # Wheel base for rectangular robot
            'wheel_radius': 0.12,  # Rear wheel radius
            'motor_gear_ratio': 210.0,  # Your motor gear ratio
            'belt_drive_ratio': 6.4,  # Your belt drive ratio
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Static transform from base_link to base_footprint
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/user/test_ws/src/create_robot/create_driver/rviz/rectangular_robot.rviz']
    )

    return LaunchDescription([
        # Launch arguments
        dev_arg,
        baud_arg,
        use_sim_time_arg,
        rviz_arg,
        
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        motor_driver_node,
        static_transform_node,
        rviz_node,
    ])
