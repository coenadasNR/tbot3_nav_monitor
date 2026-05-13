"""Launch the monitor nodes (metrics_collector, adaptive_behavior, data_logger,
web_dashboard, waypoint_patrol). Does NOT start Gazebo/Nav2 — run a simulation
launch file first, then this."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'tbot3_nav_monitor'

    return LaunchDescription([
        DeclareLaunchArgument('world',                  default_value='unknown'),
        DeclareLaunchArgument('config',                  default_value='default'),
        DeclareLaunchArgument('adaptive',                default_value='true',
                              description='Set to false to disable adaptive_behavior node'),
        DeclareLaunchArgument('dashboard_port',          default_value='8080'),
        DeclareLaunchArgument('data_dir',                default_value='/ros2_ws/data/csv'),
        DeclareLaunchArgument('use_sim_time',            default_value='true'),
        DeclareLaunchArgument('nav2_ready_timeout_sec',  default_value='30.0'),

        Node(
            package=pkg,
            executable='metrics_collector',
            name='metrics_collector',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': LaunchConfiguration('world'),
                'publish_rate_hz': 2.0,
                'battery_drain_per_meter': 0.05,
                'nav2_ready_timeout_sec': LaunchConfiguration('nav2_ready_timeout_sec'),
            }],
            output='screen',
        ),

        Node(
            package=pkg,
            executable='adaptive_behavior',
            name='adaptive_behavior',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'recovery_threshold': 3,
                'accuracy_threshold_m': 0.40,
                'efficiency_threshold': 0.40,
                'check_period_sec': 5.0,
                'nav2_ready_timeout_sec': LaunchConfiguration('nav2_ready_timeout_sec'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('adaptive')),
        ),

        Node(
            package=pkg,
            executable='data_logger',
            name='data_logger',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'data_dir': LaunchConfiguration('data_dir'),
                'config': LaunchConfiguration('config'),
                'world': LaunchConfiguration('world'),
                'flush_every_n': 10,
            }],
            output='screen',
        ),

        Node(
            package=pkg,
            executable='web_dashboard',
            name='web_dashboard',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port': LaunchConfiguration('dashboard_port'),
                'world': LaunchConfiguration('world'),
                'data_dir': LaunchConfiguration('data_dir'),
            }],
            output='screen',
        ),

        Node(
            package=pkg,
            executable='waypoint_patrol',
            name='waypoint_patrol',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': LaunchConfiguration('world'),
                'loop_delay_sec': 2.0,
                'max_retries': 2,
                'retry_delay_sec': 3.0,
            }],
            output='screen',
        ),
    ])
