"""Launch Gazebo Classic + TurtleBot3 Burger + Nav2 in the narrow passages world.
Uses official turtlebot3_gazebo resources as per assignment requirements.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, SetEnvironmentVariable,
                             TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share         = get_package_share_directory('tbot3_nav_monitor')
    nav2_bringup      = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_file  = os.path.join(pkg_share, 'worlds', 'narrow_passages.world')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_narrow.yaml')
    map_file    = '/ros2_ws/maps/narrow_map.yaml'

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode',         default_value='slam'),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                               os.path.join(turtlebot3_gazebo, 'models') + ':' +
                               '/usr/share/gazebo-11/models'),

        # 1. Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ),
        ),

        # 2. Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # 3. Spawn robot in C1 (bottom-left corridor)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': '-1.5', 'y_pose': '-4.0'}.items(),
        ),

        # 4. Nav2 + RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'nav_monitor.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'map': '', 'use_sim_time': 'true',
                              'params_file': nav2_params, 'slam': 'True'}.items(),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'map': map_file, 'use_sim_time': 'true',
                              'params_file': nav2_params, 'slam': 'False'}.items(),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'amcl'"])),
        ),

        # 5. Dynamic obstacles — delayed so Gazebo finishes spawning models first
        TimerAction(period=5.0, actions=[
            Node(
                package='tbot3_nav_monitor',
                executable='dynamic_obstacles',
                name='dynamic_obstacles',
                parameters=[{'speed': 0.2, 'half_period_sec': 5.0}],
                output='screen',
            ),
        ]),
    ])
