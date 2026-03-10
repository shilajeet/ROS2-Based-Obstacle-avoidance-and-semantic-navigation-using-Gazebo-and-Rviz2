"""
semantic_nav_demo.launch.py
──────────────────────────
Single-command launch that brings up:
  1. TurtleBot3 in Gazebo (turtlebot3_world)
  2. Nav2 full stack (AMCL + planner + controller + BT)
  3. RViz2 with semantic overlay config
  4. semantic_navigator node
  5. obstacle_monitor node

Usage:
  ros2 launch semantic_nav semantic_nav_demo.launch.py
  ros2 launch semantic_nav semantic_nav_demo.launch.py loop_mission:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # ── Paths ──────────────────────────────────────────────────────────────
    pkg_semantic   = get_package_share_directory('semantic_nav')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2       = get_package_share_directory('nav2_bringup')

    # ── Args ───────────────────────────────────────────────────────────────
    use_sim_time_arg   = DeclareLaunchArgument('use_sim_time',   default_value='true')
    loop_mission_arg   = DeclareLaunchArgument('loop_mission',   default_value='false')
    robot_model_arg    = DeclareLaunchArgument('robot_model',    default_value='burger')
    map_yaml_arg       = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_semantic, 'maps', 'turtlebot3_world.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    loop_mission = LaunchConfiguration('loop_mission')
    robot_model  = LaunchConfiguration('robot_model')
    map_yaml     = LaunchConfiguration('map')

    # ── Environment ────────────────────────────────────────────────────────
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', robot_model)

    # ── Gazebo + TurtleBot3 ────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Nav2 ───────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map':          map_yaml,
            'params_file':  os.path.join(pkg_semantic, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # ── RViz2 ──────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_semantic, 'rviz', 'semantic_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ── Semantic Navigator (delayed to let Nav2 start) ─────────────────────
    semantic_nav_node = Node(
        package='semantic_nav',
        executable='semantic_navigator',
        name='semantic_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'loop_mission': loop_mission,
            'goal_tolerance_m': 0.25,
        }]
    )

    # ── Obstacle Monitor ───────────────────────────────────────────────────
    obstacle_monitor_node = Node(
        package='semantic_nav',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'warning_distance_m':  0.5,
            'critical_distance_m': 0.25,
            'forward_cone_deg':    60.0,
            'blocked_fraction':    0.6,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        loop_mission_arg,
        robot_model_arg,
        map_yaml_arg,
        set_tb3_model,
        gazebo_launch,
        nav2_launch,
        rviz_node,
        # Delay our nodes by 5 s to let Nav2 initialise
        TimerAction(period=5.0, actions=[semantic_nav_node]),
        TimerAction(period=3.0, actions=[obstacle_monitor_node]),
    ])
