"""Deterministic planar obstacle-avoidance test for SCAN-Planner.

The map contains tall 3-D boxes, but the robot motion is constrained to the
XY plane.  This isolates perception/planning from Gazebo contacts and from a
quadruped locomotion controller, making local obstacle avoidance repeatable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")

    navigation_map = Node(
        package="go2_description",
        executable="nav_map_publisher.py",
        name="scan_2d_navigation_map",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "run.launch.py")
        ),
        launch_arguments={
            "is_real_world": "false",
            "external_simulator": "false",
            "external_map": "true",
            "navi_mode": "1",
            "sensor_type": "lidar",
            "controller_mode": "closed_loop",
            "use_gpu": "false",
            "map_size_x": "24.0",
            "map_size_y": "20.0",
            "map_size_z": "4.0",
            "init_x": LaunchConfiguration("x"),
            "init_y": LaunchConfiguration("y"),
            "init_z": LaunchConfiguration("z"),
            "use_sim_time": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "rviz.launch.py")
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("x", default_value="-8.0"),
            DeclareLaunchArgument("y", default_value="-6.0"),
            DeclareLaunchArgument("z", default_value="0.3"),
            navigation_map,
            planner,
            rviz,
        ]
    )
