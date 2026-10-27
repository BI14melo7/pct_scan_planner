"""Complete Go2 + Gazebo Fortress + SCAN-Planner navigation demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, "launch", "go2_sim.launch.py")
        ),
        launch_arguments={
            "world": "navigation.sdf",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
        }.items(),
    )

    # Publishes the exact point-cloud counterpart of navigation.sdf.
    navigation_map = Node(
        package="go2_description",
        executable="nav_map_publisher.py",
        name="go2_navigation_map",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "run.launch.py")
        ),
        launch_arguments={
            "is_real_world": "false",
            "external_simulator": "true",
            "external_map": "true",
            "navi_mode": "1",
            "sensor_type": "lidar",
            "controller_mode": "closed_loop",
            "use_gpu": "false",
            "map_size_x": "24.0",
            "map_size_y": "20.0",
            "map_size_z": "4.0",
            "use_sim_time": "true",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("x", default_value="-8.0"),
            DeclareLaunchArgument("y", default_value="-6.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
            gazebo,
            navigation_map,
            planner,
            rviz,
        ]
    )
