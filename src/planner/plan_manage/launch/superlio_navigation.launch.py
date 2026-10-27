"""Connect the local Super-LIO ROS 2 outputs to SCAN-Planner.

Input expected by Super-LIO (Livox MID-360 preset):
  /livox/lidar  livox_ros_driver2/msg/CustomMsg
  /livox/imu    sensor_msgs/msg/Imu

The planner consumes Super-LIO's robot odometry and world-frame scan directly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    superlio_share = get_package_share_directory("super_lio")

    superlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(superlio_share, "launch", "Livox_mid360.py")
        ),
        launch_arguments={"rviz": "false"}.items(),
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "run.launch.py")
        ),
        launch_arguments={
            "is_real_world": "true",
            "external_simulator": "true",
            "navi_mode": LaunchConfiguration("navi_mode"),
            "sensor_type": "lidar",
            "controller_mode": "closed_loop",
            "use_sim_time": "false",
            "body_pose_topic": "/lio/robo/odom",
            "sensor_pose_topic": "/lio/odom",
            "cloud_topic": "/lio/cloud_world",
            "cmd_vel_topic": "/cmd_vel",
            "cloud_is_world": "true",
            "need_extrinsic": "false",
            "initial_path_topic": LaunchConfiguration("initial_path_topic"),
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_share, "launch", "rviz.launch.py")
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("initial_path_topic", default_value="/initial_path"),
            superlio,
            planner,
            rviz,
        ]
    )
