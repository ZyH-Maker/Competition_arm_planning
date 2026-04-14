from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot_description", package_name="robot_config")
        .to_moveit_configs()
    )

    robot_config_share = get_package_share_directory("robot_config")
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_config_share, "launch", "demo.launch.py")
        )
    )

    record_node = Node(
        package="main_bt",
        executable="record_target_transform",
        name="record_target_transform_runner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"enable_visualization": True},
            {"interpolation_segments": 10},
            {"target_x": -0.17},
            {"target_y": -0.114},
            {"target_z": 0.1375},
            {"target_roll": 0.0},
            {"target_pitch": 0.0},
            {"target_yaw": 0.0},
            {"output_yaml": "/home/fins/robot/src/Config/target_transform.yaml"},
        ],
    )

    return LaunchDescription([
        demo_launch,
        record_node,
    ])
