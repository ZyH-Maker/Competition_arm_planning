from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # 说明：加载 MoveIt 配置（包含 kinematics.yaml）
    moveit_config = (
        MoveItConfigsBuilder("robot_description", package_name="robot_config")
        .to_moveit_configs()
    )

    # 说明：启动测试节点，并注入 MoveIt 相关参数
    test_node = Node(
        package="main_bt",
        executable="test_trajectory",
        name="test_trajectory_runner",
        output="screen",
        parameters=[
            # 说明：MoveIt 需要的核心参数
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        test_node,
    ])
