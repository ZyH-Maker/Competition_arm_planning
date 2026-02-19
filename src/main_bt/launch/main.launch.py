from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 说明：构建 MoveIt 配置（包含机器人描述、语义、运动学等）
    moveit_config = MoveItConfigsBuilder("robot_description", package_name="robot_config").to_moveit_configs()

    # 说明：启动 main_bt 的 BT Runner（行为树主节点）
    bt_runner = Node(
        name="bt_runner",
        package="main_bt",
        executable="bt_runner",
        output="screen",
        parameters=[
            # 说明：MoveIt 需要的核心参数
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # 说明：BT/MoveIt 参数由节点内部默认值提供，避免重复声明
        ],
    )

    return LaunchDescription([bt_runner])
