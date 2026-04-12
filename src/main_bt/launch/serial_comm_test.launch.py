from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bt_runner = Node(
        name="test_bt_runner",
        package="main_bt",
        executable="test_bt_runner",
        output="screen",
        parameters=[
            {"tree_relpath": "test/trees/main_test.xml"},
        ],
    )

    return LaunchDescription([bt_runner])
