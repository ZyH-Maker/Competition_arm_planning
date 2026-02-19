#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from robot_interfaces.srv import GetCircles


class CirclesService(Node):
    def __init__(self):
        super().__init__("test_pose_service")

        self.srv = self.create_service(
            GetCircles,
            "Circles",
            self.handle_service,
        )

        self.get_logger().info("圆检测测试服务已启动")
        self.get_logger().info("服务名称: /Circles")

    def handle_service(self, request, response):
        if not request.trigger:
            response.circles_string = ""
            self.get_logger().warn("请求触发为假，返回空结果")
            return response

        response.circles_string = (
            "1:red,True,0.40,-0.15,-0.0565;"
            "2:green,True,0.30,0.0,-0.0565;"
            "3:blue,True,0.30,0.15,-0.0565"
        )

        self.get_logger().info(
            f"请求触发为真，返回: '{response.circles_string}'"
        )
        return response


def main():
    rclpy.init()
    node = CirclesService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
