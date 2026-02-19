import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QRCodePublisher(Node):
    def __init__(self) -> None:
        super().__init__("qrcode_publisher")
        self.declare_parameter("topic", "qrcode")
        self.declare_parameter("period", 1.0)
        self.declare_parameter("task_order", "3,2,1")

        topic = self.get_parameter("topic").value
        period = self.get_parameter("period").value
        self.task_order = self.get_parameter("task_order").value

        self.pub = self.create_publisher(String, topic, 10)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self) -> None:
        msg = String()
        msg.data = str(self.task_order)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = QRCodePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
