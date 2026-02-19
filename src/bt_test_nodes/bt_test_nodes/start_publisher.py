import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StartPublisher(Node):
    def __init__(self) -> None:
        super().__init__("start_publisher")
        self.declare_parameter("topic", "start")
        self.declare_parameter("period", 1.0)
        self.declare_parameter("value", True)

        topic = self.get_parameter("topic").value
        period = self.get_parameter("period").value
        self.value = self.get_parameter("value").value

        self.pub = self.create_publisher(Bool, topic, 10)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self) -> None:
        msg = Bool()
        msg.data = bool(self.value)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StartPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
