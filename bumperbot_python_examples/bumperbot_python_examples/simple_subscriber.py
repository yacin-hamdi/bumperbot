import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")
        self.sub_ = self.create_subscription(String, "chatter", self.subscriberCallback, 10)

    def subscriberCallback(self, msg):
        self.get_logger().info(f"counter {msg.data}")


def main():
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()