import rclpy
from rclpy.node import Node 
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")
        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        self.frequency = 1.0

        self.get_logger().info(f"Publishing at {self.frequency} HZ")
        self.timer_ = self.create_timer(1/self.frequency, self.timerCallback)

    def timerCallback(self):
        self.counter_ += 1
        msg = String()
        msg.data = f"timer: {self.counter_}"
        self.pub_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    