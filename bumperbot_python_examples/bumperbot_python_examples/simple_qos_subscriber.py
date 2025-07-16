import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

class SimpleQoSSubscriber(Node):
    def __init__(self):
        super().__init__("simple_qos_subscriber")

        self.qos_profile_ = QoSProfile(depth=10)

        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")

        reliability = self.get_parameter("reliability").get_parameter_value().string_value
        durability = self.get_parameter("durability").get_parameter_value().string_value

        if reliability == "best_effort":
            self.qos_profile_.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("[Reliability] : Best Effort")
        elif reliability == "reliable":
            self.qos_profile_.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("[Reliability] : Reliable")
        elif reliability == "system_default":
            self.qos_profile_.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Reliability] : System Default")
        else:
            self.get_logger().error(f"Selected Reliability QoS: {reliability} doesn't exists")
            return

        if durability == "transient_local":
            self.qos_profile_.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("[Durability] : Transient Local")
        elif durability == "volatile":
            self.qos_profile_.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info("[Durability] : Volatile")
        elif durability == "system_default":
            self.qos_profile_.reliability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Durability] : System Default")
        else:
            self.get_logger().error(f"Selected Reliability QoS: {durability} doesn't exists")
            return


        self.sub_ = self.create_subscription(String, "chatter", self.subscriberCallback, self.qos_profile_)

    def subscriberCallback(self, msg):
        self.get_logger().info(f"counter {msg.data}")


def main():
    rclpy.init()
    node = SimpleQoSSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()