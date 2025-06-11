import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__("simpe_service_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b

        self.future_ = self.client_.call_async(self.req)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info(f"Service response {future.result().sum}")




def main():

    a = 6 
    b = 10

    rclpy.init()
    node = SimpleServiceClient(a, b)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()