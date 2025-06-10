import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts



class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self_server_ = self.create_service(AddTwoInts, "add_two_ints", self.AddCallback)

        self.get_logger().info(f"add two ints server has been started!")

    def AddCallback(self, req, res):
        self.get_logger().info(f"request to add a:{req.a} + b:{req.b}")
        res.sum = req.a + req.b
        self.get_logger().info(f"response is {res.sum}")
        
        return res
    

def main():
    rclpy.init()
    node = SimpleServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()