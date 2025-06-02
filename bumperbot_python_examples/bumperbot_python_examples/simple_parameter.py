import rclpy 
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.get_logger().info("simple_parameter node started...")
        self.declare_parameter("simple_int_param", 0)
        self.declare_parameter("simple_string_param", "moose")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.Type == Parameter.Type.INTEGER:
                self.get_logger().info(f"param simple_int_param changed new value {param.value}")
                result.successful = True
            
            if param.name == "simple_string_param" and param.Type == Parameter.Type.STRING:
                self.get_logger().info(f"param simple_string_param changed new value {param.value}")
                result.successful = True
        return result
    

def main():
    rclpy.init()
    node = SimpleParameter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
