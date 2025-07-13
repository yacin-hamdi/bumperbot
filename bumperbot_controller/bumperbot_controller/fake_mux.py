#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


class FakeMux(Node):
    def __init__(self):
        super().__init__("fake_mux")
        self.joy_sub_ = self.create_subscription(TwistStamped, 
                                                 "/input_joy/cmd_vel", 
                                                 self.joyCallback, 
                                                 10)
        
        self.safety_sub_ = self.create_subscription(Bool, 
                                                    "safety_stop", 
                                                    self.safetyCallback, 
                                                    10)
        
        self.vel_pub_ = self.create_publisher(TwistStamped, 
                                              "bumperbot_controller/cmd_vel", 
                                              10)
        
        self.twistStamped = TwistStamped()
        
    
    def joyCallback(self, msg:TwistStamped):
        self.twistStamped = msg

    def safetyCallback(self, msg: Bool):
        if msg.data:
            self.twistStamped.twist.linear.x = 0.0
            self.twistStamped.twist.angular.z = 0.0

        self.vel_pub_.publish(self.twistStamped)





def main():
    rclpy.init()
    node = FakeMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()