#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        self.controller_sub_ = self.create_subscription(
            Twist, 
            "/bumperbot_controller/cmd_vel_unstamped", 
            self.controllerTwistCallback,
            10
        )

        self.controller_pub_ = self.create_publisher(
            TwistStamped, 
            "/bumperbot_controller/cmd_vel", 
            10
        )


        self.joy_sub_ = self.create_subscription(
            TwistStamped, 
            "/input_joy/cmd_vel_stamped", 
            self.joyTwistCallback,
            10
        )

        self.joy_pub_ = self.create_publisher(
            Twist, 
            "input_joy/cmd_vel", 
            10
        )



    def controllerTwistCallback(self, msg:Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.controller_pub_.publish(twist_stamped)

    def joyTwistCallback(self, msg:TwistStamped):
        twist = Twist()
        twist = msg.twist
        self.joy_pub_.publish(twist)


def main():
    rclpy.init()
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
