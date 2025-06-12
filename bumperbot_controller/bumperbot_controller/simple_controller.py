#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import numpy as np 
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_contoller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.previous_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
    

        self.get_logger().info(f"using wheel radius: {self.wheel_radius}")
        self.get_logger().info(f"using wheel separation: {self.wheel_separation}")

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, 
                                                    "simple_velocity_controller/commands", 
                                                    10)
        self.vel_sub_ = self.create_subscription(TwistStamped, 
                                                 "bumperbot_controller/cmd_vel", 
                                                 self.velCallback, 
                                                 10)
        
        self.pos_sub_ = self.create_subscription(JointState, 
                                                 "joint_states", 
                                                 self.positionCallback,
                                                 10)
        
        self.speed_conversion_ = np.array([
            [self.wheel_radius/2, self.wheel_radius/2],
            [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
            ])
        
        self.get_logger().info(f"the conversion matrix is {self.speed_conversion_}")
        
    def velCallback(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def positionCallback(self, msg:JointState):
        dpos_left = msg.position[1] - self.left_wheel_prev_pos
        dpos_right = msg.position[0] - self.right_wheel_prev_pos

        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]

        dt = Time.from_msg(msg.header.stamp) - self.previous_time_
        self.previous_time_ = Time.from_msg(msg.header.stamp)

        pos_dot_left = dpos_left / (dt.nanoseconds / S_TO_NS)
        pos_dot_right = dpos_right / (dt.nanoseconds / S_TO_NS)

        linear_vel = (self.wheel_radius * pos_dot_right + self.wheel_radius * pos_dot_left) / 2
        angular_vel = (self.wheel_radius * pos_dot_right - self.wheel_radius * pos_dot_left) / self.wheel_separation

        position = (self.wheel_radius * dpos_right + self.wheel_radius * dpos_left) / 2
        orientation = (self.wheel_radius * dpos_right - self.wheel_radius * dpos_left) / self.wheel_separation

        self.theta_ += orientation
        self.x_ += position * math.cos(self.theta_)
        self.y_ += position * math.sin(self.theta_)


        self.get_logger().info(f"linear vel:{linear_vel}, angular vel:{angular_vel}")
        self.get_logger().info(f"x:{self.x_}, y:{self.y_}, theta:{self.theta_}")




def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
