#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import numpy as np 
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

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

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
    

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
        
        self.odom_pub_ = self.create_publisher(Odometry, 
                                               "bumperbot_controller/odom", 
                                               10)
        
        self.speed_conversion_ = np.array([
            [self.wheel_radius/2, self.wheel_radius/2],
            [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
            ])

        self.tf_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        
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
        dt = Time.from_msg(msg.header.stamp) - self.previous_time_

        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
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


        q = quaternion_from_euler(0, 0, self.theta_)

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        self.odom_msg_.twist.twist.linear.x = linear_vel
        self.odom_msg_.twist.twist.angular.z = angular_vel

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.odom_pub_.publish(self.odom_msg_)
        
        
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.tf_.sendTransform(self.transform_stamped_)

        
        # self.get_logger().info(f"linear vel:{linear_vel}, angular vel:{angular_vel}")
        # self.get_logger().info(f"x:{self.x_}, y:{self.y_}, theta:{self.theta_}")




def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
