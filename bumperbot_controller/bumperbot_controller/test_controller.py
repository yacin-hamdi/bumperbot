#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np 
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math



class TestController(Node):
    def __init__(self):
        super().__init__("test_controller")
        self.wheel_radius_ = 0.033
        self.wheel_seperation_ = 0.17
        self.conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
                                     [self.wheel_radius_/ self.wheel_seperation_, -self.wheel_radius_ / self.wheel_seperation_]])
        
        self.last_left_wheel_pos = 0.0
        self.last_right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"

        self.transform_ = TransformStamped()
        self.transform_.header.frame_id = "odom"
        self.transform_.child_frame_id = "base_footprint"

        self.tf_ = TransformBroadcaster(self)
        
        self.wheels_pub_ = self.create_publisher(Float64MultiArray, 
                                                 "simple_velocity_controller/commands", 
                                                 10)
        
        self.vel_sub_ = self.create_subscription(Twist, 
                                                 "bumperbot_controller/cmd_vel", 
                                                 self.velCallback, 
                                                 10)
        
        self.wheel_joint_sub_ = self.create_subscription(JointState, 
                                                         "joint_states", 
                                                         self.odomCallback,
                                                         10)
        
        self.odom_pub_ = self.create_publisher(Odometry, 
                                               "odom", 
                                               10)
        

    def velCallback(self, msg: Twist):
        vels = np.array([[msg.linear.x],
                         [msg.angular.z]])
        
        inv_conversion = np.linalg.inv(self.conversion_)
        wheels_vel = np.matmul(inv_conversion, vels)
        wheels_vel_msg = Float64MultiArray()
        wheels_vel_msg.data = [wheels_vel[0, 0], wheels_vel[1, 0]]
        self.wheels_pub_.publish(wheels_vel_msg)


    def odomCallback(self, msg: JointState):
        wheel_right_pos = msg.position[0]
        wheel_left_pos = msg.position[1]

        dpos_right = wheel_right_pos - self.last_right_wheel_pos
        dpos_left = wheel_left_pos - self.last_left_wheel_pos

        dt = Time.from_msg(msg.header.stamp) - self.last_time
        wheel_right_vel = dpos_right / (dt.nanoseconds / S_TO_NS)
        wheel_left_vel = dpos_left / (dt.nanoseconds / S_TO_NS)

        self.last_right_wheel_pos = msg.position[0]
        self.last_left_wheel_pos = msg.position[1]
        self.last_time = Time.from_msg(msg.header.stamp)

        linear_vel = (self.wheel_radius_ * wheel_left_vel + self.wheel_radius_ * wheel_right_vel) / 2
        angular_vel = (self.wheel_radius_ * wheel_left_vel - self.wheel_radius_ * wheel_right_vel) / self.wheel_seperation_

        position = linear_vel * dt.nanoseconds / S_TO_NS
        orientation = angular_vel * dt.nanoseconds / S_TO_NS

        
        self.theta_ += orientation
        self.x_ += position * math.cos(self.theta_)
        self.y_ += position * math.sin(self.theta_)

        q = quaternion_from_euler(0.0, 0.0, self.theta_)

        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.x_
        self.odom.pose.pose.position.y = self.y_
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]

        self.odom.twist.twist.linear.x = linear_vel
        self.odom.twist.twist.angular.z = angular_vel

        self.odom_pub_.publish(self.odom)

        self.transform_.transform.translation.x = self.x_
        self.transform_.transform.translation.y = self.y_
        self.transform_.transform.rotation.x = q[0]
        self.transform_.transform.rotation.y = q[1]
        self.transform_.transform.rotation.z = q[2]
        self.transform_.transform.rotation.w = q[3]
        self.transform_.header.stamp = self.get_clock().now().to_msg()

        self.tf_.sendTransform(self.transform_)








def main():
    rclpy.init()
    node = TestController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()