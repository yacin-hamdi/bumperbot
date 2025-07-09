#!/usr/bin/env python3

import rclpy
from rclpy import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, atan2, sqrt, fabs, pi
import numpy as np
import random
import time


def angle_diff(a, b):
    a = atan2(sin(a), cos(a))
    b = atan2(sin(b), cos(b))
    d1 = a - b 
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0

    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2



class OdometryMotionModel(Node):
    def __init__(self):
        super.__init__("odometry_motion_model")
        self.is_first_time_ = True
        self.last_odom_x_ = 0.0
        self.last_odom_y_ = 0.0
        self.last_odom_theta_ = 0.0

        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        self.declare_parameter("nb_samples", 300)

        self.alpha1 = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2 = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3 = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4 = self.get_parameter("alpha4").get_parameter_value().double_value

        self.nb_samples = self.get_parameter("nb_samples").get_parameter_value().integer_value

        

        if self.nb_samples >= 0:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nb_samples)]

        else:
            self.get_logger().fatal(f"invalid number of samples:{self.nb_samples}")
            return


        self.odom_sub_ = self.create_subscription(Odometry, 
                                                  "bumperbot_controller/odom", 
                                                  self.odomCallback,
                                                  10)
        
        self.pose_array_pub_ = self.create_publisher(PoseArray, 
                                                     "odometry_motion_model/samples", 
                                                     10)
        
    
    def odomCallback(self, odom:Odometry):
        
        roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                  odom.pose.pose.orientation.y,
                                                  odom.pose.pose.orientation.z,
                                                  odom.pose.pose.orientation.w])
        
        if self.is_first_time_:
            self.last_odom_x_ = odom.pose.pose.position.x
            self.last_odom_y_ = odom.pose.pose.position.y
            self.last_odom_theta_ = yaw
            self.samples.header.frame_id = odom.header.frame_id
            self.is_first_time_ = False
            return
        
        odom_x_increment = self.last_odom_x_ - odom.pose.pose.position.x
        odom_y_increment = self.last_odom_y_ - odom.pose.pose.position.y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta_)

        if sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2)) < 0.01:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), self.last_odom_theta_)

        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)
        delta_transl = sqrt(pow(odom_x_increment, 2) + pow(odom_y_increment, 2))

        rot1_variance = self.alpha1 * delta_rot1 + self.alpha2 * delta_transl
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_transl
        transl_variance = self.alpha3 * delta_transl + self.alpha4 * (delta_rot1 + delta_rot2)

        random.seed(int(time.time()))

        for sample in self.samples.poses:

            rot1_noise = random.gauss(0.0, rot1_variance)
            rot2_noise = random.gauss(0.0, rot2_variance)
            transl_noise = random.gauss(0.0, transl_variance)

            delta_rot1_noise = angle_diff(delta_rot1, rot1_noise)
            delta_rot2_noise = angle_diff(delta_rot2, rot2_noise)
            delta_transl_noise = delta_transl - transl_noise

            sample_roll, sample_pitch, sample_yaw = euler_from_quaternion([sample.orientation.x, 
                                                                            sample.orientation.y,
                                                                            sample.orientation.z, 
                                                                            sample.orientation.w])


            sample.position.x += delta_transl_noise * cos(sample_yaw + delta_rot1_noise)
            sample.position.y += delta_transl_noise * sin(sample_yaw + delta_rot1_noise)
            sample_q = quaternion_from_euler(0.0, 0.0, sample_yaw + delta_rot1_noise + delta_rot2_noise)
            sample.orientation.x = sample_q[0]
            sample.orientation.y = sample_q[1]
            sample.orientation.z = sample_q[2]
            sample.orientation.w = sample_q[3]

        self.last_odom_x_ = odom.pose.pose.position.x
        self.last_odom_y_ = odom.pose.pose.position.y
        self.last_odom_theta_ = yaw
        self.pose_array_pub_.publish(self.samples)




def main():
    rclpy.init()
    node = OdometryMotionModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()