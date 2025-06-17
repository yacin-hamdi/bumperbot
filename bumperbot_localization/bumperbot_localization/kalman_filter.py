#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        self.noisy_odom_sub_ = self.create_subscription(Odometry, 
                                                        "bumperbot_controller/odom_noisy", 
                                                        self.odomNoisyCallback, 
                                                        10)
        self.imu_sub_ = self.create_subscription(Imu, 
                                                 "imu/out", 
                                                 self.imuCallback, 
                                                 10)
        
        self.kalman_filter_pub_ = self.create_publisher(Odometry, 
                                                        "bumperbot_controller/odom_kalman", 
                                                        10)
        

        self.mean_ = 0.0
        self.variance_ = 1000.0


        self.imu_angular_z = 0.0
        self.is_first_odom = True
        self.last_angular_z = 0.0

        self.motion_ = 0.0

        self.odom_kalman_ = Odometry()

        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z) / (self.measurement_variance_ + self.variance_)
        self.variance_ = (self.measurement_variance_ * self.variance_) / (self.measurement_variance_ + self.variance_)

    def statePrediction(self):
        self.mean_ += self.motion_
        self.variance_ += self.motion_variance_
        


    def imuCallback(self, imu: Imu):
        self.imu_angular_z = imu.angular_velocity.z
        

    



    def odomNoisyCallback(self, odom: Odometry):
        self.odom_kalman_ = odom

        if self.is_first_odom:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z

            self.is_first_odom = False
            return 
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z
        
        self.statePrediction()
        self.measurementUpdate()

        self.last_angular_z = odom.twist.twist.angular.z

        self.odom_kalman_.twist.twist.angular.z = self.mean_
        self.kalman_filter_pub_.publish(self.odom_kalman_)
        

def main():
    rclpy.init()
    kalman_node = KalmanFilter()
    rclpy.spin(kalman_node)
    kalman_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    