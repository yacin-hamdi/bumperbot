#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import rclpy.time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException

class Pose:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x)/map_info.resolution)
    pose.y = round((py - map_info.origin.position.y)/map_info.resolution)
    

    return pose

def poseOnMap(robot_pose: Pose, map_info: MapMetaData):
    return robot_pose.x < map_info.width and robot_pose.x >= 0 and robot_pose.y < map_info.height and robot_pose.y >= 0

def poseToCell(pose:Pose, map_info: MapMetaData):
    return pose.y * map_info.width + pose.x


class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.resolution = resolution
        self.map_.info.origin.position.x = float(-round(width/2.0))
        self.map_.info.origin.position.y = float(-round(height/2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.map_pub_ = self.create_publisher(OccupancyGrid, 
                                             "map", 
                                             1)
        self.scan_sub_ = self.create_subscription(LaserScan, 
                                                  "scan", 
                                                  self.scanCallback, 
                                                  10)
        
        self.timer = self.create_timer(1.0, self.timerCallback)

        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)


    def scanCallback(self, scan: LaserScan):
        try:
            t = self.buffer.lookup_transform(self.map_.header.frame_id, scan.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().error("Unable to transform between /odom and /base_footprint")
            return
        
        robot_pose = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap(robot_pose, self.map_.info):
            self.get_logger().error("The robot is out of the map!")
            return 
        
        robot_cell = poseToCell(robot_pose, self.map_.info)
        self.map_.data[robot_cell] = 100

    def timerCallback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub_.publish(self.map_)

def main():
    rclpy.init()
    node = MappingWithKnownPoses("mapping_with_know_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        

        

