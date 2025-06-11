import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
import math


class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_transform_stamped = TransformStamped()
        self.dynamic_transform_stamped = TransformStamped()

        self.tf_buffer_ = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer_, self)

        self.x_increment_ = 0.01
        self.last_x_= 0.0
        self.last_orientation_ = quaternion_from_euler(0.0, 0.0, 0.0)
        self.orientation_increment_ = quaternion_from_euler(0.0, 0.0, 0.05)
        self.orientation_count_ = 0

        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped.header.frame_id = "base"
        self.static_transform_stamped.child_frame_id = "top"

        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 1.0
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(self.static_transform_stamped)

        self.get_logger().info(f"Publishing static transform between {self.static_transform_stamped.header.frame_id}"\
                               f"and {self.static_transform_stamped.child_frame_id}")
        
        self.timer_ = self.create_timer(0.1, self.timerCallback)

        self.server_ = self.create_service(GetTransform, "get_transform", self.transformCallback)


    def timerCallback(self):    
        self.dynamic_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped.header.frame_id = "odom"
        self.dynamic_transform_stamped.child_frame_id = "base"
        
        self.dynamic_transform_stamped.transform.translation.x = self.x_increment_ + self.last_x_
        self.dynamic_transform_stamped.transform.translation.y = 0.0
        self.dynamic_transform_stamped.transform.translation.z = 0.0

        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)

        self.dynamic_transform_stamped.transform.rotation.x = q[0]
        self.dynamic_transform_stamped.transform.rotation.y = q[1]
        self.dynamic_transform_stamped.transform.rotation.z = q[2]
        self.dynamic_transform_stamped.transform.rotation.w = q[3]

        
        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_transform_stamped)

        self.last_x_ = self.dynamic_transform_stamped.transform.translation.x
        self.last_orientation_ = q
        self.orientation_count_ += 1

        if self.orientation_count_ >= 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.orientation_count_ = 0

    def transformCallback(self, req, res):
        self.get_logger().info(f"Requested transform between {req.frame_id} and {req.child_frame_id}")
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e :
            res.success = False
            return res
        
        res.success = True
        res.transform = requested_transform
        return res


def main():
    rclpy.init()
    node = SimpleTFKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()