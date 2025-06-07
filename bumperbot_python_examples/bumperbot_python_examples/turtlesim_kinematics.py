import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math 

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")
        self.turtle1_pos_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.turtle1PoseCallback, 10)
        self.turtle2_pos_sub_ = self.create_subscription(Pose, "/turtle2/pose", self.turtle2PoseCallback, 10)
        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg



    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg
        
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_degree = 180 * theta_rad / 3.14

        self.get_logger().info(f"\n" \
        "Translation vector turtle1 -> turtle2 \n" \
        f"Tx: {Tx}\n" \
        f"Ty: {Ty}\n" \
        f"Rotation Matrix turtle2 -> turtle1 \n"\
        f"theta_rad: {theta_rad}\n" \
        f"theta_degree: {theta_degree}\n"\
        f"[R11   R12]: [{math.cos(theta_rad)}   {-math.sin(theta_rad)}]\n"\
        f"[R21   R22]: [{math.sin(theta_rad)}   {math.cos(theta_rad)}]\n")


def main():
    rclpy.init()
    node = SimpleTurtlesimKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()