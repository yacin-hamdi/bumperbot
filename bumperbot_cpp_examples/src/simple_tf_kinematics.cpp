#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;

class SimpleTFKinematics: public rclcpp::Node
{
public:
    SimpleTFKinematics(): Node("simple_tf_kinematics")
    {
        static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        dynamic_transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped static_transform_stamped;


        static_transform_stamped.header.stamp = this->get_clock()->now();
        static_transform_stamped.header.frame_id = "base";
        static_transform_stamped.child_frame_id = "top";
        static_transform_stamped.transform.translation.x = 0.0;
        static_transform_stamped.transform.translation.y = 0.0;
        static_transform_stamped.transform.translation.z = 1.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, M_PI/2);
        static_transform_stamped.transform.rotation.x = q.x();
        static_transform_stamped.transform.rotation.y = q.y();
        static_transform_stamped.transform.rotation.z = q.z();
        static_transform_stamped.transform.rotation.w = q.w();
        
        static_transform_broadcaster->sendTransform(static_transform_stamped);

        timer_ = create_wall_timer(0.1s, std::bind(&SimpleTFKinematics::timerCallback, this));


        
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_transform_broadcaster;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

    float last_x_ = 0.0;
    float x_increment_ = 0.01;

    void timerCallback()
    {
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "odom";
        dynamic_transform_stamped.child_frame_id = "base";
        dynamic_transform_stamped.transform.translation.x = last_x_ + x_increment_;
        dynamic_transform_stamped.transform.translation.y = 0.0;
        dynamic_transform_stamped.transform.translation.z = 0.0;

        dynamic_transform_stamped.transform.rotation.x = 0.0;
        dynamic_transform_stamped.transform.rotation.y = 0.0;
        dynamic_transform_stamped.transform.rotation.z = 0.0;
        dynamic_transform_stamped.transform.rotation.w = 1.0;

        last_x_ = dynamic_transform_stamped.transform.translation.x;
        dynamic_transform_broadcaster->sendTransform(dynamic_transform_stamped);
        
        


    }
};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTFKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}