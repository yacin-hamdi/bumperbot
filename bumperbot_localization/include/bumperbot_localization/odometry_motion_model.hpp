#ifndef ODOMETRY_MOTION_MODEL
#define ODOMETRY_MOTION_MODEL

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>


class OdometryMotionModel: public rclcpp::Node
{
    public:
        OdometryMotionModel(const std::string& name);
       

    private:
        void odomCallback(const nav_msgs::msg::Odometry &);
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
        geometry_msgs::msg::PoseArray samples;

        bool is_first_odom_;
        double last_odom_x_;
        double last_odom_y_;
        double last_odom_theta_;
        double alpha1, alpha2, alpha3, alpha4;
        int nb_samples;
};


#endif // ODOMETRY_MOTION_MODEL