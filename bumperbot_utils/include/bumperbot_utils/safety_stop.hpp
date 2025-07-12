#ifndef SAFETY_STOP
#define SAFETY_STOP

#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/bool.hpp>
#include<sensor_msgs/msg/laser_scan.hpp>

enum State
{
    FREE = 0,
    WARNING = 1,
    DANGER = 2
};


class SafetyStop: public rclcpp::Node
{
public:
    SafetyStop();


private:
    void laserCallback(const sensor_msgs::msg::LaserScan &msg);
    State state;
    double danger_distance_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub;
};




#endif //SAFETY_STOP