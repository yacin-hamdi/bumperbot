#include "bumperbot_utils/safety_stop.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <math.h>


using std::placeholders::_1;

SafetyStop::SafetyStop():
    Node("safety_stop")

{
    declare_parameter<double>("danger_distance", 0.2);
    declare_parameter<std::string>("scan_topic", "scan");
    declare_parameter<std::string>("safety_stop_topic", "safety_stop");

    danger_distance_ = get_parameter("danger_distance").as_double();
    std::string scan_topic = get_parameter("scan_topic").as_string();
    std::string safety_stop_topic = get_parameter("safety_stop_topic").as_string();

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 
        10, 
        std::bind(&SafetyStop::laserCallback, this, _1)
    );

    

    safety_stop_pub = create_publisher<std_msgs::msg::Bool>(
        safety_stop_topic, 
        10
    );

    state = State::FREE;
}

void SafetyStop::laserCallback( const sensor_msgs::msg::LaserScan & msg)
{
    state = State::FREE;

    for(const auto & range: msg.ranges){
        if(!std::isinf(range) && range <= danger_distance_){
            state = State::DANGER;
            break;
        }
    }

    std_msgs::msg::Bool is_safety_stop;
    if(state == State::DANGER){
        is_safety_stop.data = true;
    }else if (state == State::FREE){
        is_safety_stop.data = false;
    }

    safety_stop_pub->publish(is_safety_stop);

}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}