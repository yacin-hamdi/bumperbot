#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimpleQoSPublisher: public rclcpp::Node
{
    public:
        SimpleQoSPublisher(): Node("simple_publisher"), qos_profile_pub_(10), counter_(0)
        {
            declare_parameter<std::string>("reliability", "system_default");
            declare_parameter<std::string>("durability", "system_default");

            const auto reliability = get_parameter("reliability").as_string();
            const auto durability = get_parameter("durability").as_string();

            if(reliability == "best_effort"){
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                RCLCPP_INFO(get_logger(), "[Reliability]: Best Effort");
            }else if(reliability == "reliable"){
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                RCLCPP_INFO(get_logger(), "[Reliability]: Reliable");
            }else if(reliability == "system_default"){
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Reliability]: System Default");
            }else{
                RCLCPP_ERROR(get_logger(), "Selected Reliability QoS doesn't exists");
            }

            if(durability == "transient_local"){
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
                RCLCPP_INFO(get_logger(), "[Reliability]: transient local");
            }else if(durability == "volatile"){
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                RCLCPP_INFO(get_logger(), "[Reliability]: Volatile");
            }else if(durability == "system_default"){
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Reliability]: System Default");
            }else{
                RCLCPP_ERROR(get_logger(), "Selected Reliability QoS doesn't exists");
            }


            pub_ = create_publisher<std_msgs::msg::String>("chatter", qos_profile_pub_);
            timer_ = create_wall_timer(1s, std::bind(&SimpleQoSPublisher::TimerCallback, this));

            RCLCPP_INFO(get_logger(), "publishing at 1 HZ");
        }

    private:
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::QoS qos_profile_pub_;

        void TimerCallback()
        {
            std_msgs::msg::String msg = std_msgs::msg::String();
            msg.data = std::to_string(counter_);
            pub_->publish(msg);
            counter_ += 1;
        }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleQoSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}