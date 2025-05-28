#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher: public rclcpp::Node
{
    public:
        SimplePublisher(): Node("simple_publisher"), counter_(0)
        {
            pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::publisherCallback, this));
            RCLCPP_INFO(get_logger(), "publishing at 1 HZ");
            
        }

    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int counter_;

    void publisherCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::to_string(counter_);
        counter_ ++;
        pub_->publish(msg);
    }





};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
