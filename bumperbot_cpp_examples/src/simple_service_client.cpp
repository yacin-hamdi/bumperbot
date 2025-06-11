#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient: public rclcpp::Node
{
    public:
    SimpleServiceClient(int a, int b): Node("simple_service_client")
    {
        client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");
        while(!client_->wait_for_service(1s))
        {
            RCLCPP_INFO_STREAM(get_logger(), "Service not available, waiting again...");

        }

        
        req = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
        req->a = a;
        req->b = b;

        auto future = client_->async_send_request(req, std::bind(&SimpleServiceClient::requestCallback, this, _1));

    }

    private:
        rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;
        bumperbot_msgs::srv::AddTwoInts::Request::SharedPtr req;
        
        void requestCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future)
        {
            if(future.valid())
            {
                RCLCPP_INFO_STREAM(get_logger(), "Service response " << future.get()->sum);
            }else{
                RCLCPP_ERROR(get_logger(), "Service Failure");
            }

        }
};


int main(int argc, char* argv[])
{
    int a = 8, b = 5;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceClient>(a, b);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}