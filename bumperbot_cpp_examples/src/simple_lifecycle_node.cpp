#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using namespace std::chrono_literals;


class SimpleLifecycleNode: public rclcpp_lifecycle::LifecycleNode
{    
public:
    SimpleLifecycleNode(const std::string & node_name, bool intra_process_comms = false)
    : LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(get_logger(), "Lifecycle Node on_configure called");
        sub_ = create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&SimpleLifecycleNode::msgCallback, this, std::placeholders::_1)
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Lifecycle Node on_shutdown called");
        sub_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Lifecycle Node on_cleanup called");
        sub_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);
        RCLCPP_INFO(get_logger(), "Lifecycle Node on_activate called");
        std::this_thread::sleep_for(2s);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "Lifecycle Node on_deactivate called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    

    void msgCallback(const std_msgs::msg::String &msg)
    {
        auto state = get_current_state();
        if(state.label() == "active")
        {
            RCLCPP_INFO_STREAM(get_logger(), "Lifecycle node heard: "<< msg.data);
        }
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<SimpleLifecycleNode> node = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node");
    ste.add_node(node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();

    return 0;
}