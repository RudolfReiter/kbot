#include "rclcpp/rclcpp.hpp"
#include "kbot_interfaces/msg/next_order.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class NextOrderPublisher : public rclcpp::Node
{
public:
    //std::vector<int> order_nrs = {1100002, 1100003, 1100004, 1100005, 1100006, 1100007, 1100008, 1100009, 1100010};
    std::vector<int> order_nrs = {1000001};
    long unsigned count = 0;
    NextOrderPublisher() : Node("next_order_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Node %s started...", this->get_name());
        // Create a publisher for the nextOrder topic
        publisher_ = this->create_publisher<kbot_interfaces::msg::NextOrder>("nextOrder", 10);
        // Publish every 2 seconds
        timer_ = this->create_wall_timer(2000ms, std::bind(&NextOrderPublisher::publish_order, this));
    }

private:
    void publish_order()
    {
        auto message = kbot_interfaces::msg::NextOrder();

        // Fill in the custom message with some sample data
        if (count < order_nrs.size())
        {
            message.order_id = order_nrs[count];
            message.description = "Obtain order " + std::to_string(order_nrs[count]);
            // Publish the message
            RCLCPP_INFO(this->get_logger(), "Publishing next order: ID=%u, Description=\"%s\"",
                        message.order_id, message.description.c_str());
            publisher_->publish(message);
            count++;
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Published all messages, shutting down...");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<kbot_interfaces::msg::NextOrder>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NextOrderPublisher>());
    rclcpp::shutdown();
    return 0;
}
