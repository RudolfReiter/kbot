#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CurrentPositionPublisher : public rclcpp::Node
{
public:
    CurrentPositionPublisher() : Node("current_position_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&CurrentPositionPublisher::publish_position, this));
    }

private:
    void publish_position()
    {
        auto message = geometry_msgs::msg::PoseStamped();

        message.header.stamp = this->get_clock()->now();
        message.pose.position.x = 0.0;
        message.pose.position.y = 0.0;
        message.pose.position.z = 0.0;
        message.pose.orientation.w = 1.0;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Publishing current position: [x=%.2f, y=%.2f, z=%.2f]", 
                    message.pose.position.x, message.pose.position.y, message.pose.position.z);
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentPositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
