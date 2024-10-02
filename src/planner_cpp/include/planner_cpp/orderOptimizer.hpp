#ifndef UTILS_ORDER_OPTIMIZER
#define UTILS_ORDER_OPTIMIZER

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "planner_cpp/utils.hpp"
#include "planner_cpp/utils_parsing.hpp"
#include "planner_cpp/utils_shortest_path.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kbot_interfaces/msg/next_order.hpp"  

using namespace std;

class OrderOptimizer : public rclcpp::Node
{
public:
  OrderOptimizer();

private:
  Order current_order_;
  string order_descr_ = "";
  Position current_pos_ = {0,0};
  bool position_valid_ = false;

  string path_order_data_ = "/home/rudolf/ros2/kbot/data";
  string rel_path_config_file_ = "/configuration/products.yaml";
  string file_path = path_order_data_ + rel_path_config_file_;

  unordered_map<string, Part> parts_;
  unordered_map<int, Product> products_;

  void current_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void next_order_callback(const kbot_interfaces::msg::NextOrder::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_current_position_;
  rclcpp::Subscription<kbot_interfaces::msg::NextOrder>::SharedPtr subscription_next_order_;

  std::mutex search_mutex;

  void get_order_(Position &, Order &);
  void parse_config_file_();
  bool find_order(int);

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // UTILS_ORDER_OPTIMIZER