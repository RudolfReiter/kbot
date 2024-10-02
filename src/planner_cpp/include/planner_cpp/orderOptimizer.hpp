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

#include "rclcpp/rclcpp.hpp"
#include "planner_cpp/utils.hpp"
#include "planner_cpp/utils_parsing.hpp"
#include "planner_cpp/utils_shortest_path.hpp"

using namespace std;

class OrderOptimizer : public rclcpp::Node
{
public:
  OrderOptimizer();

private:
  Order current_order_;
  string order_descr_ = "";

  string path_order_data_ = "/home/rudolf/ros2/knappbot_ws/data";
  string rel_path_config_file_ = "/configuration/products.yaml";
  string file_path = path_order_data_ + rel_path_config_file_;

  unordered_map<string, Part> parts_;
  unordered_map<int, Product> products_;

  std::mutex search_mutex;

  void get_order_(Position &, Order &);
  void parse_config_file_();
  bool find_order(int);

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // UTILS_ORDER_OPTIMIZER