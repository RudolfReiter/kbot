#include "planner_cpp/orderOptimizer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kbot_interfaces/msg/next_order.hpp"  

using namespace std;
using tuple_pos = std::tuple<double, double, string>;

OrderOptimizer::OrderOptimizer() : Node("orderOptimizer")
{
  RCLCPP_INFO(this->get_logger(), "Node %s started...", this->get_name());

  RCLCPP_INFO(this->get_logger(), "Parsing config file...");
  parse_config_file_();
  RCLCPP_INFO(this->get_logger(), "Finished parsing config file with %d products and %d parts ...", products_.size(), parts_.size());

  Position current_pos = {1000.0, 1000.0};
  RCLCPP_INFO(this->get_logger(), "Current position x: %f, y: %f", current_pos.x, current_pos.y);

  int order_nr = 1400004;
  RCLCPP_INFO(this->get_logger(), "Looking for order %d ...", order_nr);

  bool is_valid = find_order(order_nr);

  if (is_valid)
  {
    RCLCPP_INFO(this->get_logger(), "Order %d found. Processing...", current_order_.order_nr);
    get_order_(current_pos, current_order_);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Order %d not found...", order_nr);
  }
  RCLCPP_INFO(this->get_logger(), "Finished...");
}

void OrderOptimizer::get_order_(Position &current_pos, Order &order)
{
  unordered_map<string, vector<string>> partid2products;
  set<string> all_parts;

  tuple_pos start{current_pos.x, current_pos.y, "current position"};
  tuple_pos goal{order.pos.x, order.pos.y, "goal position"};

  for (auto &product_nr : order.product_nrs)
  {
    for (auto &part : products_[product_nr].part_count)
    {
      all_parts.insert(get<0>(part));
      partid2products[get<0>(part)].push_back(products_[product_nr].product_name);
    }
  }
  vector<string> all_parts_vec(all_parts.begin(), all_parts.end());

  vector<tuple_pos> intermediates;
  for (auto &part : all_parts_vec)
  {
    intermediates.push_back({parts_[part].pos.x, parts_[part].pos.y, part});
  }

  auto result = tsp_shortest_path(start, goal, intermediates);

  // double shortest_path = result.first;
  std::vector<string> path = result.second;

  RCLCPP_INFO(this->get_logger(), "Working on order %d (%s)", current_order_.order_nr, order_descr_.c_str());

  for (auto &part : path)
  {
    for (auto &product : partid2products[part])
    {
      RCLCPP_INFO(this->get_logger(), "Fetching part \'%s\' for product \'%s\' at x: %f, y: %f",
                  part.c_str(), product.c_str(), parts_[part].pos.x, parts_[part].pos.y);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Delivering to destination x: %f, y: %f", order.pos.x, order.pos.y);
}

void OrderOptimizer::parse_config_file_()
{
  try
  {
    parse_products(file_path, products_, parts_);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

bool OrderOptimizer::find_order(int order_nr)
{
  std::string root_order_path = path_order_data_ + "/orders";
  std::vector<std::string> file_names = get_all_files_in_directory(root_order_path);

  if (file_names.empty())
  {
    RCLCPP_INFO(this->get_logger(), "No order files found in path %s", root_order_path.c_str());
    return false;
  }

  // Vector to store threads
  std::vector<std::thread> threads;
  bool is_valid = false;

  // Start a thread for each file
  for (const auto &file_name : file_names)
    threads.emplace_back(
        parse_orders,
        root_order_path + "/" + file_name,
        order_nr,
        ref(is_valid),
        ref(current_order_),
        ref(search_mutex));

  // Join all threads (wait for all threads to finish)
  for (auto &t : threads)
    if (t.joinable())
      t.join();
    
  
  if (is_valid)
  {
    cout << "Order found" << endl;
    cout << "Order number: " << current_order_.order_nr << endl;
    cout << "Order position: " << current_order_.pos.x << ", " << current_order_.pos.y << endl;
    cout << "Product numbers: " << current_order_.product_nrs.size() << endl;
  }
  return is_valid;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = make_shared<OrderOptimizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}