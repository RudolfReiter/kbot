#include "planner_cpp/orderOptimizer.hpp"

using namespace std;
using tuple_pos = std::tuple<double, double, string>;

OrderOptimizer::OrderOptimizer() : Node("orderOptimizer")
{
  this->declare_parameter<std::string>("data_path", "/home/rudolf/ros2/kbot/data");
  this->get_parameter("data_path", path_order_data_);

  RCLCPP_INFO(this->get_logger(), "Node %s started...", this->get_name());

  RCLCPP_INFO(this->get_logger(), "Parsing config file...");
  parse_config_file_();
  RCLCPP_INFO(this->get_logger(), "Parsed config file of %d products and %d parts ...",
              products_.size(), parts_.size());

  // Subscribe to currentPosition (PoseStamped message)
  subscription_current_position_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "currentPosition", 10, std::bind(&OrderOptimizer::current_position_callback, this, std::placeholders::_1));

  // Subscribe to nextOrder (NextOrder custom message)
  subscription_next_order_ = this->create_subscription<kbot_interfaces::msg::NextOrder>(
      "nextOrder", 10, std::bind(&OrderOptimizer::next_order_callback, this, std::placeholders::_1));

  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);

}

void OrderOptimizer::current_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received current position: x=%.2f, y=%.2f, z=%.2f",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  current_pos_.x = msg->pose.position.x;
  current_pos_.y = msg->pose.position.y;
  position_valid_ = true;
}

void OrderOptimizer::next_order_callback(const kbot_interfaces::msg::NextOrder::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received next order: ID=%u, Description=\"%s\"",
              msg->order_id, msg->description.c_str());
  int order_nr = msg->order_id;
  order_descr_ = msg->description;

  if (position_valid_)
  {
    bool is_valid = find_order(order_nr);
    if (is_valid)
    {
      RCLCPP_INFO(this->get_logger(), "Order %d found. Processing...", current_order_.order_nr);
      get_order_(current_pos_, current_order_);
      RCLCPP_INFO(this->get_logger(), "Order %d completed.", current_order_.order_nr);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Order %d not found...", order_nr);
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Current robot position not valid. Could not process order %d...", order_nr);
  }
}

void OrderOptimizer::get_order_(Position &current_pos, Order &order)
{
  unordered_map<string, vector<int>> partid2productids;
  set<string> all_parts;

  tuple_pos start{current_pos.x, current_pos.y, "current position"};
  tuple_pos goal{order.pos.x, order.pos.y, "goal position"};

  for (auto &product_nr : order.product_nrs)
  {
    for (auto &part : products_[product_nr].part_count)
    {
      all_parts.insert(get<0>(part));
      partid2productids[get<0>(part)].push_back(product_nr);
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
  std::stringstream ss;

  ss << "Working on order " << current_order_.order_nr << " (" << order_descr_ << ")" << endl;
  int n_products = 0;
  string plural_part_str = "part";

  for (auto &part : path)
  {
    for (auto &productid : partid2productids[part])
    {
      n_products = products_[productid].part_count[part];
      if (n_products > 1)
        plural_part_str = "parts";
      else
        plural_part_str = "part";

      ss << "Fetching " << n_products << " " << plural_part_str << " \'" << part
         << "\' for product \'" << products_[productid].product_name
         << "\' at x: " << parts_[part].pos.x << ", y: " << parts_[part].pos.y << endl;
    }
  }
  ss << "Delivering to destination x: " << order.pos.x << ", y: " << order.pos.y << endl;
  append_to_file(path_order_data_ + "/" + out_file, ss);

  publish_path();
}

void OrderOptimizer::parse_config_file_()
{
  try
  {
    parse_products(path_order_data_ + rel_path_config_file_, products_, parts_);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

bool OrderOptimizer::find_order(int order_nr)
{
  std::string root_order_path = path_order_data_ + rel_path_orders_;
  std::vector<std::string> file_names = get_all_files_in_directory(root_order_path);

  if (file_names.empty())
  {
    RCLCPP_INFO(this->get_logger(), "No order files found in path %s", root_order_path.c_str());
    return false;
  }

  std::vector<std::thread> threads;
  bool is_valid = false;

  for (const auto &file_name : file_names)
    threads.emplace_back(
        parse_orders,
        root_order_path + "/" + file_name,
        order_nr,
        ref(is_valid),
        ref(current_order_),
        ref(search_mutex));

  for (auto &t : threads)
    if (t.joinable())
      t.join();

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

void OrderOptimizer::publish_path(void)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker amr_marker;
        amr_marker.header.frame_id = "map";
        amr_marker.header.stamp = this->get_clock()->now();
        amr_marker.ns = "knappbot_position";
        amr_marker.id = 0;
        amr_marker.type = visualization_msgs::msg::Marker::CUBE;
        amr_marker.action = visualization_msgs::msg::Marker::ADD;
        amr_marker.pose.position.x = 0.0; 
        amr_marker.pose.position.y = 0.0; 
        amr_marker.pose.position.z = 0.5;
        amr_marker.pose.orientation.w = 1.0;
        amr_marker.scale.x = 0.5;
        amr_marker.scale.y = 0.5;
        amr_marker.scale.z = 0.5;
        amr_marker.color.r = 0.0f;
        amr_marker.color.g = 1.0f;
        amr_marker.color.b = 0.0f;
        amr_marker.color.a = 1.0f;
        marker_array.markers.push_back(amr_marker);

        // Example part pickup locations (x, y positions)
        std::vector<std::pair<float, float>> pickup_locations = {
            {2.0, 1.0},
            {3.0, 2.0},
            {4.0, 1.5}};

        // Add markers for each pickup location (CYLINDER type)
        int id = 1; // Start ID from 1 for pickup locations
        for (const auto &location : pickup_locations)
        {
            visualization_msgs::msg::Marker pickup_marker;
            pickup_marker.header.frame_id = "map";
            pickup_marker.header.stamp = this->get_clock()->now();
            pickup_marker.ns = "pickup_locations";
            pickup_marker.id = id++;
            pickup_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            pickup_marker.action = visualization_msgs::msg::Marker::ADD;
            pickup_marker.pose.position.x = location.first;
            pickup_marker.pose.position.y = location.second;
            pickup_marker.pose.position.z = 0.5;
            pickup_marker.pose.orientation.w = 1.0;
            pickup_marker.scale.x = 0.3;
            pickup_marker.scale.y = 0.3;
            pickup_marker.scale.z = 0.7;
            pickup_marker.color.r = 1.0f;
            pickup_marker.color.g = 0.0f;
            pickup_marker.color.b = 0.0f;
            pickup_marker.color.a = 1.0f;
            marker_array.markers.push_back(pickup_marker);
        }

        // Publish the MarkerArray
        marker_array_pub_->publish(marker_array);
    }