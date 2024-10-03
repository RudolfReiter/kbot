#include <gtest/gtest.h>
#include "planner_cpp/utils.hpp"
#include "planner_cpp/utils_shortest_path.hpp"
#include "planner_cpp/utils_parsing.hpp"
#include <thread>
#include <mutex>
#include <unordered_map>

#ifndef TEST_DATA_DIR
#error "TEST_DATA_DIR is not defined"
#endif

std::string get_test_file_path(const std::string &filename)
{
  return std::string(TEST_DATA_DIR) + "/" + filename;
}

TEST(planner_cpp, utils_distance_1)
{
  const Position p1 = {1.0, 0.0};
  const Position p2 = {0.0, 0.0};
  ASSERT_EQ(1.0, p1.distance_to(p2)) << "Distance between points is not correct";
}

TEST(planner_cpp, utils_distance_2)
{
  const Position p1 = {0.0, 0.0};
  const Position p2 = {0.0, -2.0};
  ASSERT_EQ(2.0, p1.distance_to(p2)) << "Distance between points is not correct";
}

TEST(planner_cpp, utils_distance_3)
{
  tuple_pos start{0.0, 0.0, "start"};
  tuple_pos goal{10.0, 0.0, "end"};
  ASSERT_EQ(10.0, distance_between(start, goal)) << "Distance between point is not correct";
}

TEST(planner_cpp, shortest_path_1)
{
  tuple_pos start{0.0, 0.0, "start"};
  tuple_pos goal{10.0, 10.0, "end"};

  std::vector<tuple_pos> intermediates = {
      {10.0, 0.0, "5"},
      {10.0, 5.0, "6"},
      {5.0, 0.0, "4"},
      {3.0, 0.0, "3"},
      {2.0, 0.0, "2"},
      {1.0, 0.0, "1"},
  };
  auto result = tsp_shortest_path(start, goal, intermediates);
  double shortest_path = result.first;
  ASSERT_EQ(20.0, shortest_path) << "Shortest path is not correct";
}

TEST(planner_cpp, shortest_path_reverse_negative)
{
  tuple_pos start{10.0, 10.0, "start"};
  tuple_pos goal{-10.0, 0.0, "end"};

  std::vector<tuple_pos> intermediates = {
      {10.0, 0.0, "5"},
      {10.0, 5.0, "6"},
      {5.0, 0.0, "4"},
      {3.0, 0.0, "3"},
      {2.0, 0.0, "2"},
      {1.0, 0.0, "1"},
  };
  auto result = tsp_shortest_path(start, goal, intermediates);
  double shortest_path = result.first;
  ASSERT_EQ(30.0, shortest_path) << "Shortest path is not correct";
}

TEST(planner_cpp, parsing_1)
{
  std::string input = " a b c ";
  ASSERT_EQ("abc", trim(input)) << "Could not trim string";
}

TEST(planner_cpp, parsing_2)
{
  std::string input = " a b \"123\" c ";
  ASSERT_EQ("123", getStringBetweenQuotes(input)) << "Could not extract string between quotes";
}

TEST(planner_cpp, product_struct)
{
  Product product;
  product.id = 1;
  product.product_name = "product1";
  product.addPart("part1");
  product.addPart("part1");
  product.addPart("part2");
  product.addPart("part1");
  ASSERT_EQ(3, product.part_count["part1"]) << "Part count for Product object is not correct";
}

TEST(FileTest, open_test_file)
{
  std::string file_path = get_test_file_path("orders/orders_20201201.yaml");
  std::ifstream test_file(file_path);
  ASSERT_TRUE(test_file.is_open()) << "Could not open test file: " << file_path;
}

TEST(FileTest, parse_orders)
{
  std::string file_path = get_test_file_path("orders/orders_20201201.yaml");
  std::vector<std::thread> threads;
  bool is_valid = false;
  int order_nr = 1;
  std::mutex search_mutex;
  Order order;

  threads.emplace_back(
      parse_orders,
      file_path,
      order_nr,
      ref(is_valid),
      ref(order),
      ref(search_mutex));

  for (auto &t : threads)
    if (t.joinable())
      t.join();

  ASSERT_EQ(1, *order.product_nrs.begin()) << "Order not correctly parsed";
}

TEST(planner_cpp, parse_products_1)
{
  unordered_map<string, Part> parts_;
  unordered_map<int, Product> products_;
  std::string file_path = get_test_file_path("configuration/products.yaml");

  parse_products(file_path, products_, parts_);

  ASSERT_EQ("Product 1", products_[1].product_name) << "Wrong product name in parsed product";
}

TEST(planner_cpp, parse_products_2)
{
  unordered_map<string, Part> parts_;
  unordered_map<int, Product> products_;
  std::string file_path = get_test_file_path("configuration/products.yaml");

  parse_products(file_path, products_, parts_);

  ASSERT_EQ(0.0, parts_["Part A"].pos.x) << "Wrong x position in parsed part";
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}