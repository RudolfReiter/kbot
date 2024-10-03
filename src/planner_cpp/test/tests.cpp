#include <gtest/gtest.h>
#include "planner_cpp/utils.hpp"
#include "planner_cpp/utils_shortest_path.hpp"
#include "planner_cpp/utils_parsing.hpp"


TEST(planner_cpp, utils_distance_1)
{
    const Position p1 = {1.0, 0.0};
    const Position p2 = {0.0, 0.0};
    ASSERT_EQ(1.0, p1.distance_to(p2));
}

TEST(planner_cpp, utils_distance_2)
{
    const Position p1 = {0.0, 0.0};
    const Position p2 = {0.0, -2.0};
    ASSERT_EQ(2.0, p1.distance_to(p2));
}

TEST(planner_cpp, utils_distance_3)
{
    tuple_pos start{0.0, 0.0, "start"}; 
    tuple_pos goal{10.0, 0.0, "end"};
    ASSERT_EQ(10.0, distance_between(start, goal));
}

TEST(planner_cpp, shortest_path_1)
{
    tuple_pos start{0.0, 0.0, "start"}; 
    tuple_pos goal{10.0, 10.0, "end"}; 

    std::vector<tuple_pos> intermediates1 = {
        {10.0, 0.0, "5"},
        {10.0, 5.0, "6"},
        {5.0, 0.0, "4"},
        {3.0, 0.0, "3"},
        {2.0, 0.0, "2"},
        {1.0, 0.0, "1"},
    };
    auto result = tsp_shortest_path(start, goal, intermediates1);
    double shortest_path = result.first;
    ASSERT_EQ(20.0, shortest_path);
}

TEST(planner_cpp, shortest_path_reverse_negative)
{
    tuple_pos start{10.0, 10.0, "start"}; 
    tuple_pos goal{-10.0, 0.0, "end"}; 

    std::vector<tuple_pos> intermediates1 = {
        {10.0, 0.0, "5"},
        {10.0, 5.0, "6"},
        {5.0, 0.0, "4"},
        {3.0, 0.0, "3"},
        {2.0, 0.0, "2"},
        {1.0, 0.0, "1"},
    };
    auto result = tsp_shortest_path(start, goal, intermediates1);
    double shortest_path = result.first;
    ASSERT_EQ(30.0, shortest_path);
}

TEST(planner_cpp, parsing_1)
{
    std::string input = " a b c ";
    ASSERT_EQ("abc", trim(input));
}

TEST(planner_cpp, parsing_1)
{
    std::string input = " a b \"123\" c ";
    ASSERT_EQ("123", getStringBetweenQuotes(input));
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}