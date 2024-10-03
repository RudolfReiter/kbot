#ifndef UTILS
#define UTILS

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <set>
#include <cmath>
#include <dirent.h>   // For opendir, readdir, closedir
#include <sys/stat.h> // For struct stat, stat
#include "visualization_msgs/msg/marker_array.hpp"

struct Position
{
    double x, y;
    double distance_to(const Position &) const;
};

struct Order
{
    int order_nr;
    Position pos;
    std::set<int> product_nrs;
};

std::vector<std::string> get_all_files_in_directory(std::string &);

void append_to_file(const std::string &, std::stringstream &);

visualization_msgs::msg::Marker get_marker(double, double, int, int, std::string);
visualization_msgs::msg::Marker get_strip_marker(double, double, double, double, int);

#endif // UTILS

