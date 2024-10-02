#ifndef UTILS_SHORTEST_PATH_H
#define UTILS_SHORTEST_PATH_H

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <set>
#include <cmath>
#include "planner_cpp/utils.hpp"


using tuple_pos = std::tuple<double, double, std::string>; 

double distance_between(const tuple_pos& , const tuple_pos& );
std::pair<double, std::vector<std::string>> tsp_shortest_path(const tuple_pos& , const tuple_pos& , const std::vector<tuple_pos>& );


#endif // UTILS_SHORTEST_PATH_H