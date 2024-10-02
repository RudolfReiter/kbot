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
#include <dirent.h>  // For opendir, readdir, closedir
#include <sys/stat.h>  // For struct stat, stat



struct Position {
    double x, y;
    double distance_to(const Position& ) const ;
};

struct Order {
    int order_nr;
    Position pos;
    std::set<int> product_nrs;
};


std::vector<std::string> get_all_files_in_directory(std::string& ) ;

void append_to_file(const std::string& , std::stringstream& );

#endif // UTILS