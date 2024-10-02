#ifndef UTILS_PARSING_H
#define UTILS_PARSING_H

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <set>
#include <cmath>
#include <thread>
#include <mutex>

#include "planner_cpp/utils.hpp"

using namespace std;

struct Part
{
    string part_name;
    Position pos;
};

struct Product
{
    int id;
    string product_name;
    unordered_map<string, int> part_count;

    void addPart(const string &);
    void clear();
};

string trim(const string &);
string getStringBetweenQuotes(const string &);
void parse_products(const string &,
                    unordered_map<int, Product> &,
                    unordered_map<string, Part> &);

void parse_orders(const string &, int, bool &, Order &, mutex &);


#endif // UTILS_PARSING_H