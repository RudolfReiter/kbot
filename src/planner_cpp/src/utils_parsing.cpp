#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <set>
#include <mutex>
#include <thread>
#include "planner_cpp/utils_parsing.hpp"

using namespace std;

void Product::addPart(const string &part_name)
{
    if (part_count.find(part_name) != part_count.end())
        part_count[part_name]++;

    else
        part_count[part_name] = 1;
}

void Product::clear()
{
    id = 0;
    product_name.clear();
    part_count.clear();
}

// Utility function to trim leading and trailing spaces and hyphens
string trim(const string &input)
{
    string result;
    bool open_quote = false;
    for (char ch : input)
    {
        if ((ch != ' ' && ch != '-') || open_quote == true)
        {
            result += ch;
        }
        if (ch == '"')
        {
            open_quote = !open_quote;
        }
    }
    return result;
}

string getStringBetweenQuotes(const string &input)
{
    size_t first_quote = input.find('\"');
    if (first_quote == string::npos)
    {
        return ""; // No opening quotation mark found.
    }

    size_t second_quote = input.find('\"', first_quote + 1);
    if (second_quote == string::npos)
    {
        return ""; // No closing quotation mark found.
    }

    return input.substr(first_quote + 1, second_quote - first_quote - 1);
}

// Function to parse the YAML-like configuration file
void parse_products(const string &file_path,
                    unordered_map<int, Product> &products,
                    unordered_map<string, Part> &parts)
{
    ifstream file(file_path);
    if (!file.is_open())
    {
        throw runtime_error("Could not open file: " + file_path);
    }

    string line;
    string part_name;
    Product current_product;
    Part current_part;
    bool in_parts_list = false;
    bool in_specific_part = false;

    while (getline(file, line))
    {
        line = trim(line);

        // Ignore empty lines
        if (line.empty())
        {
            continue;
        }

        // Check for new product (id)
        if (line.find("id:") == 0)
        {
            // Start a new product
            // If we are already parsing a product, push it to the map
            if (current_product.id != 0)
            {
                products[current_product.id] = current_product;
                current_product.clear();
            }

            // Start a new product
            current_product.id = stoi(line.substr(line.find(":") + 1));
            in_parts_list = false; // Reset the parts list flag
        }
        // Parse product line
        else if (line.find("product:") == 0)
        {
            current_product.product_name = getStringBetweenQuotes(line);
        }
        // Start parsing parts list
        else if (line.find("parts:") == 0)
        {
            in_parts_list = true;
        }
        // Parse part details within the parts list
        else if (in_parts_list && line.find("part:") == 0)
        {
            part_name = getStringBetweenQuotes(line);
            if (!part_name.empty())
            {
                // Add the previously parsed part to the current product's parts map
                current_product.addPart(part_name);
            }
            in_specific_part = true;
            current_part.part_name = part_name;
        }
        else if (in_parts_list && line.find("cx:") == 0 && in_specific_part)
        {
            current_part.pos.x = stod(line.substr(line.find(":") + 1));
        }
        else if (in_parts_list && line.find("cy:") == 0 && in_specific_part)
        {
            current_part.pos.y = stod(line.substr(line.find(":") + 1));
            parts[part_name] = current_part;
            in_specific_part = false;
        }
    }

    if (current_product.id != 0)
    {
        products[current_product.id] = current_product;
    }

    file.close();
}

bool is_number(const std::string &s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it))
        ++it;
    return !s.empty() && it == s.end();
}

// Function to parse the YAML-like configuration file

void parse_orders(const string &file_path,
                  int order_nr,
                  bool &is_valid,
                  Order &order,
                  mutex &search_mutex)
{
    ifstream file(file_path);
    if (!file.is_open())
        throw runtime_error("Could not open file: " + file_path);

    string line;
    int current_order_id;
    bool found_order = false;
    bool in_prod_list = false;
    bool is_valid_entry = false;
    Order order_tmp;

    while (getline(file, line))
    {
        line = trim(line);

        // Ignore empty lines
        if (line.empty())
            continue;

        // Check for new product (id)
        if (line.find("order:") == 0)
        {
            if (found_order)
                break;
            // Start a new product
            current_order_id = stoi(line.substr(line.find(":") + 1));
            order_tmp.order_nr = current_order_id;
            found_order = (current_order_id == order_nr);
        }
        else if (found_order)
        {
            if (line.find("products:") == 0)
                in_prod_list = true;

            else if (line.find("cx:") == 0)
                order_tmp.pos.x = stod(line.substr(line.find(":") + 1));

            else if (line.find("cy:") == 0)
                order_tmp.pos.y = stod(line.substr(line.find(":") + 1));

            else if (in_prod_list && is_number(line))
            {
                order_tmp.product_nrs.insert(stoi(line));
                is_valid_entry = true;
            }
        }
    }

    if (is_valid_entry)
    {
        search_mutex.lock();
        order = order_tmp;
        is_valid = true;
        search_mutex.unlock();
    }
    file.close();
}

/*
int main(int argc, char *argv[])
{
    string order_descr_ = "Example description 1";
    string path_order_data_ = "/home/rudolf/ros2/knappbot_ws/data";
    string rel_path_config_file_ = "/configuration/products.yaml";
    string file_path = path_order_data_ + rel_path_config_file_;

    unordered_map<string, Part> parts;
    unordered_map<int, Product> products;

    parse_products(file_path, products, parts);

    return 0;
}*/
