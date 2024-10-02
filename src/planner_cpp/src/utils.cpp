#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <set>
#include <fstream>

#include "planner_cpp/utils.hpp"

double Position::distance_to(const Position &other) const
{
    return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
}

// Function to get all file names within a folder
std::vector<std::string> get_all_files_in_directory(std::string &folder_path)
{
    std::vector<std::string> file_names;
    DIR *dir = opendir(folder_path.c_str()); // Open the directory

    // Check if directory was opened successfully
    if (dir == nullptr)
    {
        std::cerr << "Error: Could not open directory " << folder_path << std::endl;
        return file_names; // Return empty vector
    }

    struct dirent *entry;  // Structure to hold directory entry
    struct stat file_stat; // Structure to hold file information

    // Iterate over directory entries
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string file_name = entry->d_name;

        // Skip "." and ".." directories
        if (file_name == "." || file_name == "..")
        {
            continue;
        }

        std::string full_path = folder_path + "/" + file_name;

        // Get file status to check if it's a regular file
        if (stat(full_path.c_str(), &file_stat) == 0 && S_ISREG(file_stat.st_mode))
        {
            file_names.push_back(file_name); // Add regular file name to the list
        }
    }

    closedir(dir); // Close the directory
    return file_names;
}

void append_to_file(const std::string &filename, std::stringstream &ss)
{
    std::ofstream file;
    file.open(filename, std::ios::app);

    if (!file)
    {
        std::cerr << "Error opening or creating the file." << std::endl;
        return;
    }

    file << ss.str();
    file.close();
}

visualization_msgs::msg::Marker get_marker(double x, double y, int type, int id, std::string name)
{
    visualization_msgs::msg::Marker amr_marker;
    amr_marker.header.frame_id = "map";
    amr_marker.ns = name;   
    amr_marker.id = id;
    if (type == 0)
    {
        amr_marker.type = visualization_msgs::msg::Marker::CUBE;
        amr_marker.color.r = 0.0f;
        amr_marker.color.g = 1.0f;
        amr_marker.color.b = 0.0f;
        amr_marker.color.a = 1.0f;
    }
    else if (type == 1)
    {
        amr_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        amr_marker.color.r = 1.0f;
        amr_marker.color.g = 0.0f;
        amr_marker.color.b = 0.0f;
        amr_marker.color.a = 1.0f;
    }
    else
    {
        amr_marker.type = visualization_msgs::msg::Marker::SPHERE;
        amr_marker.color.r = 0.0f;
        amr_marker.color.g = 0.0f;
        amr_marker.color.b = 1.0f;
        amr_marker.color.a = 1.0f;
    }
    amr_marker.action = visualization_msgs::msg::Marker::ADD;
    amr_marker.pose.position.x = x;
    amr_marker.pose.position.y = y;
    amr_marker.pose.position.z = 0.5;
    amr_marker.pose.orientation.w = 1.0;
    amr_marker.scale.x = 80;
    amr_marker.scale.y = 80;
    amr_marker.scale.z = 80;

    return amr_marker;
}

visualization_msgs::msg::Marker get_strip_marker(double x_start, double y_start, double x_end, double y_end, int id){
    visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.ns = "line_marker"+std::to_string(id);
        line_strip.id = id;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 20;  // Line width

        // Define line color
        line_strip.color.r = 1.0f;
        line_strip.color.g = 0.0f;
        line_strip.color.b = 0.5f;
        line_strip.color.a = 1.0;

        // Add points to the line (connecting the cube and cylinder)
        geometry_msgs::msg::Point p1, p2;
        p1.x = x_start; p1.y = y_start; p1.z = 0.0;
        p2.x = x_end; p2.y = y_end; p2.z = 0.0;

        line_strip.points.push_back(p1);  // Start point
        line_strip.points.push_back(p2);  
        return line_strip;

}
