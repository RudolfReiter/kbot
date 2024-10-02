#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <set>

#include "planner_cpp/utils.hpp"

double Position::distance_to(const Position &other) const
{
    return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
}

// Function to get all file names within a folder
std::vector<std::string> get_all_files_in_directory(std::string& folder_path) {
    std::vector<std::string> file_names;
    DIR* dir = opendir(folder_path.c_str()); // Open the directory

    // Check if directory was opened successfully
    if (dir == nullptr) {
        std::cerr << "Error: Could not open directory " << folder_path << std::endl;
        return file_names; // Return empty vector
    }

    struct dirent* entry;  // Structure to hold directory entry
    struct stat file_stat; // Structure to hold file information

    // Iterate over directory entries
    while ((entry = readdir(dir)) != nullptr) {
        std::string file_name = entry->d_name;

        // Skip "." and ".." directories
        if (file_name == "." || file_name == "..") {
            continue;
        }

        std::string full_path = folder_path + "/" + file_name;

        // Get file status to check if it's a regular file
        if (stat(full_path.c_str(), &file_stat) == 0 && S_ISREG(file_stat.st_mode)) {
            file_names.push_back(file_name); // Add regular file name to the list
        }
    }

    closedir(dir); // Close the directory
    return file_names;
}