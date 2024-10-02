#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

using tuple_t = std::tuple<double, double, int>;
using std::get;

// A utility function to calculate the Euclidean distance between two positions
double distance_between(const tuple_t & a, const tuple_t & b) {
    return std::sqrt((get<0>(a) - get<0>(b)) * (get<0>(a) - get<0>(b)) + (get<1>(a) - get<1>(b)) * (get<1>(a) - get<1>(b)));
}

// Function to compute the total path distance for a specific order of intermediate points
double calculate_path_length(const tuple_t & start, const tuple_t & goal, const std::vector<tuple_t >& positions) {
    double total_distance = 0.0;

    // Start from the start position to the first intermediate point
    total_distance += distance_between(start, positions[0]);

    // Visit all intermediate points in the given order
    for (size_t i = 1; i < positions.size(); ++i) {
        total_distance += distance_between(positions[i - 1], positions[i]);
    }

    // Finally, go from the last intermediate point to the goal position
    total_distance += distance_between(positions.back(), goal);

    return total_distance;
}

// Function to compute the shortest path
double find_shortest_path(const tuple_t & start, const tuple_t & goal, std::vector<tuple_t > intermediates) {
    double min_path_length = std::numeric_limits<double>::infinity();

    // Try all permutations of the intermediate positions
    std::sort(intermediates.begin(), intermediates.end());
    do {
        // Compute the total distance for the current permutation of intermediate points
        double current_path_length = calculate_path_length(start, goal, intermediates);
        
        // Update the minimum path length if the current one is shorter
        if (current_path_length < min_path_length) {
            min_path_length = current_path_length;
        }
    } while (std::next_permutation(intermediates.begin(), intermediates.end()));

    return min_path_length;
}

int main() {
    // Example usage:
    tuple_t  start{0.0, 0.0, -1}; // Starting position
    tuple_t  goal{10.0, 10.0, -1}; // Goal position

    std::vector<tuple_t > intermediates = {
        {10.0, 0.0, 1},
        {10.0, 5.0, 2},
        {5.0, 0.0, 3},
    };

    double shortest_path = find_shortest_path(start, goal, intermediates);

    std::cout << "Shortest path length: " << shortest_path << std::endl;

    return 0;
}