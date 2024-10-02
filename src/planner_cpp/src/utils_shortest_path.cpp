#include "planner_cpp/utils_shortest_path.hpp"

using tuple_pos = std::tuple<double, double, std::string>;

double distance_between(const tuple_pos &a, const tuple_pos &b)
{
    double x1 = std::get<0>(a), y1 = std::get<1>(a);
    double x2 = std::get<0>(b), y2 = std::get<1>(b);
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

std::pair<double, std::vector<std::string>> tsp_shortest_path(
    const tuple_pos &start,
    const tuple_pos &goal,
    const std::vector<tuple_pos> &intermediates)
{
    int n = intermediates.size(); // Number of intermediate points

    // Precompute the distances between all positions (including start)
    std::vector<std::vector<double>> dist(n + 1, std::vector<double>(n + 1, 0.0));
    for (int i = 0; i < n; ++i)
    {
        dist[0][i + 1] = distance_between(start, intermediates[i]); // Distance from start to intermdiates
        dist[i + 1][0] = dist[0][i + 1];                            // Symetry
    }
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            dist[i + 1][j + 1] = distance_between(
                intermediates[i],
                intermediates[j]);                   // Intermediates
            dist[j + 1][i + 1] = dist[i + 1][j + 1]; // Symmetry
        }
    }

    // Dynamic programming table to store minimum distances
    std::vector<std::vector<double>> dp(
        1 << n,
        std::vector<double>(n, std::numeric_limits<double>::infinity()));

    // Tracking array to store the previous node for each state
    std::vector<std::vector<int>> prev(
        1 << n, std::vector<int>(n, -1));

    // Initial state: from start to each intermediate
    for (int i = 0; i < n; ++i)
        dp[1 << i][i] = dist[0][i + 1]; // Distance from start to intermediate i

    // Iterate over all subsets of intermediate positions
    for (int mask = 1; mask < (1 << n); ++mask)
    {
        for (int u = 0; u < n; ++u)
        {
            if (!(mask & (1 << u)))
                continue; // Skip if u is not in the current subset (mask)

            // Try to extend the path to another position v
            for (int v = 0; v < n; ++v)
            {
                if (mask & (1 << v))
                    continue;                   // Skip if v is already visited in the current subset
                int new_mask = mask | (1 << v); // New subset with v added
                double new_distance = dp[mask][u] + dist[u + 1][v + 1];
                if (new_distance < dp[new_mask][v])
                {
                    dp[new_mask][v] = new_distance;
                    prev[new_mask][v] = u; // Track the previous node
                }
            }
        }
    }

    // Find the minimum path to the goal from the last intermediate position
    double min_path_length = std::numeric_limits<double>::infinity();
    int last_node = -1;
    for (int i = 0; i < n; ++i)
    {
        double final_distance = dp[(1 << n) - 1][i] + distance_between(intermediates[i], goal);
        if (final_distance < min_path_length)
        {
            min_path_length = final_distance;
            last_node = i; // This is the last intermediate node before reaching the goal
        }
    }

    // Backtrack to find the order of visited intermediate nodes
    std::vector<std::string> path;
    int mask = (1 << n) - 1; // Full set of visited nodes
    while (last_node != -1)
    {
        path.push_back(std::get<2>(intermediates[last_node])); // Add the index of the intermediate node
        int next_mask = mask ^ (1 << last_node);               // Remove the last node from the bitmask
        last_node = prev[mask][last_node];                     // Move to the previous node
        mask = next_mask;
    }

    std::reverse(path.begin(), path.end()); // Reverse the path to get the correct order

    return {min_path_length, path};
}
