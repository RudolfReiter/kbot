# KBot ROS 2 Project
![Title Image](./data/header.png)
## Overview

The KBot project features an `OrderOptimizer`, which is a ROS 2 node designed to handle task optimization for Autonomous Mobile Robots (AMR). It subscribes to topics related to the robot's current position and orders, calculates the geometrically shortest path to collect necessary parts for an order, and publishes the path as a marker array. The goal of this node is to demonstrate practical coding skills, including threading, parsing files, and working within the ROS 2 ecosystem.

This project adheres to the following key conditions:

- Written in **C++**.
- Targeted for ROS 2 **Foxy Fitzroy**.
- Runs on **Ubuntu 20.04 LTS**.
- Utilizes multi-threading for file parsing.
- Publishes a `visualization_msgs::MarkerArray` for visualizing the AMR’s optimized path.

## Features

- **ROS 2 Topics**:
  - Subscribes to:
    - `currentPosition` (`geometry_msgs::msg::PoseStamped`): The AMR’s current position.
    - `nextOrder` (custom message `Order`): Provides the ID and description of the next order.
  - Publishes:
    - `order_path` (`visualization_msgs::msg::MarkerArray`): Marker visualization of the AMR’s optimized path.
  
- **Multithreaded Parsing**:
  - Parses multiple order files simultaneously using separate threads to improve performance.
  - Each order file contains orders with destinations and required products.

- **Configuration Parsing**:
  - Loads and caches product part locations from a configuration file at startup, assuming it rarely changes.

- **Shortest Path Calculation**:
  - Computes the geometrically shortest path for the AMR to collect all required parts from specified locations and deliver them to the final destination.
  
## Installation

### Prerequisites

- **ROS 2 Foxy Fitzroy** installed on **Ubuntu 20.04 LTS**.
- `colcon` for building ROS 2 packages.

### Building the Package



1. Clone the repository to your workspace folder:

   ```bash
   git clone https://github.com/RudolfReiter/kbot.git

2. Change to workspace directory:

    ```bash
    cd <path-to-your-workspace>

3. Source the ros setup.bash. Either source in terminal or add to global .bashrc

    ```bash
    source /opt/ros/foxy/setup.bash

4. Build the project and run the unit tests

    ```bash
    colcon build
    colcon test

5. Source the bashrc of the KBot project install folder

    ```bash
    source <path-to-your-workspace>/install/setup.bash


## Launch Demonstration Examles
To show the proper implementation of the node, three demonstrations can be launched. For each demonstration, two publishers of the current position and the order number are launched. Moreover, RVIZ is started in order to vizualize the shortest path to the pick-up locations. The output files are stored in the directory `<path-to-your-workspace>/data` or in the directory of the launch file. 

- **Example 1**: Samples from the original list.

    In this example, several different order numbers (1100002 until 1100010) are published every two seconds. The optimized path is updated in RVIZ.
    ```bash
    ros2 launch kbot_launch test1.launch.py data_path:=<path-to-your-workspace>/data

- **Example 2**: Parts in square.

    In this example, one order with two products and 13 parts are published. The published parts are located on a square and the goal location is close to the starting location at an edge of the square. The example should vizualize an obvious case of the shortest path. The optimized path is updated in RVIZ.
    ```bash
    ros2 launch kbot_launch test2.launch.py data_path:=<path-to-your-workspace>/data/test_cases/test_circular/

- **Example 3**: Random Parts.

    In this example, one order with two products and 13 randomly placed parts are published. The example should vizualize a more complex case of the shortest path. The optimized path is updated in RVIZ.
    ```bash
    ros2 launch kbot_launch test3.launch.py data_path:=<path-to-your-workspace>/data/test_cases/test_random/

## Implementation Details
In the following, some details of the implementation are listed.

- **Data structures to store products and parts**:
  - Parts and products are stored in hash tables ("unordered_map" in C++) in order to achive fast look-up in O(n).

- **Shortest path computation**:
  - The computation of the shortest path resembles an instance of the traveling salesman problem, which is known to be NP complete. For problem sizes of up to 20 pick-up locations, a dynamic programming approach was implemented.

- **Output format of orders**:
  - The output file "completed_orders.txt" lists the individual actions in consecutive order. Particularly, for each picked up part, the number of parts and the related product are listed, in addition to the location.

- **Publishing of the optimized path**:
  - In addition to the pickup locations of the parts (blue cylinders) and the robot position (green cube), RVIZ visualizes the path (red) and the delivery location of the current order (blue sphere).

- **Testing**:
  - Several important components are unit tested with the gtest framework. 
