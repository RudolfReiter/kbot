cmake_minimum_required(VERSION 3.5)
project(planner_cpp)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kbot_interfaces REQUIRED)

include_directories(include)

add_executable(order_optimizer 
src/orderOptimizer.cpp 
src/utils_parsing.cpp 
src/utils_shortest_path.cpp 
src/utils.cpp)

ament_target_dependencies(order_optimizer rclcpp geometry_msgs std_msgs kbot_interfaces)

install(
  DIRECTORY include/
  DESTINATION include
)


install(TARGETS
  order_optimizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
