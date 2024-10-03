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
find_package(visualization_msgs REQUIRED)
find_package(kbot_interfaces REQUIRED)

include_directories(include)

add_executable(order_optimizer 
src/orderOptimizer.cpp 
src/utils_parsing.cpp 
src/utils_shortest_path.cpp 
src/utils.cpp)
ament_target_dependencies(order_optimizer rclcpp geometry_msgs std_msgs kbot_interfaces visualization_msgs)

add_executable(position_publisher src/posePublisher.cpp)
ament_target_dependencies(position_publisher rclcpp geometry_msgs)

add_executable(order_publisher src/orderPublisher.cpp)
ament_target_dependencies(order_publisher rclcpp kbot_interfaces)



install(
  DIRECTORY include/
  DESTINATION include
)


install(TARGETS
  order_optimizer
  position_publisher
  order_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)

  set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/test_data")

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/tests.cpp 
  src/utils.cpp 
  src/utils_shortest_path.cpp
  src/utils_parsing.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
  )
  
endif()

ament_package()
