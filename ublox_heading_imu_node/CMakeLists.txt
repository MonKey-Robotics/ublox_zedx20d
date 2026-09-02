cmake_minimum_required(VERSION 3.22)
project(ublox_heading_imu_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)

# include directories
include_directories(include SYSTEM)

add_library(ublox_heading_imu_components SHARED
  src/ublox_heading_imu_node.cpp
)

target_link_libraries(ublox_heading_imu_components PUBLIC
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${ublox_ubx_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)

rclcpp_components_register_node(ublox_heading_imu_components
  PLUGIN "ublox_heading_imu::UbloxHeadingImuNode"
  EXECUTABLE ublox_heading_imu)

# install executable
install(TARGETS
ublox_heading_imu_components
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_heading_conversion test/test_heading_conversion.cpp)
  target_include_directories(test_heading_conversion PRIVATE include)
endif()

ament_package()
