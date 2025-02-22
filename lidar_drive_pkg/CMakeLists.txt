cmake_minimum_required(VERSION 3.8)
project(create3_rpi_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rplidar_ros REQUIRED)


install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)
ament_package()

