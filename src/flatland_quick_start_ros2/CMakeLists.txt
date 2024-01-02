cmake_minimum_required(VERSION 3.5)
project(flatland_quick_start_ros2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ament_export_dependencies(geometry_msgs sensor_msgs roscpp message_runtime)
include_directories(
  include
)


# Install launchfiles
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# Install assets
install(DIRECTORY flatland_worlds
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY flatland_models
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/custom_robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
