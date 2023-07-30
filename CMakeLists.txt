cmake_minimum_required(VERSION 3.5)
project(mpu_6050)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_library(pigpiod pigpiod_if2)  # Find pigpio for building node.

# Set the include directories
include_directories(include)

# Add executable
add_executable(mpu_6050_node src/mpu_6050_driver.cpp)

# Check if pigpiod is found and link the dependencies
if(pigpiod)
    message(STATUS "PIGPIO found. Building mpu_6050_node.")
    ament_target_dependencies(mpu_6050_node rclcpp sensor_msgs std_msgs)
    target_link_libraries(mpu_6050_node ${pigpiod})
else()
    message(WARNING "PIGPIO not found. Skipping mpu_6050_node.")
endif()

# Install the executable
install(TARGETS mpu_6050_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install the package.xml file
ament_package()
