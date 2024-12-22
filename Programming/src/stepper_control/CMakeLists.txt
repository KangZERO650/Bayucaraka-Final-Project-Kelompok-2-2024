cmake_minimum_required(VERSION 3.5)
project(stepper_control)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Manually specify the serial library
find_library(SERIAL_LIBRARY NAMES serial PATHS /usr/lib /usr/local/lib)

# Add your executable
add_executable(stepper_node src/stepper_node.cpp)

# Link libraries
target_link_libraries(stepper_node
  rclcpp::rclcpp
  ${OpenCV_LIBS}
  ${SERIAL_LIBRARY}
)

ament_target_dependencies(stepper_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  stepper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
