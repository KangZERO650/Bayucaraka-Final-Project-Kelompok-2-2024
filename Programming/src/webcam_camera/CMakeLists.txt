cmake_minimum_required(VERSION 3.5)
project(webcam_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
set(OpenCV_DIR "/usr/local/share/opencv4")
  
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)

add_definitions(-DOPENCV_DNN_EXTERN_PROTOBUF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Buat executable
add_executable(camera_node src/camera_node.cpp)

# Tambahkan dependensi target
ament_target_dependencies(camera_node 
  rclcpp 
  cv_bridge 
  sensor_msgs 
  geometry_msgs 
  image_transport
)

target_link_libraries(camera_node
  ${OpenCV_LIBS}
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_videoio
  opencv_dnn
)

# Tambahkan include directories
target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Tentukan direktori instalasi
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()