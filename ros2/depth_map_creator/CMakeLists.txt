cmake_minimum_required(VERSION 3.5)
project(depth_map_creator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lopencv_core)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# ros2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
#opencv
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
#Others
find_package(Eigen3 3.3.7 REQUIRED)

# Executables
add_executable(template src/depth_map_manipulator_template.cpp)
ament_target_dependencies(template rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

add_executable(interpolation src/interpolation_depth_map.cpp)
ament_target_dependencies(interpolation rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

add_executable(disparity src/disparity_from_stereo.cpp)
ament_target_dependencies(disparity rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

add_executable(fill_nxn src/filled_nxn_depth_map.cpp)
ament_target_dependencies(fill_nxn rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

add_executable(canny src/canny.cpp)
ament_target_dependencies(canny rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

add_executable(nearest src/nearestneighbor.cpp)
ament_target_dependencies(nearest rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

add_executable(kittipublisher src/kitti_publisher.cpp)
ament_target_dependencies(kittipublisher rclcpp example_interfaces sensor_msgs OpenCV cv_bridge)

# Install Targets
install(TARGETS 
template
interpolation
disparity
fill_nxn
canny
nearest
kittipublisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
