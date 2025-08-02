cmake_minimum_required(VERSION 3.8)
project(pca9685)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Add the library for the PCA9685 hardware logic
add_library(pca9685_driver src/pca9685_driver.cpp)
target_include_directories(pca9685_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pca9685_driver rclcpp)

# Add the library for the node component
add_library(pca9685_node_component src/pca9685_node.cpp)
target_include_directories(pca9685_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add rclcpp_components to the dependencies for this target
ament_target_dependencies(pca9685_node_component rclcpp std_msgs rclcpp_components)
target_link_libraries(pca9685_node_component pca9685_driver)

# Register the component and create a dedicated executable container for it
rclcpp_components_register_node(pca9685_node_component
  PLUGIN "pca9685::PCA9685Node"
  EXECUTABLE pca9685_node_executable)

# Install targets
install(
  TARGETS
  pca9685_driver
  pca9685_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

install(
  DIRECTORY include/
  DESTINATION include)

ament_package()