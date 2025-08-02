cmake_minimum_required(VERSION 3.8)
project(pca9685_cpp)

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


# Add the node component that uses the library
add_library(pca9685_node src/pca9685_node.cpp)
target_include_directories(pca9685_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pca9685_node rclcpp std_msgs)
target_link_libraries(pca9685_node pca9685_driver)

# Add the executable that will host the node component
add_executable(pca9685_executable src/main.cpp)
ament_target_dependencies(pca9685_executable rclcpp)
target_link_libraries(pca9685_executable pca9685_node)

# Export the node component so it can be used by composition
rclcpp_components_register_node(pca9685_node
  PLUGIN "pca9685_cpp::PCA9685Node"
  EXECUTABLE pca9685_executable)

# Install targets
install(
  TARGETS
  pca9685_driver
  pca9685_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

install(
  TARGETS pca9685_executable
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

ament_package()
