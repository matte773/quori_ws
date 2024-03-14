cmake_minimum_required(VERSION 3.8)
project(quori_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclpy REQUIRED)

# Add quori_description package
find_package(quori_description REQUIRED)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files, if you have them.
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# If the package includes C++ nodes, you can compile and install them like this:
# add_executable(node_name src/node_name.cpp)
# ament_target_dependencies(node_name rclcpp)
# install(TARGETS node_name DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  rclcpp
  gazebo_ros
  rclpy
  quori_description 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
