cmake_minimum_required(VERSION 3.8)
project(quori_controller)

# Compile as C++14, supported in ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)

# Include directories are not strictly needed to be declared in ROS 2,
# ament_target_dependencies handles it, but you can include specific ones if needed.
# include_directories(include)

# Declare C++ executable targets
add_executable(${PROJECT_NAME}_node
  src/quori_controller_node.cpp
  src/Quori.cpp
  src/Csv.cpp
  src/SerialDevice.cpp
  src/Joint.cpp
  src/message.cpp
)

add_executable(${PROJECT_NAME}_init
  src/init.cpp
  src/Quori.cpp
  src/Csv.cpp
  src/SerialDevice.cpp
  src/Joint.cpp
  src/message.cpp
)

# Set target properties if needed
# The PREFIX "" is useful if you're specifically aiming to avoid default naming conventions.
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_init PROPERTIES OUTPUT_NAME init PREFIX "")

# Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  hardware_interface
  controller_manager
)

ament_target_dependencies(${PROJECT_NAME}_init
  rclcpp
  hardware_interface
  controller_manager
)

# Install executables and/or libraries
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_init
  DESTINATION lib/${PROJECT_NAME}
)

# If you have any custom messages, services or actions, they should be handled like this:
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage1.msg"
#   "srv/YourService1.srv"
#   # More message/service definitions here
# )

# If you have configurations for parameters, consider using the ROS 2 parameter API to load them
# instead of the dynamic_reconfigure approach from ROS 1.

# ament_export_dependencies(rosidl_default_runtime) # if you have custom messages/services

ament_package()
