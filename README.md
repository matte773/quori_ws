 **quori_ws**

This is an initial attempt to convert the quori_ros running in ROS Noetic to ROS2 Humble. Currently quori_gazebo, quori_description and quori_controller are the only packages converted to ROS2. The rest of the ROS2 packages are empty. quori_controller does not compile to see the issues view the README in the package folder.

**Prerequisites**

The ROS2 workspace is compatible with Ubuntu 22.04 / ROS Humble.
The ROS1 workspace is compatible with Ubuntu 20.04 / ROS Noetic.
A working internet connection that allows for ROS communication past the firewall. 

**Future Work**

There are a number of options in order to get the quori project working in ROS2.

1. Continue to migrate the ROS1 packages to ROS2. https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html
 



**Refrences**
- https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/
- https://github.com/ros2/ros1_bridge
- 
