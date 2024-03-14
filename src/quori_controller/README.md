# quori_controller

quori_controller is an implementation of the ROS control stack for Quori and the RAMSIS base.

## Usage

### `roslaunch quori_controller quori_control_holo.yaml`

Launches the ROS control stack for Quori and the RAMSIS base in holonomic mode. The turret joint will not be accessible to ROS, as it is used in generating the psuedoholonomic behavior.

### `roslaunch quori_controller quori_control_diff.yaml`

Launches the ROS control stack for Quori and the RAMSIS base in differential drive mode. The turret joint will be accessible via ROS.

### Calibration and `calibration.yaml`
The base turret orientation is calibrated with the `calibration.yaml` file. Open the file and set `base_offset:` to your robot's correct value. This value can be updated later if you update your hardware. To learn about calibrating the arms and spine see [quori_embedded](https://github.com/semio-ai/quori_embedded/blob/master/README.md)

### ROS2 Conversion Changed Files 3/13/24
CMakeLists.txt
package.xml
quori_control_holo.launch.py
calibrate_node.cpp
Csv.cpp
Csv.hpp
Quori.cpp
Quori.hpp
quori_controller_node.cpp
SerialDevice.cpp
SerialDevice.hpp

### ROS2 Conversion Issues 3/13/24
CURRENTLY PACKAGE DOES NOT BUILD!!!

generate_dynamic_reconfigure_options no longer supported in ROS2

hardware_interface structured differently in ROS2 Galactic and above . Specifically a problem with BaseInterface see:
https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html
