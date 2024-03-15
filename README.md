# quori_ws

This is an initial attempt to convert the quori_ros running in ROS Noetic to ROS2 Humble. Currently quori_gazebo, quori_description and quori_controller are the only packages converted to ROS2. The rest of the ROS2 packages are empty. quori_controller does not compile to see the issues view the README in the package folder. So build with the following command: colcon build --symlink-install --packages-skip quori_controller

## Prerequisites

- The ROS2 workspace is compatible with Ubuntu 22.04 / ROS Humble.
- Git must be installed on the ROS2 system.
- The ROS1 workspace is compatible with Ubuntu 20.04 / ROS Noetic.
- A working internet connection that allows for ROS communication past the firewall. 

## Setup

To begin this setup we will start with the Ubuntu 22.04 / ROS Humble system. In a terminal run: 

``` bash
cd ~/
git clone https://github.com/matte773/quori_ws.git
```
*Note: change the location of the cd command if you dont want this cloned to the /home/ directory

This sets up the ROS2 workspace that we will use for this project.

The following section heavily relies on TommyChangUMD's ros-humble-ros1-bridge-builder project. See references for further information.

The following docker build takes some time. I have had up to 22 minute build times with a 8-core CPU and 32GB of memory. 

### How to create this builder docker image:

``` bash
  git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
  cd ros-humble-ros1-bridge-builder

  # By default, ros-tutorals support will be built: (bridging the ros-humble-example-interfaces package)
  docker build . -t ros-humble-ros1-bridge-builder
```

*Note: Since building the docker image just needs docker, you could do this step on any system that has docker installed, it is recommended to do this on the ROS2 system. 

Alternative builds:
```
  # **[OPTIONAL]** If you don't want to build ros-tutorals support:
  docker build . --build-arg ADD_ros_tutorials=0 -t ros-humble-ros1-bridge-builder
  
  # **[OPTIONAL]** If you want to build grid-map support:  (bridging the ros-humble-grid-map package)
  docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder
```
*Note: Don't forget to install the necessary `ros-humble-grid-map` packages on your ROS2 Humble if you choose to build the bridge with the `grid-map` support added.

### How to create ros-humble-ros1-bridge package:
####  0.) Start from the latest Ubuntu 22.04 ROS 2 Humble Desktop system, create the "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    cd ~/
    apt update; apt upgrade
    apt -y install ros-humble-desktop
    docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

Note1, it's **important** that you have **`ros-humble-desktop`** installed on your ROS2 Humble system because we want to **match it with the builder image as closely as possible**.  So, if you haven't done so already, do:
``` bash
    apt -y install ros-humble-desktop
```
Otherwise you may get an error about missing `ibexample_interfaces__rosidl_typesupport_cpp.so`.  See issue https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/10


Note1: There is no compilation at this point, the `docker run` command simply spits out a pre-compiled tarball.  The assumption is that this tarball contains configurations and libraries matching your ROS2 Humble system very closely, although not identical.

Note2: We don't need the builder image anymore, to delete it, do:

``` bash
    docker rmi ros-humble-ros1-bridge-builder
```
### How to use ros-humble-ros1-bridge:
####  1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:

``` bash
  rocker --x11 --user --home --privileged \
         --volume /dev/shm /dev/shm --network=host -- osrf/ros:noetic-desktop \
         'bash -c "sudo apt update; sudo apt install -y tilix; tilix"'
```
You may need to install rocker first:
``` bash
  sudo apt install python3-rocker
```
Note: It's important to share the host's network and the `/dev/shm/` directory with the container.

####  2.) Then, start "roscore" inside the ROS1 Noetic docker container

``` bash
  source /opt/ros/noetic/setup.bash
  roscore
```

####  3.) Now, from the Ubuntu 22.04 ROS2 Humble system, start the ros1 bridge node.

``` bash
  source /opt/ros/humble/setup.bash
  source ~/ros-humble-ros1-bridge/install/local_setup.bash
  ros2 run ros1_bridge dynamic_bridge
```
*Note: We need to source `local_setup.bash` and NOT `setup.bash` because the bridge was compiled in a docker container that may have different underlay locations.  Besides, we don't need to source these underlays in the host system again.

####  3.) Back to the ROS1 Noetic docker container, run in another terminal tab:

``` bash
  source /opt/ros/noetic/setup.bash
  rosrun rospy_tutorials talker
```

####  4.) Finally, from the Ubuntu 22.04 ROS2 Humble system:

``` bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp listener
```
Note: If you have run into any issues to this point please see the troubleshooting and reference sections at the bottom of this README.

### Network Connection Setup:

It is critical that all the systems are connected to the same network connection.

On the Quori open a terminal and run: 
``` bash
Hostname -I
```
The output of this would be an IP address for the Quori. We will refer to this as {$QUORI_IP}.

On the Quori run: 

``` bash
export ROS_IP={QUORI_IP}
export ROS_MASTER_URI=http://{QUORI_IP}:11311
```

Note: Port can be changed but must be consistant throughout all export commands.

In Tilix/Docker Noetic system open a terminal and run: 
``` bash
Hostname -I
```
The output of this would be an IP address for the Quori. We will refer to this as {$ROS1_IP}.

In Tilix/Docker Noetic system run: 

``` bash
export ROS_IP={$ROS1_IP}
export ROS_MASTER_URI=http://**{QUORI_IP}**:11311
```

In the ROS2/Humble system open a terminal and run: 
``` bash
Hostname -I
```
The output of this would be an IP address for the Quori. We will refer to this as {$ROS2_IP}.

In the ROS2/Humble system run: 

``` bash
export ROS_IP={$ROS2_IP}
export ROS_MASTER_URI=http://**{QUORI_IP}**:11311
```
Note: It is critical to set these on the ROS2 system for the bridge to work correctly, regardless of how ROS2 handles network communication.

## Testing 

Similar to above we will be using the talker and listener tutorial nodes to test a basic functionality.

If you are missing a terminal or system repeat the steps from the "How to use ros-humble-ros1-bridge" section of this README. 

On the Quori run:
``` bash
  source /opt/ros/noetic/setup.bash
  rosrun rospy_tutorials talker
```

From the Ubuntu 22.04 ROS2 Humble system:

``` bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp listener
```
Note: If this doesnt work start by running the following on the Tilix/ Docker Noetic system:
``` bash
rostopic list
```

Here we should see a topic called /chatter

If it isnt there then in order try:
- running ```$ROS_IP``` and ```$ROS_MASTER_URI``` on all systems, make sure that all of the MASTER's is set to the QUORI_IP and that all the ROS_IP's on each system is unique.
- run ```rostopic list``` on the Tilix system and see if the the /chatter topic is there. If its there its an issue with the bridge, if it isn't it is a network issue. 
- If they are the same please repeat section "Network Connection Setup" from this README

## Future Work

There are a number of options in order to get the quori project working in ROS2.

1. Continue to migrate the ROS1 packages to ROS2. https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html
2. New clean build creating a quori system in ROS2 that can follow a new architecture which can be different from the original.
3. Hybrid with bridge. With ROS2 Nodes communicating with ROS1 nodes over topics.

Not all ROS topics are on the bridge. (solutions)
1. Figure out how to write a costum topic.
2. Remap the topics.
3. Build a costum bridge.
4. Galactic computer. 

## Troubleshoot

### Check tf2 message / service
``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i tf2
  - 'tf2_msgs/msg/TF2Error' (ROS 2) <=> 'tf2_msgs/TF2Error' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf2_msgs/TFMessage' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf/tfMessage' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```

### Check AddTwoInts message / service
By default, `--build-arg ADD_ros_tutorials=1` is implicitly added to the `docker build ...` command.

Note: In addition, the ROS2 Humble system must have the `ros-humble-example-interfaces` package installed.
``` bash
$ sudo apt -y install ros-humble-example-interfaces
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i addtwoints
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'roscpp_tutorials/TwoInts' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'rospy_tutorials/AddTwoInts' (ROS 1)
```

### Check grid-map message / service
Must have `--build-arg ADD_grid_map=1` added to the `docker build ...` command.

Note: In addition, the ROS2 Humble system must have the `ros-humble-grid-map` package installed.
``` bash
$ sudo apt -y install ros-humble-grid-map
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i grid_map
  - 'grid_map_msgs/msg/GridMap' (ROS 2) <=> 'grid_map_msgs/GridMap' (ROS 1)
  - 'grid_map_msgs/msg/GridMapInfo' (ROS 2) <=> 'grid_map_msgs/GridMapInfo' (ROS 1)
  - 'grid_map_msgs/srv/GetGridMap' (ROS 2) <=> 'grid_map_msgs/GetGridMap' (ROS 1)
  - 'grid_map_msgs/srv/GetGridMapInfo' (ROS 2) <=> 'grid_map_msgs/GetGridMapInfo' (ROS 1)
  - 'grid_map_msgs/srv/ProcessFile' (ROS 2) <=> 'grid_map_msgs/ProcessFile' (ROS 1)
  - 'grid_map_msgs/srv/SetGridMap' (ROS 2) <=> 'grid_map_msgs/SetGridMap' (ROS 1)
```

#### Error: gibgrid_map_msgs__rosidl_typesupport_cpp.so: cannot open shared object file: No such file or directory
``` bash
$ sudo apt -y install ros-humble-grid-map
```

## References
- https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/
- https://github.com/ros2/ros1_bridge
- https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
- https://index.ros.org/p/ros1_bridge/
