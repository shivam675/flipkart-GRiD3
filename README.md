# flipkart GRiD3 Robotics challenge 
This project is a solution to the problem statement provided by flipkart GRiD 3.0

# Clone repo instructions
```sh
git clone --recurse-submodules https://github.com/shivam675/flipkart-GRiD3.git
```

# Create Markers:

**1. use the createmarkers.py node in make_markers package :**


### How to build:
- In terminal `catkin_make -DCMAKE_BUILD_TYPE=Release`

### How to run:

<!-- 1. To open arm in Gazebo | Terminal 1: `roslaunch arm_gazebo gazebo_spawn.launch` -->
<!-- 2. To open arm in Rviz   | Terminal 2: `roslaunch arm_prismatic_octomap bringup.launch` -->
1. Run **ros_camera.launch** file | Terminal 1 (camera_driver): `roslaunch camera_driver ros_camera.launch`
2. Run **aruco_detect_ros.launch** fine | Terminal 2 (aruco node): `roslaunch make_markers aruco_detect_ros.launch`


# Tested on System config:
- Hardware: i3 quad thread AMD64
- ROS: Melodic
- OS: Ubuntu 18.04 LTS
- 2-core AMD
- 8 GB RAM

# To-do:
1. Camera calibration for intrensic and extrensic params
2. Multiple aruco_marker detection
3. python node to publish all the tfs of the aruco markers in ros
