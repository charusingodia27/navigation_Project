# testbed_navigation Package

This package provides the configuration, launch files, and the required parameters for integrating map loading, localization, and navigation functionalities using the ROS 2 Navigation stack. It is intended for use in robotic applications to enable autonomous navigation, localization, and path planning.

## Package Contents
- **Configuration Files**: Parameter files for map loading, localization, and controller settings.
- **Launch Files**: Scripts to launch the various components like map loading, localization, and navigation.
- **README.md**: Overview of the package and instructions for usage.

## Prerequisites

Before you can use the `testbed_navigation` package, ensure that you have the following prerequisites:

1. **ROS 2 Humble** installed on your system. Follow the instructions to install ROS 2 Humble from [here](https://docs.ros.org/en/humble/Installation.html).

2. **ROS 2 Navigation2** package installed:
   - To install Navigation2, run:
     ```bash
     sudo apt install ros-humble-navigation2
     ```

3. **Robot description (URDF/XACRO)**: 
   - Ensure that you have the correct robot description in your workspace or modify the launch files to point to your robot's URDF.

4. **Additional dependencies**:
   - Install the required dependencies for the `testbed_navigation` package by running:
     ```bash
     sudo apt install ros-humble-nav2-bringup ros-humble-nav2-common
     ```

## Approach

The package integrates key components from ROS 2 Navigation stack:

1. **Map Loading**:
   - The `nav2_map_server` is used to load the static map. The map is provided as a `.yaml` file with the corresponding image, resolution, and origin.
   
2. **Localization**:
   - The package uses AMCL (Adaptive Monte Carlo Localization) for localization based on the pre-loaded map. The parameters are configured to use laser scan data for position estimation.

3. **Navigation**:
   - Navigation is managed using the Navigation2 stack with a set of parameters configured for the robot’s path planning and movement. The `nav2_controller` and `nav2_planner` are used to compute optimal paths and control the robot’s velocity.

## Directory Structure
```
testbed_navigation/
├── launch/
│   ├── map_loader.launch.py           
│   ├── localization.launch.py
│   ├── navigation.launch.py
├── config/
│   ├── amcl_params.yaml           
│   ├── nav2_params.yaml
└── README.md
```        

- **config/**: Contains the YAML configuration files for the map, AMCL, and controller parameters.
- **launch/**: Contains launch files to initialize and run each node (map, AMCL, navigation).

## Challenges Faced
Parameter Tuning: Configuring the parameters for AMCL and navigation to ensure smooth operation in different environments.
Compatibility: Ensuring that all components of the ROS 2 Navigation stack work together seamlessly.
Localization Accuracy: Ensuring that the localization remains accurate and stable throughout the navigation process.

## Usage
Ensure your robot is set up and the necessary sensors (e.g., LiDAR, camera) are working.
Modify the map.yaml and amcl.yaml to suit your environment and robot's setup.
Run the following command to launch the entire navigation stack:

`ros2 launch testbed_navigation navigation_launch.py`


This will start the map server, AMCL, and the navigation stack to get your robot moving autonomously.
