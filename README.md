# navigation_Project

This project integrates existing ROS packages to build a complete robot navigation system in simulation. It uses simulation environments, robot models, and navigation logic to demonstrate a real-world inspired navigation stack.

## Project Structure

The following packages are part of this workspace:

- **`testbed_bringup`** – Launch files and overall orchestration
- **`testbed_description`** – Robot URDF and sensor setup
- **`testbed_gazebo`** – Simulation world and plugins
- **`testbed_navigation`** – Custom package developed for path planning, localization, and robot control

## 🔍 How It Works

The main logic and navigation setup are implemented in the custom package:  
 **[`testbed_navigation`](./testbed_navigation)**

This package links the other modules and brings everything together — from robot spawning to executing navigation goals using ROS navigation stack components.

For detailed steps, features, and configurations, check the README inside `testbed_navigation`.

---
