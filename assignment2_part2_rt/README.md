# Assignment1_Part2_RT
This package contains a ROS 2 node for controlling a robot in a Gazebo simulation. The robot is spawned at position (2, 2) in the simulation environment, and the node publishes velocity commands to make the robot move.



## Overview
### Nodes
1. **`move_robot_node`**
   - **Location**: `src/move_robot_node.cpp`
   - **Functionality**:
     - Publishes velocity commands to the `/cmd_vel` topic to control the robot's linear and angular velocity.
     - Moves the robot with a linear velocity of 0.5 m/s and an angular velocity of 1.0 rad/s.

### Launch Files
1. **`gazebo.launch.py`**
   - **Functionality**:
     - Spawns the robot in the Gazebo simulation at position (2, 2).
     - Launches the `robot_state_publisher` and `joint_state_publisher` nodes.
     - Starts the `move_robot_node` for controlling the robot.
     - Opens RViz with a preconfigured setup.



## Installation
### Prerequisites
- ROS 2 Foxy installed on your system.
- Gazebo simulator installed:
    ```bash
    sudo apt-get install ros-foxy-gazebo-ros-pkgs
    ```

### Build Instructions
- Navigate to your workspace:
    ```bash
    cd ~/ros2_ws
    ```
- Build the package using `colcon`:
    ```bash
    colcon build --packages-select assignment2_part2_rt
    ```
- Source your workspace:
    ```bash
    source install/setup.bash
    ```



## Usage
### Steps to Launch
1. Launch the Gazebo simulation and the robot:
    ```bash
    ros2 launch assignment2_part2_rt gazebo.launch.py
    ```

2. Control the robot:
   - The `move_robot_node` will automatically send velocity commands to move the robot.



## File Structure
    assignment2_part2_rt
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── config.yaml
    │   ├── rviz.rviz
    ├── launch/
    │   ├── gazebo.launch.py
    ├── src/
    │   ├── move_robot_node.cpp
    ├── urdf/
    │   ├── robot4.xacro
    │   ├── robot4.gazebo



## Topics 
- **Published**:
    - `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands for controlling the robot's movement.

