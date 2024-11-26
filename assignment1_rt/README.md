# Assignment1_RT

This package contains ROS nodes for interacting with turtles in the `turtlesim` simulator. The package includes nodes for controlling turtle movement, spawning turtles, and monitoring distances between turtles.

## Overview

### Nodes
1. **`ui_node1`**
   - **Location**: `scripts/ui_node1.py` or `src/ui_node1.cpp`
   - **Functionality**:
     - Spawns a new turtle (`turtle2`) in the simulator.
     - Allows the user to control the movement of `turtle1` or `turtle2` through a simple text-based interface.
     - Sends velocity commands to the selected turtle for 1 second and then stops it.

2. **`distance_node2`**
   - **Location**: `scripts/distance_node2.py` or `src/distance_node2.cpp`
   - **Functionality**:
     - Monitors the distance between `turtle1` and `turtle2` and publishes it on the `/turtles_distance` topic.
     - Stops turtles if they come too close or approach the boundaries of the simulation area.

## Installation

### Prerequisites
- ROS Noetic installed on your system.
- `turtlesim` package installed:
    ```bash
    sudo apt-get install ros-noetic-turtlesim
    ```

## Usage
### Steps
1. **Launch the turtlesim simulator**
    ```bash
    rosrun turtlesim turtlesim_node
    ```

2. **Start the ui_node1 node**
   - **Python**:
    ```bash
    rosrun assignment1_rt ui_node1.py
    ```
   - **C++**:
    ```bash
    rosrun assignment1_rt ui_node1
    ```

3. **Start the distance_node2 node**
   - **Python**:
    ```bash
    rosrun assignment1_rt distance_node2.py
    ```
   - **C++**:
    ```bash
    rosrun assignment1_rt distance_node2
    ```

### User Instructions
- **`ui_node1`**
    - Choose a turtle (`turtle1` or `turtle2`) by entering the number.
    - Specify linear and angular velocities.
    - The turtle will move for 1 second and then stop.
- **`distance_node2`**
    - Monitors the distance between turtles and logs warnings when they are too close or near the boundaries.

## File Structure
    assignment1_rt/
    ├── CMakeLists.txt
    ├── package.xml
    ├── src/
    │   ├── ui_node1.cpp
    │   ├── distance_node2.cpp
    ├── scripts/
    │   ├── ui_node1.py
    │   ├── distance_node2.py

## Topics 
- **Published**:
    - `/turtles_distance` (`std_msgs/Float32`): Distance between `turtle1` and `turtle2`.
- **Subscribed**:
    - `/turtle1/pose` (`turtlesim/Pose`): Pose of `turtle1`.
    - `/turtle2/pose` (`turtlesim/Pose`): Pose of `turtle2`.
