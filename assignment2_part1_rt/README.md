# Assignment1_Part1_RT
This package contains ROS nodes for controlling a robot to reach specified targets and monitoring its position and velocity using an action server. The package includes nodes for setting goals, monitoring status, and calculating distance and speed averages.

## Overview
### Nodes
1. **`action_client`**
   - **Location**: `scripts/action_client.py`
   - **Functionality**:
     - Allows the user to set a target (x, y) or cancel the current goal using a console interface.
     - Publishes the robot's feedback/status to track the progress towards the target.
     - Provides status updates through the `/status` topic, including position (x, y) and velocity (linear and angular).

2. **`bug_as`**
   - **Location**: `scripts/bug_as.py`
   - **Functionality**:
     - Implements the action server to handle the robot's movement towards a specified target.
     - Manages obstacle avoidance using laser scan data to follow walls when necessary.
     - Publishes robot control commands on the `/cmd_vel` topic.

3. **`dis_avg`**
   - **Location**: `scripts/dis_avg.py`
   - **Functionality**:
     - Returns the robot's distance from the target.
     - Returns the average speed along the x-axis.
     - Returns the average angular velocity along the z-axis.

4. **`go_to_point_service`**
   - **Location**: `scripts/go_to_point_service.py`
   - **Functionality**:
     - Controls the robot's motion towards a target point using proportional controllers.
     - Switches between "fix yaw," "go straight," and "done" states to ensure accurate navigation.

5. **`last_target`**
   - **Location**: `scripts/last_target.py`
   - **Functionality**:
     - Implements a service `/last_input` that returns the coordinates of the last target set by the user.
     - Retrieves the target from ROS parameters (`des_pos_x` and `des_pos_y`).

6. **`wall_follower_service`**
   - **Location**: `scripts/wall_follower_service.py`
   - **Functionality**:
     - Enables wall-following behavior to avoid obstacles.
     - Uses laser scan data to detect walls and navigates alongside them.
     - Switches between states: "find wall," "turn left," and "follow wall."

7. **`robot_control_interface.ipynb`**
   - **Location**: `notebooks/robot_control_interface.ipynb`
   - **Functionality**: 
     - Provides an interactive Jupyter Notebook interface for controlling the robot.
     - **Features**:
       - Assign or cancel goals for the robot.
       - Display the distance to the closest obstacle in real-time.
       - Real-time plot of the robot's position using `FuncAnimation`.
       - Real-time plot of the number of reached/not-reached targets using `FuncAnimation`.

## Installation
### Prerequisites
- ROS Noetic installed on your system.
- Actionlib and other dependencies installed:
    ```bash
    sudo apt-get install ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs  jupyter-notebook python3-ipywidgets
    
    pip3 install jupyros
    ```
- Build the package:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Usage
- Launch the simulation:
    ```bash
    roslaunch assignment2_part1_rt assignment1.launch
    ```

### Jupyter Notebook Interface
1. Source the workspace in another terminal  
   ```bash
   source ~/catkin_ws/devel/setup.bash
   cd ~/catkin_ws/src/assignment2_part1_rt/notebooks
   jupyter notebook robot_control_interface.ipynb
   ```
2. Open the notebook in your browser and run each cell in turn.

### Notebook Features
- **Goal Management**:
  - Assign a new goal to the robot by specifying x and y coordinates.
  - Cancel the current goal if needed.
- **Obstacle Information**:
  - Display the distance to the closest obstacle in real-time.
- **Real-Time Plots**:
  - **Robot Position**: A real-time plot of the robot's position and path using `FuncAnimation`.
  - **Target Statistics**: A real-time plot showing the number of reached and not-reached targets using `FuncAnimation`.

## User Instructions
- Action Client:
    - Set a new goal or cancel the current goal through console commands.
    - Tracks feedback such as position and velocity.

- Bug Action Server:
    - Handles navigation towards targets while avoiding obstacles.
    - Automatically switches to wall-following mode if an obstacle is detected.

- Distance Average Server:
    - Calculates and provides the distance to the goal and average speeds upon request.

- Go To Point Service:
    - Moves the robot to the target point with precision control.

- Last Target Service:
    - Returns the last specified target coordinates.

- Wall Follower Service:
    - Enables obstacle avoidance by following walls when obstacles are detected.

- **Jupyter Notebook**:
    - Provides an interactive interface for controlling the robot and visualizing its behavior in real-time.

## File Structure
    assignment2_part1_rt
    ├── action/
    │   ├── Planning.action
    ├── config/
    │   ├── sim.rviz
    │   ├── sim2.rviz
    ├── launch/
    │   ├── assignment1.launch
    │   ├── sim_w1.launch
    ├── msg/
    │   ├── Status.msg
    ├── scripts/
    │   ├── action_client.py
    │   ├── bug_as.py
    │   ├── dis_avg.py
    │   ├── go_to_point_service.py
    │   ├── last_target.py
    │   ├── wall_follower_service.py
    ├── srv/
    │   ├── DisAvg.srv
    │   ├── LastInput.srv
    ├── urdf/
    │   ├── robot2_laser.gazebo
    │   ├── robot2_laser.xacro
    ├── world/
    │   ├── assignment.world
    ├── notebooks/
    │   ├── robot_control_interface.ipynb
    ├── CMakeLists.txt
    ├── package.xml

## Topics 
- **Published**:
    - `/cmd_vel` (`geometry_msgs/Twist`): Robot velocity commands.
    - `/status` (`assignment2_part1_rt/Status`): Current position and velocity.

- **Subscribed**:
    - `/odom` (`nav_msgs/Odometry`): Odometry data for position and orientation.
    - `/scan` (`sensor_msgs/LaserScan`): Laser scan data for obstacle detection.

## Services
- **`/dist_avg`**:
    - Request: None.
    - Response: Distance to the target, average linear and angular velocities.

- **`/last_input`**:
    - Request: None.
    - Response: Last target coordinates.

- **`/go_to_point_switch`**:
    - Enables or disables the go-to-point behavior.

- **`/wall_follower_switch`**:
    - Enables or disables wall-following behavior.

