"""
Launch file for setting up a Gazebo simulation environment.

This script performs the following tasks:
1. Loads and publishes the robot state using a URDF description.
2. Publishes joint states for the robot.
3. Spawns the robot entity in Gazebo at coordinates (2.0, 2.0).
4. Launches a custom node to control the robot's movement.
5. Launches Gazebo and RViz2 for visualization and simulation.

Usage
-----
.. code-block:: bash

   ros2 launch assignment2_part2_rt gazebo.launch.py

Parameters
----------
model : str, optional
    Path to the robot model URDF/XACRO file.
    Default: package's urdf/robot4.xacro

Dependencies
-----------
* ROS 2 Foxy or later
* Gazebo and RViz2
* Package 'assignment2_part2_rt' with necessary URDF, config, and scripts
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for setting up the simulation.

    This function sets up the robot description, publishers, Gazebo environment,
    and visualization tools required for the simulation.

    Components
    ----------
    1. Robot state publisher - Broadcasts the robot model
    2. Joint state publisher - Publishes joint states
    3. Spawn entity - Places the robot in Gazebo
    4. Move robot node - Controls robot movement
    5. Gazebo - Simulation environment
    6. RViz2 - Visualization tool

    Returns
    -------
    LaunchDescription
        A complete launch description containing all nodes and processes 
        needed for the simulation environment.
    """
    # Define paths for robot description and RViz configuration
    test_robot_description_share: str = FindPackageShare(
        package="assignment2_part2_rt"
    ).find("assignment2_part2_rt")

    default_model_path: str = os.path.join(
        test_robot_description_share, "urdf/robot4.xacro"
    )
    rviz_config_path: str = os.path.join(
        test_robot_description_share, "config/rviz.rviz"
    )

    # Node to publish the robot's state using the URDF description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    # Node to publish joint states for the robot
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # Node to spawn the robot entity in Gazebo at position (2.0, 2.0)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "my_test_robot",
            "-topic",
            "/robot_description",
            "-x",
            "2.0",
            "-y",
            "2.0",
        ],
        output="screen",
    )

    # Node to move the robot using custom commands
    move_robot_node = Node(
        package="assignment2_part2_rt",
        executable="move_robot_node",
        name="move_robot_node",
    )

    # Create and return the launch description
    return LaunchDescription(
        [
            # Declare the robot model path argument
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot URDF file",
            ),
            # Add nodes to the launch description
            robot_state_publisher_node,
            joint_state_publisher_node,
            spawn_entity,
            move_robot_node,
            # Launch Gazebo simulator
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "worlds/empty.world",
                    "-s",
                    "libgazebo_ros_factory.so",
                ],
                output="screen",
            ),
            # Launch RViz2 for visualization
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config_path], output="screen"),
        ]
    )

