#! /usr/bin/env python3

import rospy
from assignment2_part1_rt.msg import Status
from assignment2_part1_rt.srv import DisAvg, DisAvgResponse
import sys
import math
from typing import List

# Global variables for robot position and velocities
x: float = 0.0  # Current x-coordinate of the robot
y: float = 0.0  # Current y-coordinate of the robot
v_x_list: List[float] = []  # List to store recent x-axis velocities
v_z_list: List[float] = []  # List to store recent z-axis velocities


def distAvgCallback(msg: DisAvg) -> DisAvgResponse:
    """
    Service callback function to compute the robot's distance from the target
    and calculate the average linear and angular velocities.

    Args:
        msg (DisAvg): Service request (not used directly in this implementation).

    Returns:
        DisAvgResponse: Response containing:
            - distance (float): Distance between the robot and target.
            - avg_v_x (float): Average linear velocity along the x-axis.
            - avg_v_z (float): Average angular velocity around the z-axis.
    """
    # Retrieve goal coordinates from parameters
    des_x: float = rospy.get_param("des_pos_x")
    des_y: float = rospy.get_param("des_pos_y")

    # Calculate the Euclidean distance to the target
    distance: float = math.sqrt(pow(des_x - x, 2) + pow(des_y - y, 2))

    # Compute average velocities
    avg_v_x: float = sum(v_x_list) / len(v_x_list) if v_x_list else 0.0
    avg_v_z: float = sum(v_z_list) / len(v_z_list) if v_z_list else 0.0

    # Return the computed values as a response
    return DisAvgResponse(distance, avg_v_x, avg_v_z)


def subCallback(msg: Status) -> None:
    """
    Subscriber callback function for the '/status' topic.
    Updates the robot's position and maintains a sliding window of recent velocities.

    Args:
        msg (Status): Incoming message containing the robot's position and velocities.
    """
    global x, y

    # Update robot position
    x = msg.x
    y = msg.y

    # Append new velocity values to the lists
    v_x_list.append(msg.v_x)
    v_z_list.append(msg.v_z)

    # Get the sliding window size parameter
    window_size: int = rospy.get_param("/window_size")

    # Maintain the size of the sliding window
    if len(v_x_list) > window_size:
        v_x_list.pop(0)  # Remove oldest x-axis velocity
        v_z_list.pop(0)  # Remove oldest z-axis velocity


def main() -> None:
    """
    Main function to initialize the ROS node, set up the subscriber and service,
    and keep the server running until shutdown.
    """
    # Initialize the ROS node
    rospy.init_node("dist_avg_srv")

    # Subscribe to the '/status' topic
    rospy.Subscriber("/status", Status, subCallback)

    # Advertise the 'dist_avg' service
    rospy.Service("dist_avg", DisAvg, distAvgCallback)

    # Keep the service running
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        # Handle program interruption gracefully
        print("Program interrupted before completion", file=sys.stderr)

