#! /usr/bin/env python3

"""
Last Target Service Node

This module provides a ROS service that returns the last target coordinates 
set as ROS parameters. It can be used to retrieve the most recent goal 
position sent to the robot.
"""

import rospy
import assignment2_part1_rt.srv
import assignment2_part1_rt.msg
from assignment2_part1_rt.srv import LastInput, LastInputResponse
from typing import Union


def callback(msg: assignment2_part1_rt.msg.PlanningActionGoal) -> LastInputResponse:
    """
    Callback function for the 'last_input' service.

    This function retrieves the last target coordinates (x, y) set as ROS parameters
    and returns them as a service response.

    :param msg: The service request message, though not used in the current implementation
    :type msg: assignment2_part1_rt.msg.PlanningActionGoal
    :return: A response object containing the last target coordinates
    :rtype: LastInputResponse
    """
    # Retrieve x, y coordinates from ROS parameters
    x: Union[int, float] = rospy.get_param("des_pos_x")
    y: Union[int, float] = rospy.get_param("des_pos_y")

    # Return the coordinates as a service response
    return LastInputResponse(x, y)


def main() -> None:
    """
    Main function for the 'last_input_srv' node.

    This function initializes the ROS node, creates a service to handle requests for
    the last target input, and keeps the node running until shut down.

    :raises: rospy.ROSInterruptException: If the ROS node is interrupted before completion
    """
    # Initialize the ROS node with the name 'last_input_srv'
    rospy.init_node("last_input_srv")

    # Create a service named 'last_input' using the LastInput service type and 'callback' function
    rospy.Service("last_input", LastInput, callback)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    try:
        # Execute the main function
        main()
    except rospy.ROSInterruptException:
        # Handle interruption (e.g., Ctrl+C) and print an error message
        print("Program interrupted before completion")

