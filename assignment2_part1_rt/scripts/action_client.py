#! /usr/bin/env python3

"""
Action client module for robot navigation.

This module implements an action client that allows users to set target positions
for a robot or cancel current goals. It also publishes the robot's current status
including position and velocity.

ROS Components:
    - Node: action_client
    - Subscriber: /odom
    - Publisher: /status
    - Action Client: /reaching_goal
"""

import rospy
from nav_msgs.msg import Odometry
import actionlib
from assignment2_part1_rt.msg import PlanningAction, PlanningGoal, Status
from std_srvs.srv import *
from actionlib_msgs.msg import GoalStatus


# Global variables
client: actionlib.SimpleActionClient  # Action client for sending goals
pub: rospy.Publisher  # Publisher for robot status


def callback(msg: Odometry) -> None:
    """
    Process odometry data and publish robot status.
    
    This callback function extracts position and velocity information from 
    odometry messages and publishes them as Status messages.
    
    :param msg: The current odometry message containing position and velocity data
    :type msg: nav_msgs.msg.Odometry
    
    :return: None
    :rtype: None
    
    :publishes: Status message with robot's position (x, y) and velocities (v_x, v_z)
    """
    status = Status()
    status.x = msg.pose.pose.position.x
    status.y = msg.pose.pose.position.y
    status.v_x = msg.twist.twist.linear.x
    status.v_z = msg.twist.twist.angular.z
    pub.publish(status)


def goalReached() -> bool:
    """
    Check if the goal has been reached.
    
    This function checks the state of the action client to determine whether
    the robot has successfully reached its goal.
    
    :return: True if the goal is reached, False otherwise
    :rtype: bool
    """
    return client.get_state() == GoalStatus.SUCCEEDED


def action_client() -> None:
    """
    Action client for setting and canceling goals.
    
    This function allows users to set new goals by specifying x and y coordinates
    or cancel the current goal if it is not already reached. Goals are constrained
    within [-10, 10] for both x and y coordinates.
    
    :return: None
    :rtype: None
    """
    global client
    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)

    # Wait for the action server to be available
    client.wait_for_server()

    while not rospy.is_shutdown():
        print("Insert 'c' to cancel the current goal, anything else to set a goal:")
        command = input("Enter command: ")

        # Cancel the current goal
        if command == "c":
            if goalReached():
                print("You have just reached the goal, you cannot cancel it")
            else:
                client.cancel_goal()
                rospy.loginfo("Goal cancelled")
            continue

        limit = 10  # Position limits

        # Get user input for goal position
        try:
            x = float(input("Insert x: "))
            y = float(input("Insert y: "))

            # Validate input values
            if x < -limit or x > limit or y < -limit or y > limit:
                raise ValueError(f"Insert values between [{-limit}, {limit}]")

        except ValueError as e:
            rospy.loginfo(e)
            continue

        # Create and set goal
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Update ROS parameters for desired position
        rospy.set_param("des_pos_x", x)
        rospy.set_param("des_pos_y", y)

        # Send goal to action server
        client.send_goal(goal)


def main() -> None:
    """
    Initialize the ROS node and set up publishers and subscribers.
    
    This function initializes the ROS node named 'action_client', sets up a publisher
    for robot status, and subscribes to the '/odom' topic to receive odometry data.
    
    :return: None
    :rtype: None
    """
    # Initialize ROS node
    rospy.init_node("action_client")

    # Define publisher for status updates
    global pub
    pub = rospy.Publisher("/status", Status, queue_size=1)

    # Subscribe to the '/odom' topic
    rospy.Subscriber("/odom", Odometry, callback)

    # Start action client
    action_client()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
