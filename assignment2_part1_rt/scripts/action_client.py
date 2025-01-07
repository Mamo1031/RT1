#! /usr/bin/env python3

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
    Callback function for the '/odom' topic subscriber.

    Args:
        msg (Odometry): The current odometry message containing position and velocity data.

    Publishes:
        Status: Custom status message containing robot's position (x, y) and velocities (v_x, v_z).
    """
    status = Status()
    status.x = msg.pose.pose.position.x
    status.y = msg.pose.pose.position.y
    status.v_x = msg.twist.twist.linear.x
    status.v_z = msg.twist.twist.angular.z
    pub.publish(status)


def goalReached() -> bool:
    """
    Checks whether the goal has been reached by the robot.

    Returns:
        bool: True if the goal is reached, False otherwise.
    """
    return client.get_state() == GoalStatus.SUCCEEDED


def action_client() -> None:
    """
    Implements an action client to allow the user to set or cancel goals.

    Features:
        - Users can set new goals by specifying x and y coordinates.
        - Users can cancel the current goal if it is not already reached.
        - Goals are constrained within [-10, 10] for both x and y.
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
    Main function to initialize the ROS node and set up publishers and subscribers.

    Initializes:
        - ROS node named 'action_client'
        - Publisher '/status' for publishing robot status
        - Subscriber '/odom' for receiving odometry data
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

