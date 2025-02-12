#!/usr/bin/env python3
"""
Modified action client node for the second assignment.

Added features:
- A service (/get_last_target_distance) that returns the distance from the robot's current position
  to the last target set (returns 0 if no target has been set), using DisAvg.srv.
- A feedback callback (for the action server) that prints "reached" when the target is reached.
- A publisher on the topic "warning" that publishes True if the distance of the closest obstacle is less than 1 m.
"""

import rospy
from nav_msgs.msg import Odometry
import actionlib
from assignment2_part1_rt.msg import PlanningAction, PlanningGoal, Status
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool, Float32

from assignment2_part1_rt.srv import DisAvg, DisAvgResponse

import sys
import threading
from math import sqrt

# Global variables
client = None  # Action client for sending goals
status_pub = None  # Publisher for robot status (custom message Status)
last_target = None  # Last target set as a tuple (x, y)
current_odom = None  # Latest odometry message
warning_pub = None  # Publisher for warning messages (Bool)


def odom_callback(msg: Odometry) -> None:
    global current_odom
    current_odom = msg


def feedback_callback(feedback):
    """
    Feedback callback for the action server.
    Assumes that the feedback message has a boolean field 'reached' that is True when the target is reached.
    """
    if hasattr(feedback, "reached") and feedback.reached:
        rospy.loginfo("reached")


def goal_done_callback(status, result):
    if status == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached! (done callback)")
    else:
        rospy.loginfo("Goal finished with status: %d", status)


def get_last_target_distance_callback(req):
    """
    Service callback that returns the Euclidean distance between the robot's current position (from /odom)
    and the last target set.
    Uses the DisAvg service response format. If no target has been set or if current odometry is unavailable,
    returns 0 for the distance (and 0 for the average velocities).
    """
    global current_odom, last_target
    if last_target is None or current_odom is None:
        return DisAvgResponse(d=0.0, avg_v_x=0.0, avg_v_z=0.0)
    dx = current_odom.pose.pose.position.x - last_target[0]
    dy = current_odom.pose.pose.position.y - last_target[1]
    distance = sqrt(dx**2 + dy**2)
    return DisAvgResponse(d=distance, avg_v_x=0.0, avg_v_z=0.0)


def obstacle_callback(msg: Float32):
    """
    Callback for the closest obstacle distance.
    Publishes a warning (True) on the "warning" topic if the distance is less than 1 meter,
    otherwise publishes False.
    """
    global warning_pub
    warn_msg = Bool()
    warn_msg.data = True if msg.data < 1.0 else False
    warning_pub.publish(warn_msg)


def action_client():
    """
    Implements the action client loop that allows the user to set new goals or cancel the current goal.
    Updates the global last_target variable when a new goal is sent.
    """
    global client, last_target
    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server ready!")

    while not rospy.is_shutdown():
        print("Insert 'c' to cancel the current goal, anything else to set a new goal:")
        command = input("Enter command: ").strip()
        if command == "c":
            if client.get_state() == GoalStatus.SUCCEEDED:
                print("You have just reached the goal; you cannot cancel it.")
            else:
                client.cancel_goal()
                rospy.loginfo("Goal cancelled")
            continue

        limit = 10  # Position limits
        try:
            x = float(input("Insert target x: "))
            y = float(input("Insert target y: "))
            if x < -limit or x > limit or y < -limit or y > limit:
                raise ValueError(f"Insert values between [{-limit}, {limit}]")
        except ValueError as e:
            rospy.loginfo(e)
            continue

        # Update the last target global variable
        last_target = (x, y)

        # Create and set the goal
        goal = PlanningGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"Sending goal: ({x}, {y})")
        client.send_goal(
            goal, feedback_cb=feedback_callback, done_cb=goal_done_callback
        )


def main():
    rospy.init_node("action_client_node")

    # Publisher for robot status (custom Status message)
    global status_pub
    status_pub = rospy.Publisher("/status", Status, queue_size=1)

    # Subscriber for odometry
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Service server for returning the distance to the last target using DisAvg.srv
    rospy.Service(
        "/get_last_target_distance", DisAvg, get_last_target_distance_callback
    )

    # Set up warning publisher on "warning" topic
    global warning_pub
    warning_pub = rospy.Publisher("warning", Bool, queue_size=10)

    # Subscriber for closest obstacle distance (assumed to be published on this topic)
    rospy.Subscriber("closest_obstacle_distance", Float32, obstacle_callback)

    # Start the action client in a separate thread to allow asynchronous command input.
    action_thread = threading.Thread(target=action_client)
    action_thread.start()

    rospy.spin()
    action_thread.join()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")

