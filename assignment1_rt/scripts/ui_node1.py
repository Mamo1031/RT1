#!/usr/bin/env python3
"""Provides a ROS node for spawning and controlling turtles in turtlesim, with asynchronous threshold update."""

import rospy
from turtlesim.srv import Spawn, SpawnRequest
from geometry_msgs.msg import Twist
import sys
from typing import Optional, Tuple
from assignment1_rt.srv import (
    SetThreshold,
)  # Assuming the service is defined in this package


def spawn_turtle(name: str, x: float, y: float, theta: float) -> None:
    rospy.wait_for_service("/spawn")
    try:
        spawn_service = rospy.ServiceProxy("/spawn", Spawn)
        req = SpawnRequest()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = name
        spawn_service(req)
        rospy.loginfo(f"Spawned {name} at ({x}, {y}, {theta})")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {name}: {e}")
        sys.exit(1)


def get_user_input() -> Tuple[Optional[str], Optional[float], Optional[float]]:
    """Get user input for turtle selection and velocities.
    Returns None values if the user chooses to update threshold.
    """
    mode = input("Enter 'c' to control turtles or 't' to update threshold: ").strip()
    if mode == "t":
        return None, None, None  # Special signal for threshold update
    elif mode == "c":
        choice = input(
            "Which turtle to control? (1 for turtle1, 2 for turtle2): "
        ).strip()
        if choice == "1":
            turtle = "turtle1"
        elif choice == "2":
            turtle = "turtle2"
        else:
            print("Invalid choice. Please enter 1 or 2.")
            return None, None, None

        try:
            vel = float(input("Enter linear velocity: "))
            ang = float(input("Enter angular velocity: "))
        except ValueError:
            print("Invalid input. Please enter numeric values for velocity.")
            return None, None, None

        return turtle, vel, ang
    else:
        print("Invalid mode. Please try again.")
        return None, None, None


def update_threshold():
    """Function to update the distance threshold asynchronously."""
    try:
        new_threshold = float(input("Enter new distance threshold: "))
    except ValueError:
        print("Invalid input. Please enter a numeric value.")
        return
    rospy.wait_for_service("/set_distance_threshold")
    try:
        set_threshold_client = rospy.ServiceProxy(
            "/set_distance_threshold", SetThreshold
        )
        response = set_threshold_client(new_threshold)
        rospy.loginfo(f"Threshold update response: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def send_command(pub: rospy.Publisher, vel: float, ang: float) -> None:
    cmd = Twist()
    cmd.linear.x = vel
    cmd.angular.z = ang
    pub.publish(cmd)
    rospy.loginfo(f"Command sent: linear={vel}, angular={ang}")
    rospy.sleep(1)
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
    rospy.loginfo("Turtle stopped.")


def main() -> None:
    rospy.init_node("ui_node", anonymous=True)

    # Turtle spawning
    spawn_turtle("turtle2", 3.0, 3.0, 0.0)

    # Creating Publishers for turtle control
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)

    rospy.loginfo("User interface is ready. Enter commands:")

    while not rospy.is_shutdown():
        turtle, vel, ang = get_user_input()
        # If turtle is None, it means user wants to update threshold
        if turtle is None:
            update_threshold()
            continue

        if turtle == "turtle1":
            send_command(pub_turtle1, vel, ang)
        elif turtle == "turtle2":
            send_command(pub_turtle2, vel, ang)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down UI node.")

