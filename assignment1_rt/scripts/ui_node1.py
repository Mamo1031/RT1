#!/usr/bin/env python3
"""Provides a ROS node for spawning and controlling turtles in turtlesim."""

import rospy
from turtlesim.srv import Spawn, SpawnRequest
from geometry_msgs.msg import Twist
import sys
from typing import Optional, Tuple


def spawn_turtle(name: str, x: float, y: float, theta: float) -> None:
    """Call the spawn service to create a new turtle.

    Args:
        name (str): Name of the turtle to spawn.
        x (float): X-coordinate of the spawn location.
        y (float): Y-coordinate of the spawn location.
        theta (float): Initial orientation of the turtle in radians.

    Raises:
        SystemExit: If the spawn service call fails.

    """
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

    Returns:
        tuple: A tuple containing:
            - (str): The selected turtle name ("turtle1" or "turtle2").
            - (float): The linear velocity.
            - (float): The angular velocity.
            
        Returns (None, None, None) if the input is invalid.
    """
    choice = input("Which turtle to control? (1 for turtle1, 2 for turtle2): ")
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


def send_command(pub: rospy.Publisher, vel: float, ang: float) -> None:
    """Send velocity command to the turtle.

    Args:
        pub (rospy.Publisher): The publisher object for sending commands to the turtle.
        vel (float): Linear velocity.
        ang (float): Angular velocity.

    """
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
    """Initialize the ROS node and handle user input for controlling turtles.

    This function sets up the ROS node, spawns a second turtle, and continuously
    accepts user input to control the movement of either turtle1 or turtle2.
    """
    rospy.init_node("ui_node", anonymous=True)

    # Turtle spawning
    spawn_turtle("turtle2", 3.0, 3.0, 0.0)

    # Creating Publishers
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)

    rospy.loginfo("User interface is ready. Enter commands:")

    while not rospy.is_shutdown():
        try:
            turtle, vel, ang = get_user_input()
            if turtle is None:
                continue

            if turtle == "turtle1":
                send_command(pub_turtle1, vel, ang)
            elif turtle == "turtle2":
                send_command(pub_turtle2, vel, ang)

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down UI node.")
            break
        except KeyboardInterrupt:
            print("Exiting...")
            break


if __name__ == "__main__":
    main()

