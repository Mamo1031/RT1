#!/usr/bin/env python3
"""ROS node to monitor distance between turtles and enforce boundary constraints."""

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import sqrt
from typing import Optional


class DistanceMonitor:
    """Class to monitor the distance between turtles and enforce boundary constraints."""

    def __init__(self) -> None:
        """Initialize the node, publishers, and subscribers.

        Attributes:
            distance_threshold (float): Minimum allowable distance between turtles.
            boundary_min (float): Minimum allowable x or y coordinate.
            boundary_max (float): Maximum allowable x or y coordinate.
            turtle1_pose (Optional[Pose]): Current position of turtle1.
            turtle2_pose (Optional[Pose]): Current position of turtle2.

        """
        rospy.init_node("distance_node2", anonymous=True)

        # Thresholds
        self.distance_threshold: float = (
            2.0  # Minimum allowable distance between turtles
        )
        self.boundary_min: float = 1.0  # Minimum x or y value
        self.boundary_max: float = 10.0  # Maximum x or y value

        # Turtles' positions
        self.turtle1_pose: Optional[Pose] = None
        self.turtle2_pose: Optional[Pose] = None

        # Publishers
        self.distance_pub: rospy.Publisher = rospy.Publisher(
            "/turtles_distance", Float32, queue_size=10
        )
        self.stop_pub_turtle1: rospy.Publisher = rospy.Publisher(
            "/turtle1/cmd_vel", Twist, queue_size=10
        )
        self.stop_pub_turtle2: rospy.Publisher = rospy.Publisher(
            "/turtle2/cmd_vel", Twist, queue_size=10
        )

        # Subscribers
        rospy.Subscriber("/turtle1/pose", Pose, self.turtle1_pose_callback)
        rospy.Subscriber("/turtle2/pose", Pose, self.turtle2_pose_callback)

    def turtle1_pose_callback(self, msg: Pose) -> None:
        """Update turtle1's position.

        Args:
            msg (Pose): The current position of turtle1.

        """
        self.turtle1_pose = msg

    def turtle2_pose_callback(self, msg: Pose) -> None:
        """Update turtle2's position.

        Args:
            msg (Pose): The current position of turtle2.

        """
        self.turtle2_pose = msg

    def calculate_distance(self) -> float:
        """Calculate the Euclidean distance between turtle1 and turtle2.

        Returns:
            float: The distance between turtle1 and turtle2. Returns `inf` if either pose is unavailable.

        """
        if self.turtle1_pose and self.turtle2_pose:
            dx = self.turtle1_pose.x - self.turtle2_pose.x
            dy = self.turtle1_pose.y - self.turtle2_pose.y
            return sqrt(dx**2 + dy**2)
        return float("inf")

    def enforce_constraints(self) -> None:
        """Enforce distance and boundary constraints.

        - Stops turtles if the distance between them is below the threshold.
        - Stops turtles if they are near the defined boundaries.
        """
        if not self.turtle1_pose or not self.turtle2_pose:
            return

        # Calculate distance between turtles
        distance = self.calculate_distance()
        rospy.loginfo(f"Distance between turtles: {distance:.2f}")
        self.distance_pub.publish(Float32(data=distance))

        # Stop turtles if too close to each other
        if distance < self.distance_threshold:
            rospy.logwarn("Turtles are too close! Stopping both turtles.")
            self.stop_turtles()

        # Stop turtles if they are near the boundary
        if self.is_near_boundary(self.turtle1_pose):
            rospy.logwarn("Turtle1 is near the boundary! Stopping turtle1.")
            self.stop_turtle(self.stop_pub_turtle1)

        if self.is_near_boundary(self.turtle2_pose):
            rospy.logwarn("Turtle2 is near the boundary! Stopping turtle2.")
            self.stop_turtle(self.stop_pub_turtle2)

    def is_near_boundary(self, pose: Pose) -> bool:
        """Check if the turtle is near the boundary.

        Args:
            pose (Pose): The current position of the turtle.

        Returns:
            bool: True if the turtle is near the boundary, False otherwise.

        """
        return (
            pose.x < self.boundary_min
            or pose.x > self.boundary_max
            or pose.y < self.boundary_min
            or pose.y > self.boundary_max
        )

    def stop_turtles(self) -> None:
        """Publish stop commands to both turtles."""
        self.stop_turtle(self.stop_pub_turtle1)
        self.stop_turtle(self.stop_pub_turtle2)

    def stop_turtle(self, pub: rospy.Publisher) -> None:
        """Publish a stop command to a specific turtle.

        Args:
            pub (rospy.Publisher): The publisher object for sending stop commands.

        """
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        pub.publish(stop_cmd)

    def run(self) -> None:
        """Run the main loop to monitor and enforce constraints."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.enforce_constraints()
            rate.sleep()


if __name__ == "__main__":
    try:
        monitor = DistanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Distance monitor node terminated.")

