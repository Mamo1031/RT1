#!/usr/bin/env python3
"""ROS node to monitor distance between turtles and enforce boundary constraints, with dynamic threshold update."""

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import sqrt
from typing import Optional
from assignment1_rt.srv import (
    SetThreshold,
    SetThresholdResponse,
)  # New service type for threshold update


class DistanceMonitor:
    def __init__(self) -> None:
        rospy.init_node("distance_node2", anonymous=True)

        # Thresholds (distance_threshold is now dynamic)
        self.distance_threshold: float = 2.0  # initial threshold value
        self.boundary_min: float = 1.0
        self.boundary_max: float = 10.0

        # Turtles' positions
        self.turtle1_pose: Optional[Pose] = None
        self.turtle2_pose: Optional[Pose] = None

        # Publishers
        self.distance_pub = rospy.Publisher("/turtles_distance", Float32, queue_size=10)
        self.stop_pub_turtle1 = rospy.Publisher(
            "/turtle1/cmd_vel", Twist, queue_size=10
        )
        self.stop_pub_turtle2 = rospy.Publisher(
            "/turtle2/cmd_vel", Twist, queue_size=10
        )

        # Subscribers
        rospy.Subscriber("/turtle1/pose", Pose, self.turtle1_pose_callback)
        rospy.Subscriber("/turtle2/pose", Pose, self.turtle2_pose_callback)

        # New service server for updating threshold asynchronously
        self.threshold_service = rospy.Service(
            "/set_distance_threshold", SetThreshold, self.set_threshold_callback
        )

    def turtle1_pose_callback(self, msg: Pose) -> None:
        self.turtle1_pose = msg

    def turtle2_pose_callback(self, msg: Pose) -> None:
        self.turtle2_pose = msg

    def calculate_distance(self) -> float:
        if self.turtle1_pose and self.turtle2_pose:
            dx = self.turtle1_pose.x - self.turtle2_pose.x
            dy = self.turtle1_pose.y - self.turtle2_pose.y
            return sqrt(dx**2 + dy**2)
        return float("inf")

    def enforce_constraints(self) -> None:
        if not self.turtle1_pose or not self.turtle2_pose:
            return

        distance = self.calculate_distance()
        rospy.loginfo(f"Distance between turtles: {distance:.2f}")
        self.distance_pub.publish(Float32(data=distance))

        if distance < self.distance_threshold:
            rospy.logwarn("Turtles are too close! Stopping both turtles.")
            self.stop_turtles()

        if self.is_near_boundary(self.turtle1_pose):
            rospy.logwarn("Turtle1 is near the boundary! Stopping turtle1.")
            self.stop_turtle(self.stop_pub_turtle1)

        if self.is_near_boundary(self.turtle2_pose):
            rospy.logwarn("Turtle2 is near the boundary! Stopping turtle2.")
            self.stop_turtle(self.stop_pub_turtle2)

    def is_near_boundary(self, pose: Pose) -> bool:
        return (
            pose.x < self.boundary_min
            or pose.x > self.boundary_max
            or pose.y < self.boundary_min
            or pose.y > self.boundary_max
        )

    def stop_turtles(self) -> None:
        self.stop_turtle(self.stop_pub_turtle1)
        self.stop_turtle(self.stop_pub_turtle2)

    def stop_turtle(self, pub: rospy.Publisher) -> None:
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        pub.publish(stop_cmd)

    def set_threshold_callback(
        self, req: SetThreshold.Request
    ) -> SetThreshold.Response:
        self.distance_threshold = req.threshold
        rospy.loginfo(f"Distance threshold updated to: {req.threshold}")
        return SetThresholdResponse(
            success=True, message="Threshold updated successfully."
        )

    def run(self) -> None:
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

