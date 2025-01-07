#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment2_part1_rt.msg
from tf import transformations
from std_srvs.srv import *
import time
from typing import Optional, Dict

# Global variables
srv_client_go_to_point_: Optional[rospy.ServiceProxy] = (
    None  # Service client for go-to-point behavior
)
srv_client_wall_follower_: Optional[rospy.ServiceProxy] = (
    None  # Service client for wall-following behavior
)
yaw_: float = 0.0  # Current yaw angle of the robot
yaw_error_allowed_: float = 5 * (math.pi / 180)  # Allowable yaw error in radians
position_: Point = Point()  # Current position of the robot
pose_: Pose = Pose()  # Current pose of the robot
desired_position_: Point = Point()  # Desired target position
desired_position_.z = 0.0  # Fix z-coordinate to 0
regions_: Optional[Dict[str, float]] = None  # Stores the laser scan region distances
state_desc_: list = ["Go to point", "wall following", "done"]  # Descriptions of states
state_: int = 0  # Current state: 0 (go to point), 1 (wall following), 2 (done)

# Callbacks


def clbk_odom(msg: Odometry) -> None:
    """
    Callback function for the '/odom' topic subscriber.

    Args:
        msg (Odometry): Odometry message containing the robot's position and orientation.

    Updates:
        - Position and pose of the robot.
        - Yaw angle of the robot.
    """
    global position_, yaw_, pose_

    # Extract position and pose
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # Compute yaw from quaternion
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg: LaserScan) -> None:
    """
    Callback function for the '/scan' topic subscriber.

    Args:
        msg (LaserScan): Laser scan data providing distances to obstacles.

    Updates:
        - Distance measurements in predefined regions around the robot.
    """
    global regions_
    regions_ = {
        "right": min(min(msg.ranges[0:143]), 10),
        "fright": min(min(msg.ranges[144:287]), 10),
        "front": min(min(msg.ranges[288:431]), 10),
        "fleft": min(min(msg.ranges[432:575]), 10),
        "left": min(min(msg.ranges[576:719]), 10),
    }


def change_state(state: int) -> None:
    """
    Changes the robot's behavior state and activates the corresponding services.

    Args:
        state (int): The new state to switch to.
                     0 - Go to point
                     1 - Wall following
                     2 - Done
    """
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_

    state_ = state
    log = f"State changed: {state_desc_[state]}"
    rospy.loginfo(log)

    # Activate or deactivate services based on state
    if state_ == 0:
        srv_client_go_to_point_(True)
        srv_client_wall_follower_(False)
    elif state_ == 1:
        srv_client_go_to_point_(False)
        srv_client_wall_follower_(True)
    elif state_ == 2:
        srv_client_go_to_point_(False)
        srv_client_wall_follower_(False)


def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle to the range [-pi, pi].

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Normalized angle.
    """
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def done() -> None:
    """
    Stops the robot by publishing zero velocities.
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def planning(goal: assignment2_part1_rt.msg.PlanningGoal) -> None:
    """
    Executes the planning to move the robot towards the target position.

    Args:
        goal (PlanningGoal): Goal containing the target position.

    Manages the robot's movement while checking its state and updating feedback.
    """
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_

    # Initialize state
    change_state(0)
    rate = rospy.Rate(20)
    success = True

    # Set desired position
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param("des_pos_x", desired_position_.x)
    rospy.set_param("des_pos_y", desired_position_.y)

    # Feedback and result
    feedback = assignment2_part1_rt.msg.PlanningFeedback()
    result = assignment2_part1_rt.msg.PlanningResult()

    while not rospy.is_shutdown():
        # Compute position error
        err_pos = math.sqrt(
            pow(desired_position_.y - position_.y, 2)
            + pow(desired_position_.x - position_.x, 2)
        )

        # Handle goal preemption
        if act_s.is_preempt_requested():
            rospy.loginfo("Goal was preempted")
            feedback.stat = "Target cancelled!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            act_s.set_preempted()
            success = False
            change_state(2)
            done()
            break

        # Handle target reached
        elif err_pos < 0.5:
            change_state(2)
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break

        # Handle obstacle avoidance
        elif regions_ and regions_["front"] < 0.2:
            change_state(1)

        rate.sleep()

    # Final result
    if success:
        rospy.loginfo("Goal: Succeeded!")
        act_s.set_succeeded(result)


def main() -> None:
    """
    Main function to initialize the node and setup subscribers, publishers, and services.
    """
    time.sleep(2)
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node("bug0")

    desired_position_.x = 0.0
    desired_position_.y = 1.0
    rospy.set_param("des_pos_x", desired_position_.x)
    rospy.set_param("des_pos_y", desired_position_.y)

    rospy.Subscriber("/scan", LaserScan, clbk_laser)
    rospy.Subscriber("/odom", Odometry, clbk_odom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    srv_client_go_to_point_ = rospy.ServiceProxy("/go_to_point_switch", SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy("/wall_follower_switch", SetBool)

    act_s = actionlib.SimpleActionServer(
        "/reaching_goal",
        assignment2_part1_rt.msg.PlanningAction,
        planning,
        auto_start=False,
    )
    act_s.start()

    rospy.spin()


if __name__ == "__main__":
    main()
