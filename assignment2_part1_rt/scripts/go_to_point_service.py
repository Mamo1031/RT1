#! /usr/bin/env python3

"""
Go to Point Service Module

This module implements a service that allows a robot to navigate to a specified point in space.
It handles the movement control by breaking down the navigation task into three states:
1. Aligning the robot's orientation towards the target (fix_yaw)
2. Moving forward to the target (go_straight_ahead)
3. Stopping when the target is reached (done)

The module subscribes to odometry data and publishes velocity commands to navigate the robot.
"""

# Import ROS libraries and messages
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import math

# Global variables
active_: bool = False  # Whether the robot is active or not

# Robot state variables
position_: Point = Point()  # Current position of the robot
yaw_: float = 0.0  # Current orientation (yaw) of the robot

# Machine state variables
state_: int = 0  # Current state of the robot
# Goal position
desired_position_: Point = Point()
desired_position_.x = rospy.get_param("des_pos_x")
desired_position_.y = rospy.get_param("des_pos_y")
desired_position_.z = 0

# Control parameters
yaw_precision_: float = math.pi / 9  # Allowed yaw error (±20 degrees)
yaw_precision_2_: float = math.pi / 90  # Allowed yaw error (±2 degrees)
dist_precision_: float = 0.3  # Allowed distance error

kp_a: float = 3.0  # Angular velocity proportional gain
kp_d: float = 0.2  # Linear velocity proportional gain
ub_a: float = 0.6  # Upper bound for angular velocity
lb_a: float = -0.5  # Lower bound for angular velocity
ub_d: float = 0.6  # Upper bound for linear velocity

# Publisher for velocity commands
pub: rospy.Publisher = None


def go_to_point_switch(req: SetBoolRequest) -> SetBoolResponse:
    """
    Callback function to enable or disable the robot's navigation behavior.

    This service allows external nodes to activate or deactivate the robot's
    navigation functionality.

    Args:
        req (SetBoolRequest): Service request containing a boolean data field
                             to enable (True) or disable (False) navigation.

    Returns:
        SetBoolResponse: Response containing:
            - success (bool): Always True to indicate the service call was processed
            - message (str): Confirmation message "Done!"
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = "Done!"
    return res


def clbk_odom(msg: Odometry) -> None:
    """
    Callback function to update the robot's current position and orientation.

    This function is called whenever new odometry data is received. It extracts
    the position and yaw (orientation) from the odometry message and updates
    the global variables accordingly.

    Args:
        msg (Odometry): Message containing odometry data with position and orientation.

    Returns:
        None
    """
    global position_, yaw_

    # Update position
    position_ = msg.pose.pose.position

    # Update orientation (yaw)
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state: int) -> None:
    """
    Change the robot's current state in the navigation state machine.

    The robot's navigation behavior is implemented as a state machine with the
    following states:
    - 0: Fix yaw (align orientation)
    - 1: Go straight ahead (move towards target)
    - 2: Done (target reached)

    Args:
        state (int): New state to set (0, 1, or 2).

    Returns:
        None
    """
    global state_
    state_ = state
    print("State changed to [%s]" % state_)


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi].

    This function ensures that all angle calculations are performed within
    the standard range of [-π, π] radians.

    Args:
        angle (float): The angle to normalize, in radians.

    Returns:
        float: Normalized angle in the range [-π, π].
    """
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos: Point) -> None:
    """
    Align the robot's orientation (yaw) towards the target position.

    This function is called in state 0 of the navigation state machine.
    It calculates the desired yaw angle based on the target position,
    then applies appropriate angular velocity to minimize the yaw error.

    Args:
        des_pos (Point): Desired position to align to.

    Returns:
        None: Changes state to 1 when alignment is achieved.
    """
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a * err_yaw
        twist_msg.angular.z = max(min(twist_msg.angular.z, ub_a), lb_a)

    pub.publish(twist_msg)

    if math.fabs(err_yaw) <= yaw_precision_2_:
        print("Yaw error: [%s]" % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos: Point) -> None:
    """
    Move the robot straight towards the target position.

    This function is called in state 1 of the navigation state machine.
    It calculates the linear velocity based on the distance to the target
    and adjusts the angular velocity to maintain the correct heading.

    Args:
        des_pos (Point): Desired position to move towards.

    Returns:
        None: Changes state to 2 when position is reached or to 0 if
              alignment is lost.
    """
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(
        pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2)
    )

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = min(kp_d * err_pos, ub_d)
        twist_msg.angular.z = kp_a * err_yaw
        pub.publish(twist_msg)
    else:
        print("Position error: [%s]" % err_pos)
        change_state(2)

    if math.fabs(err_yaw) > yaw_precision_:
        print("Yaw error: [%s]" % err_yaw)
        change_state(0)


def done() -> None:
    """
    Stop the robot by setting linear and angular velocities to zero.

    This function is called in state 2 of the navigation state machine
    when the robot has reached its target position.

    Returns:
        None
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def main() -> None:
    """
    Main function to initialize the ROS node and control the robot's behavior.

    This function:
    1. Initializes the ROS node
    2. Sets up publishers, subscribers, and services
    3. Implements the main control loop that drives the robot's behavior based on its current state

    Returns:
        None
    """
    global pub, active_

    rospy.init_node("go_to_point")

    # Initialize publisher and subscriber
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, clbk_odom)

    # Initialize service
    rospy.Service("go_to_point_switch", SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            # Update goal position
            desired_position_.x = rospy.get_param("des_pos_x")
            desired_position_.y = rospy.get_param("des_pos_y")
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr("Unknown state!")

        rate.sleep()


if __name__ == "__main__":
    main()
