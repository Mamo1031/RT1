#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *

from typing import Dict

# Global variables
active_: bool = False  # Indicates if the wall follower is active
pub_: rospy.Publisher  # Publisher for velocity commands
regions_: Dict[str, float] = {  # Stores laser scan data divided into regions
    "right": 0,
    "fright": 0,
    "front": 0,
    "fleft": 0,
    "left": 0,
}
state_: int = 0  # Current state of the robot
state_dict_: Dict[int, str] = {  # Mapping states to descriptions
    0: "find the wall",
    1: "turn left",
    2: "follow the wall",
}


def wall_follower_switch(req: SetBool) -> SetBoolResponse:
    """
    Callback function for the wall_follower_switch service.

    Args:
        req (SetBool): Service request containing a boolean to activate/deactivate the wall follower.

    Returns:
        SetBoolResponse: Response indicating the success and status message.
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = "Done!"
    return res


def clbk_laser(msg: LaserScan) -> None:
    """
    Callback function for processing laser scan data.

    Args:
        msg (LaserScan): Laser scan data from the robot's sensors.

    Updates:
        - Populates the 'regions_' dictionary with minimum distances detected in each region.
    """
    global regions_
    regions_ = {
        "right": min(min(msg.ranges[0:143]), 10),
        "fright": min(min(msg.ranges[144:287]), 10),
        "front": min(min(msg.ranges[288:431]), 10),
        "fleft": min(min(msg.ranges[432:575]), 10),
        "left": min(min(msg.ranges[576:713]), 10),
    }
    take_action()


def change_state(state: int) -> None:
    """
    Changes the robot's current state and logs the change.

    Args:
        state (int): The new state to set (0 - find the wall, 1 - turn left, 2 - follow the wall).
    """
    global state_, state_dict_
    if state != state_:
        rospy.loginfo(f"Wall follower - [{state}] - {state_dict_[state]}")
        state_ = state


def take_action() -> None:
    """
    Decides the next action based on the laser scan data and current state.

    Updates:
        - Changes the robot's state based on obstacle proximity.
    """
    global regions_
    regions = regions_
    d0 = 1.0  # Threshold for detecting obstacles
    d = 1.5  # Safe distance threshold

    # State determination based on distances in regions
    if regions["front"] > d0 and regions["fleft"] > d and regions["fright"] > d:
        change_state(0)  # Find the wall
    elif regions["front"] < d0:
        change_state(1)  # Turn left
    elif regions["fright"] < d:
        change_state(2)  # Follow the wall


def find_wall() -> Twist:
    """
    Generates a Twist message to move forward and slightly to the right.

    Returns:
        Twist: Velocity command to find the wall.
    """
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left() -> Twist:
    """
    Generates a Twist message to turn the robot to the left.

    Returns:
        Twist: Velocity command to turn left.
    """
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall() -> Twist:
    """
    Generates a Twist message to move the robot along the wall.

    Returns:
        Twist: Velocity command to follow the wall.
    """
    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main() -> None:
    """
    Main function to initialize the ROS node and execute the wall-following behavior.

    - Sets up publishers, subscribers, and services.
    - Controls the robot based on the current state and sensor data.
    """
    global pub_, active_

    # Initialize the ROS node
    rospy.init_node("reading_laser")

    # Publisher for velocity commands
    pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Subscriber to laser scan data
    rospy.Subscriber("/scan", LaserScan, clbk_laser)

    # Service to enable or disable wall following
    rospy.Service("wall_follower_switch", SetBool, wall_follower_switch)

    # Control loop
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        # Generate velocity commands based on the current state
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        else:
            rospy.logerr("Unknown state!")

        # Publish the velocity command
        pub_.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()

