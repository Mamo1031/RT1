#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

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
    """Service callback to activate or deactivate the wall follower.
    
    This function handles requests to turn the wall following behavior
    on or off by setting the global ``active_`` flag.
    
    :param req: Service request containing a boolean to activate/deactivate the wall follower
    :type req: SetBool
    :return: Response indicating success and a status message
    :rtype: SetBoolResponse
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = "Done!"
    return res


def clbk_laser(msg: LaserScan) -> None:
    """Process laser scan data and take appropriate action.
    
    This callback divides the laser scan into five regions and stores the minimum
    distance to obstacles in each region. It then decides the next action based on these values.
    
    :param msg: Laser scan data from the robot's sensors
    :type msg: LaserScan
    :return: None
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
    """Change the robot's current state and log the change.
    
    This function updates the robot's state if it differs from the current state
    and logs the change for debugging purposes.
    
    :param state: The new state to set (0: find the wall, 1: turn left, 2: follow the wall)
    :type state: int
    :return: None
    """
    global state_, state_dict_
    if state != state_:
        rospy.loginfo(f"Wall follower - [{state}] - {state_dict_[state]}")
        state_ = state


def take_action() -> None:
    """Determine and set the next robot state based on sensor data.
    
    This function analyzes the data in the global ``regions_`` dictionary to decide
    which state the robot should be in for effective wall following.
    
    :return: None
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
    """Generate velocity command to find a wall.
    
    Creates a Twist message that moves the robot forward while turning slightly
    to the right, helping it locate a wall to follow.
    
    :return: Velocity command to find the wall
    :rtype: Twist
    """
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left() -> Twist:
    """Generate velocity command to turn the robot left.
    
    Creates a Twist message that makes the robot turn to the left,
    typically used when an obstacle is detected in front.
    
    :return: Velocity command to turn left
    :rtype: Twist
    """
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall() -> Twist:
    """Generate velocity command to follow alongside a wall.
    
    Creates a Twist message that moves the robot forward at a moderate speed,
    maintaining its current distance from the wall.
    
    :return: Velocity command to follow the wall
    :rtype: Twist
    """
    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main() -> None:
    """Initialize and run the wall following node.
    
    This function sets up the ROS node, creates publishers, subscribers, and services,
    and contains the main control loop that executes the wall-following behavior.
    
    :return: None
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
