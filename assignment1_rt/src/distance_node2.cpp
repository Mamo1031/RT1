#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>

// Global Variables
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
bool turtle1_pose_received = false;
bool turtle2_pose_received = false;

const float DISTANCE_THRESHOLD = 2.0;  // Minimum Distance Threshold
const float BOUNDARY_MIN = 1.0;        // Boundary minimum
const float BOUNDARY_MAX = 10.0;       // Boundary maximum

// A callback function that receives turtle 1's pose.
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1_pose = *msg;
    turtle1_pose_received = true;
}

// A callback function that receives the pose of Turtle 2.
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle2_pose = *msg;
    turtle2_pose_received = true;
}

// Function to calculate Euclidean distance
float calculateDistance(const turtlesim::Pose &pose1, const turtlesim::Pose &pose2) {
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// A function to determine whether or not the object is within the boundary
bool isNearBoundary(const turtlesim::Pose &pose) {
    return pose.x < BOUNDARY_MIN || pose.x > BOUNDARY_MAX || pose.y < BOUNDARY_MIN || pose.y > BOUNDARY_MAX;
}

// A function to stop a specific turtle
void stopTurtle(ros::Publisher &pub) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    pub.publish(stop_cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_node2");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/turtles_distance", 10);
    ros::Publisher stop_pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher stop_pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Subscriber
    ros::Subscriber turtle1_sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();

        if (turtle1_pose_received && turtle2_pose_received) {
            // Calculate distance
            float distance = calculateDistance(turtle1_pose, turtle2_pose);
            ROS_INFO("Distance between turtles: %.2f", distance);

            // Publish Distance
            std_msgs::Float32 distance_msg;
            distance_msg.data = distance;
            distance_pub.publish(distance_msg);

            // Stop if turtles are too close
            if (distance < DISTANCE_THRESHOLD) {
                ROS_WARN("Turtles are too close! Stopping both turtles.");
                stopTurtle(stop_pub_turtle1);
                stopTurtle(stop_pub_turtle2);
            }

            // Stop if turtle is close to boundary
            if (isNearBoundary(turtle1_pose)) {
                ROS_WARN("Turtle1 is near the boundary! Stopping turtle1.");
                stopTurtle(stop_pub_turtle1);
            }

            if (isNearBoundary(turtle2_pose)) {
                ROS_WARN("Turtle2 is near the boundary! Stopping turtle2.");
                stopTurtle(stop_pub_turtle2);
            }
        }

        loop_rate.sleep();
    }

    return 0;
}

