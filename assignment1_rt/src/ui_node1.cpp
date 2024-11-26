#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>

// Function to spawn a turtle
void spawnTurtle(const std::string &name, float x, float y, float theta) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    turtlesim::Spawn srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    srv.request.name = name;

    if (client.call(srv)) {
        ROS_INFO("Spawned turtle '%s' at (%f, %f, %f)", name.c_str(), x, y, theta);
    } else {
        ROS_ERROR("Failed to call service /spawn");
        ros::shutdown();
    }
}

// Function to get user input for turtle selection and velocities
bool getUserInput(std::string &turtle, float &linear, float &angular) {
    int choice;
    std::cout << "Which turtle to control? (1 for turtle1, 2 for turtle2): ";
    std::cin >> choice;

    if (choice == 1) {
        turtle = "turtle1";
    } else if (choice == 2) {
        turtle = "turtle2";
    } else {
        std::cerr << "Invalid choice. Please enter 1 or 2." << std::endl;
        return false;
    }

    std::cout << "Enter linear velocity: ";
    if (!(std::cin >> linear)) {
        std::cerr << "Invalid input. Please enter numeric values for velocity." << std::endl;
        return false;
    }

    std::cout << "Enter angular velocity: ";
    if (!(std::cin >> angular)) {
        std::cerr << "Invalid input. Please enter numeric values for velocity." << std::endl;
        return false;
    }

    return true;
}

// Function to send velocity commands to the turtle
void sendCommand(ros::Publisher &pub, float linear, float angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    pub.publish(cmd);
    ROS_INFO("Command sent: linear=%f, angular=%f", linear, angular);

    ros::Duration(1.0).sleep(); // Send the command for 1 second

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub.publish(cmd);
    ROS_INFO("Turtle stopped.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ui_node");

    ros::NodeHandle nh;

    // Spawn a second turtle
    spawnTurtle("turtle2", 3.0, 3.0, 0.0);

    // Publishers for turtle1 and turtle2
    ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ROS_INFO("User interface is ready. Enter commands:");

    while (ros::ok()) {
        try {
            std::string turtle;
            float linear, angular;

            if (!getUserInput(turtle, linear, angular)) {
                continue;
            }

            if (turtle == "turtle1") {
                sendCommand(pub_turtle1, linear, angular);
            } else if (turtle == "turtle2") {
                sendCommand(pub_turtle2, linear, angular);
            }

        } catch (const std::exception &e) {
            ROS_ERROR("Exception occurred: %s", e.what());
            break;
        }
    }

    return 0;
}

