/**
 * @file move_robot_node.cpp
 * @brief A ROS2 node for controlling robot movement
 * 
 * This node publishes velocity commands to control a robot's movement.
 * It sends linear and angular velocities to the cmd_vel topic.
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
 * @class MoveRobotNode
 * @brief A ROS2 node class that handles robot movement commands
 * 
 * This class creates a publisher to send Twist messages to the cmd_vel topic,
 * controlling the robot's linear and angular velocities at regular intervals.
 */
class MoveRobotNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the MoveRobotNode class
     * 
     * Initializes the node, creates a publisher for velocity commands,
     * and sets up a timer to publish messages periodically.
     */
    MoveRobotNode() : Node("move_robot_node")  // Constructor: Initializes the node
    {
        // Create a publisher to send velocity commands to the 'cmd_vel' topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create a timer to publish messages periodically (every 0.5 seconds)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // Timer interval
            std::bind(&MoveRobotNode::publish_velocity_command, this));  // Timer callback function
    }

private:
    /**
     * @brief Timer callback function to publish velocity commands
     * 
     * This function is called periodically to create and publish a Twist message
     * with pre-defined linear and angular velocities.
     */
    void publish_velocity_command()
    {
        // Create a Twist message to set linear and angular velocities
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;   // Linear velocity in the x-direction
        message.angular.z = 1.0; // Angular velocity around the z-axis

        // Publish the message to the 'cmd_vel' topic
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = '%f', angular.z = '%f'", message.linear.x, message.angular.z);
    }

    // Member variables for the publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function to initialize and run the MoveRobotNode
 * @details This function initializes the ROS2 system, creates an instance of MoveRobotNode,
 * and runs the node until shutdown is triggered.
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Return code (0 if successful)
 * @name main
 * @{
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobotNode>());
    rclcpp::shutdown();
    return 0;
}
/**@}*/
