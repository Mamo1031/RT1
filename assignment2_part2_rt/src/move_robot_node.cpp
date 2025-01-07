#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveRobotNode : public rclcpp::Node
{
public:
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

// Main function to initialize and run the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobotNode>());
    rclcpp::shutdown();
    return 0;
}

