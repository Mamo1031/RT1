C++ API Documentation
=====================

This page describes the C++ components of the project.

MoveRobotNode
-------------

The `MoveRobotNode` class implements a ROS2 node that controls the robot's movement.
This node periodically publishes velocity commands to the `cmd_vel` topic.

Detailed API documentation can be found at the following link:

`C++ API Documentation <doxygen/html/index.html>`_

Key Features
------------

- Sends periodic velocity commands to the robot
- Controls linear and angular velocity

Code Overview
-------------

.. code-block:: cpp

   // Main portion of move_robot_node.cpp (excerpt)
   class MoveRobotNode : public rclcpp::Node
   {
   public:
       MoveRobotNode() : Node("move_robot_node") 
       {
           publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
           timer_ = this->create_wall_timer(
               std::chrono::milliseconds(500), 
               std::bind(&MoveRobotNode::publish_velocity_command, this));
       }
   
   private:
       void publish_velocity_command()
       {
           auto message = geometry_msgs::msg::Twist();
           message.linear.x = 0.5;
           message.angular.z = 1.0;
           publisher_->publish(message);
       }
   };