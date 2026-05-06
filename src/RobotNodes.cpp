#include "robot/RobotNodes.h"

#include <chrono>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

RobotCommandSubscriber::RobotCommandSubscriber(robot::Robot &robot)
    : Node("robot_command_subscriber"),
      robot_(robot)
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "robot_command", 10, std::bind(&RobotCommandSubscriber::commandCallback, this, _1));
}

void RobotCommandSubscriber::commandCallback(const std_msgs::msg::String::SharedPtr message)
{
    std::istringstream stream(message->data);
    geometry::Twist velocity;

    if (!(stream >> velocity.linear >> velocity.angular))
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring invalid robot command: '%s'", message->data.c_str());
        return;
    }

    robot_.setVelocity(velocity);
}

RobotStatePublisher::RobotStatePublisher(const robot::Robot &robot)
    : Node("robot_state_publisher"),
      robot_(robot),
      publisher_(this->create_publisher<std_msgs::msg::String>("robot_state", 10))
{
    timer_ = this->create_wall_timer(100ms, std::bind(&RobotStatePublisher::publishState, this));
}

void RobotStatePublisher::publishState()
{
    const geometry::RobotState state = robot_.getState();

    auto message = std_msgs::msg::String();
    std::ostringstream data;
    data << "x=" << state.x
         << " y=" << state.y
         << " theta=" << state.theta
         << " linear=" << state.velocity.linear
         << " angular=" << state.velocity.angular
         << " collision=" << (robot_.isInCollision() ? "true" : "false");
    message.data = data.str();

    publisher_->publish(message);
}
