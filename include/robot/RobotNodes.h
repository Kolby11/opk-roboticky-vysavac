#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robot/Robot.h"
#include "std_msgs/msg/string.hpp"

class RobotCommandSubscriber : public rclcpp::Node
{
public:
    explicit RobotCommandSubscriber(robot::Robot &robot);

private:
    void commandCallback(const std_msgs::msg::String::SharedPtr message);

    robot::Robot &robot_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class RobotStatePublisher : public rclcpp::Node
{
public:
    explicit RobotStatePublisher(const robot::Robot &robot);

private:
    void publishState();

    const robot::Robot &robot_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
