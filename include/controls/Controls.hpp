#pragma once

#include <string>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Controls : public rclcpp::Node
{
public:
    Controls(double linear_speed, double angular_speed);
    ~Controls() override;

    // Returns false when the user asks to quit.
    bool handleInput(int wait_ms = 30);

private:
    void setupTerminal();
    void restoreTerminal();
    int readInput(int wait_ms) const;
    void publishVelocity(double linear, double angular);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    double linear_speed_;
    double angular_speed_;
    termios original_terminal_{};
    bool terminal_configured_{false};
};
