#pragma once

#include "game/Game.h"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace game
{
    class GameNode : public rclcpp::Node
    {
    public:
        explicit GameNode(Game &game);

    private:
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);
        void publishState();
        void publishPath();
        std::string stateJson() const;

        Game &game_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    };
} // namespace game
