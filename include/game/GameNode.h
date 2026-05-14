#pragma once

#include "game/Game.h"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_assignment/msg/game_state.hpp"
#include "robot_assignment/srv/start_game.hpp"

namespace game
{
    class GameNode : public rclcpp::Node
    {
    public:
        explicit GameNode(Game &game);

    private:
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);
        void startGameCallback(const robot_assignment::srv::StartGame::Request::SharedPtr request,
                               const robot_assignment::srv::StartGame::Response::SharedPtr response);
        void publishState();
        void publishPath();
        robot_assignment::msg::GameState stateMessage() const;

        Game &game_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
        rclcpp::Service<robot_assignment::srv::StartGame>::SharedPtr start_game_service_;
        rclcpp::Publisher<robot_assignment::msg::GameState>::SharedPtr state_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    };
} // namespace game
