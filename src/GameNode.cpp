#include "game/GameNode.h"

#include <cmath>
#include <map>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace game
{
    GameNode::GameNode(Game &game)
        : Node("game_node"),
          game_(game),
          state_publisher_(this->create_publisher<robot_assignment::msg::GameState>("game_state", 10)),
          path_publisher_(this->create_publisher<nav_msgs::msg::Path>("game_path", 10))
    {
        if (!game_.getState().running && !game_.getState().finished)
            game_.startKeepClean();

        start_game_service_ = this->create_service<robot_assignment::srv::StartGame>(
            "start_game", std::bind(&GameNode::startGameCallback, this, std::placeholders::_1, std::placeholders::_2));
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GameNode::odometryCallback, this, std::placeholders::_1));
    }

    void GameNode::startGameCallback(const robot_assignment::srv::StartGame::Request::SharedPtr,
                                     const robot_assignment::srv::StartGame::Response::SharedPtr response)
    {
        game_.startKeepClean();
        publishState();
        publishPath();
        response->ok = true;
        response->message = "started";
    }

    void GameNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
    {
        geometry::RobotState robot_state;
        robot_state.x = message->pose.pose.position.x;
        robot_state.y = message->pose.pose.position.y;
        robot_state.velocity.linear = message->twist.twist.linear.x;
        robot_state.velocity.angular = message->twist.twist.angular.z;

        const auto &orientation = message->pose.pose.orientation;
        robot_state.theta = std::atan2(2.0 * (orientation.w * orientation.z),
                                       1.0 - 2.0 * orientation.z * orientation.z);

        game_.updateRobotState(robot_state);
        publishState();
        publishPath();
    }

    void GameNode::publishState()
    {
        state_publisher_->publish(stateMessage());
    }

    void GameNode::publishPath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom";

        for (const geometry::Point2d &point : game_.getState().path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        path_publisher_->publish(path);
    }

    robot_assignment::msg::GameState GameNode::stateMessage() const
    {
        const GameState &state = game_.getState();
        std::map<std::string, int> remaining_by_type;
        for (const Waste *waste : game_.getWaste())
            ++remaining_by_type[waste->getType()];

        robot_assignment::msg::GameState message;
        message.running = state.running;
        message.finished = state.finished;
        message.success = state.success;
        message.capacity = state.current_capacity;
        message.max_capacity = state.max_capacity;
        message.score = state.score;
        message.current_wave = state.current_wave;
        message.total_waves = state.total_waves;
        message.waste_per_wave = state.waste_per_wave;
        message.required_per_wave = state.required_per_wave;
        message.collected_in_wave = state.collected_in_wave;
        message.wave_elapsed_seconds = state.wave_elapsed_seconds;
        message.wave_time_limit_seconds = state.wave_time_limit_seconds;
        message.elapsed_seconds = state.elapsed_seconds;
        message.path_length = static_cast<uint32_t>(state.path.size());
        message.end_reason = state.end_reason;

        auto addCounts = [](const std::map<std::string, int> &counts,
                            std::vector<std::string> &types,
                            std::vector<int32_t> &values)
        {
            types.reserve(counts.size());
            values.reserve(counts.size());
            for (const auto &[type, count] : counts)
            {
                types.push_back(type);
                values.push_back(count);
            }
        };

        addCounts(remaining_by_type, message.remaining_types, message.remaining_counts);
        addCounts(state.collected_by_type, message.collected_types, message.collected_counts);
        addCounts(state.delivered_by_type, message.delivered_types, message.delivered_counts);

        return message;
    }
} // namespace game
