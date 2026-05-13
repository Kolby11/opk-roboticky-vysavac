#include "game/GameNode.h"

#include <cmath>
#include <map>
#include <sstream>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace game
{
    GameNode::GameNode(Game &game)
        : Node("game_node"),
          game_(game),
          state_publisher_(this->create_publisher<std_msgs::msg::String>("game_state", 10)),
          path_publisher_(this->create_publisher<nav_msgs::msg::Path>("game_path", 10))
    {
        if (!game_.getState().running && !game_.getState().finished)
            game_.startKeepClean();

        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GameNode::odometryCallback, this, std::placeholders::_1));
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
        std_msgs::msg::String message;
        message.data = stateJson();
        state_publisher_->publish(message);
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

    std::string GameNode::stateJson() const
    {
        const GameState &state = game_.getState();
        std::map<std::string, int> remaining_by_type;
        for (const Waste *waste : game_.getWaste())
            ++remaining_by_type[waste->getType()];

        std::ostringstream body;
        body << "{\"mode\":\"keep_clean\","
             << "\"running\":" << (state.running ? "true" : "false") << ','
             << "\"finished\":" << (state.finished ? "true" : "false") << ','
             << "\"success\":" << (state.success ? "true" : "false") << ','
             << "\"capacity\":" << state.current_capacity << ','
             << "\"maxCapacity\":" << state.max_capacity << ','
             << "\"score\":" << state.score << ','
             << "\"currentWave\":" << state.current_wave << ','
             << "\"totalWaves\":" << state.total_waves << ','
             << "\"wastePerWave\":" << state.waste_per_wave << ','
             << "\"requiredPerWave\":" << state.required_per_wave << ','
             << "\"collectedInWave\":" << state.collected_in_wave << ','
             << "\"waveElapsedSeconds\":" << state.wave_elapsed_seconds << ','
             << "\"waveTimeLimitSeconds\":" << state.wave_time_limit_seconds << ','
             << "\"elapsedSeconds\":" << state.elapsed_seconds << ','
             << "\"pathLength\":" << state.path.size() << ','
             << "\"endReason\":\"" << state.end_reason << "\",";

        auto writeCounts = [&body](const std::map<std::string, int> &counts)
        {
            body << '{';
            bool first = true;
            for (const auto &[type, count] : counts)
            {
                if (!first)
                    body << ',';
                body << '"' << type << "\":" << count;
                first = false;
            }
            body << '}';
        };

        body << "\"remainingByType\":";
        writeCounts(remaining_by_type);
        body << ",\"collectedByType\":";
        writeCounts(state.collected_by_type);
        body << ",\"deliveredByType\":";
        writeCounts(state.delivered_by_type);
        body << '}';

        return body.str();
    }
} // namespace game
