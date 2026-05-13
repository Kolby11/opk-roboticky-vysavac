#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "environment/Environment.h"
#include "game/Game.h"
#include "robot/lidar.h"
#include "robot/Robot.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class RobotCommandSubscriber : public rclcpp::Node
{
public:
    explicit RobotCommandSubscriber(robot::Robot &robot);

private:
    void commandCallback(const std_msgs::msg::String::SharedPtr message);
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr message);

    robot::Robot &robot_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transform_publisher_;
};

class RobotMarkerPublisher : public rclcpp::Node
{
public:
    RobotMarkerPublisher(const robot::Robot &robot, double robot_radius);

private:
    void publishMarker();

    const robot::Robot &robot_;
    double robot_radius_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

class LaserScanPublisher : public rclcpp::Node
{
public:
    LaserScanPublisher(const robot::Robot &robot, const lidar::Lidar &lidar);

private:
    void publishScan();

    const robot::Robot &robot_;
    const lidar::Lidar &lidar_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

class WasteMarkerPublisher : public rclcpp::Node
{
public:
    explicit WasteMarkerPublisher(const game::Game &game);

private:
    void publishMarkers();

    const game::Game &game_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    std::size_t previous_count_{0};
};

class EnvironmentMarkerPublisher : public rclcpp::Node
{
public:
    explicit EnvironmentMarkerPublisher(const environment::Environment &environment);

private:
    void publishMarkers();

    const environment::Environment &environment_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

class EnvironmentMapPublisher : public rclcpp::Node
{
public:
    explicit EnvironmentMapPublisher(const environment::Environment &environment);

private:
    void publishMap();

    const environment::Environment &environment_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};
