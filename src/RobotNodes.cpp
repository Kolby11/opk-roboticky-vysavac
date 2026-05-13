#include "robot/RobotNodes.h"

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
    geometry_msgs::msg::Quaternion orientation;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
    return orientation;
}

void setMarkerColor(visualization_msgs::msg::Marker &marker,
                    float red,
                    float green,
                    float blue,
                    float alpha)
{
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
}
} // namespace

RobotCommandSubscriber::RobotCommandSubscriber(robot::Robot &robot)
    : Node("robot_command_subscriber"),
      robot_(robot)
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "robot_command", 10, std::bind(&RobotCommandSubscriber::commandCallback, this, _1));
    twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&RobotCommandSubscriber::twistCallback, this, _1));
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

void RobotCommandSubscriber::twistCallback(const geometry_msgs::msg::Twist::SharedPtr message)
{
    robot_.setVelocity({message->linear.x, message->angular.z});
}

RobotStatePublisher::RobotStatePublisher(const robot::Robot &robot)
    : Node("robot_state_publisher"),
      robot_(robot),
      publisher_(this->create_publisher<std_msgs::msg::String>("robot_state", 10)),
      odometry_publisher_(this->create_publisher<nav_msgs::msg::Odometry>("odom", 10)),
      transform_publisher_(this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10))
{
    timer_ = this->create_wall_timer(20ms, std::bind(&RobotStatePublisher::publishState, this));
}

void RobotStatePublisher::publishState()
{
    const geometry::RobotState state = robot_.getState();
    const rclcpp::Time stamp = this->now();

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

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = state.x;
    odometry.pose.pose.position.y = state.y;
    odometry.pose.pose.orientation = yawToQuaternion(state.theta);
    odometry.twist.twist.linear.x = state.velocity.linear;
    odometry.twist.twist.angular.z = state.velocity.angular;
    odometry_publisher_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = state.x;
    transform.transform.translation.y = state.y;
    transform.transform.rotation = yawToQuaternion(state.theta);

    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms.push_back(transform);
    transform_publisher_->publish(tf_message);
}

RobotMarkerPublisher::RobotMarkerPublisher(const robot::Robot &robot, double robot_radius)
    : Node("robot_marker_publisher"),
      robot_(robot),
      robot_radius_(robot_radius),
      publisher_(this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_markers", 10))
{
    timer_ = this->create_wall_timer(20ms, std::bind(&RobotMarkerPublisher::publishMarker, this));
}

void RobotMarkerPublisher::publishMarker()
{
    const geometry::RobotState state = robot_.getState();
    const rclcpp::Time stamp = this->now();

    visualization_msgs::msg::Marker body;
    body.header.stamp = stamp;
    body.header.frame_id = "odom";
    body.ns = "robot";
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::CYLINDER;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = state.x;
    body.pose.position.y = state.y;
    body.pose.position.z = 0.08;
    body.pose.orientation = yawToQuaternion(state.theta);
    body.scale.x = robot_radius_ * 2.0;
    body.scale.y = robot_radius_ * 2.0;
    body.scale.z = 0.16;
    setMarkerColor(body, 0.1F, 0.75F, 0.25F, 0.95F);

    visualization_msgs::msg::Marker heading;
    heading.header.stamp = stamp;
    heading.header.frame_id = "odom";
    heading.ns = "robot";
    heading.id = 1;
    heading.type = visualization_msgs::msg::Marker::ARROW;
    heading.action = visualization_msgs::msg::Marker::ADD;
    heading.pose.position.x = state.x;
    heading.pose.position.y = state.y;
    heading.pose.position.z = 0.18;
    heading.pose.orientation = yawToQuaternion(state.theta);
    heading.scale.x = robot_radius_ * 1.6;
    heading.scale.y = robot_radius_ * 0.35;
    heading.scale.z = robot_radius_ * 0.35;
    setMarkerColor(heading, 0.0F, 0.25F, 0.08F, 1.0F);

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(body);
    markers.markers.push_back(heading);
    publisher_->publish(markers);
}

LaserScanPublisher::LaserScanPublisher(const robot::Robot &robot, const lidar::Lidar &lidar)
    : Node("laser_scan_publisher"),
      robot_(robot),
      lidar_(lidar),
      publisher_(this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10))
{
    timer_ = this->create_wall_timer(50ms, std::bind(&LaserScanPublisher::publishScan, this));
}

void LaserScanPublisher::publishScan()
{
    const geometry::RobotState state = robot_.getState();
    const lidar::Config &config = lidar_.getConfig();
    const std::vector<double> ranges = lidar_.scanRanges(state);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->now();
    scan.header.frame_id = "base_link";
    scan.angle_min = config.first_ray_angle;
    scan.angle_max = config.last_ray_angle;
    scan.angle_increment = config.beam_count > 1
                               ? (config.last_ray_angle - config.first_ray_angle) / (config.beam_count - 1)
                               : 0.0;
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;
    scan.range_min = 0.0;
    scan.range_max = config.max_range;
    scan.ranges.reserve(ranges.size());
    for (double range : ranges)
        scan.ranges.push_back(static_cast<float>(range));

    publisher_->publish(scan);
}

WasteMarkerPublisher::WasteMarkerPublisher(const game::Game &game)
    : Node("waste_marker_publisher"),
      game_(game),
      publisher_(this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "waste_markers", rclcpp::QoS(1).transient_local().reliable()))
{
    timer_ = this->create_wall_timer(100ms, std::bind(&WasteMarkerPublisher::publishMarkers, this));
}

void WasteMarkerPublisher::publishMarkers()
{
    visualization_msgs::msg::MarkerArray markers;
    const auto waste = game_.getWaste();

    for (std::size_t i = 0; i < waste.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "waste";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waste[i]->getPosition().x;
        marker.pose.position.y = waste[i]->getPosition().y;
        marker.pose.position.z = 0.12;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = waste[i]->getRadius() * 2.0;
        marker.scale.y = waste[i]->getRadius() * 2.0;
        marker.scale.z = 0.24;
        setMarkerColor(marker, 0.95F, 0.72F, 0.15F, 0.95F);
        markers.markers.push_back(marker);
    }

    for (std::size_t i = waste.size(); i < previous_count_; ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "waste";
        marker.id = static_cast<int>(i);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(marker);
    }

    previous_count_ = waste.size();
    publisher_->publish(markers);
}

EnvironmentMarkerPublisher::EnvironmentMarkerPublisher(const environment::Environment &environment)
    : Node("environment_marker_publisher"),
      environment_(environment),
      publisher_(this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "environment_markers", rclcpp::QoS(1).transient_local().reliable()))
{
    timer_ = this->create_wall_timer(250ms, [this]()
                                     {
        publishMarkers();
        timer_->cancel(); });
}

void EnvironmentMarkerPublisher::publishMarkers()
{
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    for (const auto &obstacle : environment_.getCircleObstacles())
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "circle_obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = obstacle.center.x;
        marker.pose.position.y = obstacle.center.y;
        marker.pose.position.z = 0.025;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacle.radius * 2.0;
        marker.scale.y = obstacle.radius * 2.0;
        marker.scale.z = 0.05;
        setMarkerColor(marker, 0.85F, 0.15F, 0.15F, 0.8F);
        markers.markers.push_back(marker);
    }

    for (const auto &obstacle : environment_.getRectangleObstacles())
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "rectangle_obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = obstacle.origin.x + obstacle.width * 0.5;
        marker.pose.position.y = obstacle.origin.y + obstacle.height * 0.5;
        marker.pose.position.z = 0.025;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacle.width;
        marker.scale.y = obstacle.height;
        marker.scale.z = 0.05;
        setMarkerColor(marker, 0.85F, 0.15F, 0.15F, 0.8F);
        markers.markers.push_back(marker);
    }

    if (environment_.getStation().has_value())
    {
        const environment::Station &station = *environment_.getStation();
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "station";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = station.origin.x;
        marker.pose.position.y = station.origin.y;
        marker.pose.position.z = 0.02;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = station.radius * 2.0;
        marker.scale.y = station.radius * 2.0;
        marker.scale.z = 0.04;
        setMarkerColor(marker, 0.1F, 0.55F, 0.9F, 0.6F);
        markers.markers.push_back(marker);
    }

    publisher_->publish(markers);
}

EnvironmentMapPublisher::EnvironmentMapPublisher(const environment::Environment &environment)
    : Node("environment_map_publisher"),
      environment_(environment),
      publisher_(this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable()))
{
    timer_ = this->create_wall_timer(250ms, [this]()
                                     {
        publishMap();
        timer_->cancel(); });
}

void EnvironmentMapPublisher::publishMap()
{
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = this->now();
    map.header.frame_id = "odom";
    map.info.resolution = static_cast<float>(environment_.getResolution());
    map.info.width = static_cast<unsigned int>(environment_.getMapWidthPixels());
    map.info.height = static_cast<unsigned int>(environment_.getMapHeightPixels());
    map.info.origin.orientation.w = 1.0;

    map.data.reserve(static_cast<std::size_t>(map.info.width * map.info.height));
    for (int y = 0; y < environment_.getMapHeightPixels(); ++y)
    {
        for (int x = 0; x < environment_.getMapWidthPixels(); ++x)
        {
            const unsigned char pixel = environment_.getMapPixel(x, y);
            map.data.push_back(pixel < 128 ? 100 : 0);
        }
    }

    publisher_->publish(map);
}
