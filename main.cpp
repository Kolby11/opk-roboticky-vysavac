#include <iostream>
#include <memory>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "Canvas.h"
#include "controls/Controls.hpp"
#include "environment/Environment.h"
#include "parser/Parser.h"
#include "robot/Robot.h"
#include "robot/RobotNodes.h"
#include "robot/lidar.h"

namespace
{
    bool robotCollides(const environment::Environment &environment,
                       double robot_radius,
                       const geometry::RobotState &state)
    {
        const int points = 32;
        if (environment.isOccupied(state.x, state.y))
            return true;

        for (int i = 0; i < points; ++i)
        {
            const double angle = 2.0 * M_PI * i / points;
            const double px = state.x + robot_radius * std::cos(angle);
            const double py = state.y + robot_radius * std::sin(angle);
            if (environment.isOccupied(px, py))
                return true;
        }

        return false;
    }

    geometry::RobotState findInitialRobotState(const environment::Environment &environment,
                                               double robot_radius)
    {
        const double center_x = environment.getWidth() / 2.0;
        const double center_y = environment.getHeight() / 2.0;
        const std::vector<double> headings{
            0.0,
            M_PI / 4.0,
            M_PI / 2.0,
            3.0 * M_PI / 4.0,
            M_PI,
            -3.0 * M_PI / 4.0,
            -M_PI / 2.0,
            -M_PI / 4.0};

        geometry::RobotState best_state{center_x, center_y, 0.0, {0.0, 0.0}};
        double best_distance = std::numeric_limits<double>::max();
        bool found = false;

        for (double y = robot_radius; y <= environment.getHeight() - robot_radius; y += 1.0)
        {
            for (double x = robot_radius; x <= environment.getWidth() - robot_radius; x += 1.0)
            {
                geometry::RobotState candidate{x, y, 0.0, {0.0, 0.0}};
                if (robotCollides(environment, robot_radius, candidate))
                    continue;

                for (double heading : headings)
                {
                    bool has_forward_room = true;
                    for (int step = 1; step <= 3; ++step)
                    {
                        candidate = {x + step * std::cos(heading),
                                     y + step * std::sin(heading),
                                     heading,
                                     {0.0, 0.0}};
                        if (robotCollides(environment, robot_radius, candidate))
                        {
                            has_forward_room = false;
                            break;
                        }
                    }

                    if (!has_forward_room)
                        continue;

                    const double distance = std::hypot(x - center_x, y - center_y);
                    if (distance < best_distance)
                    {
                        found = true;
                        best_distance = distance;
                        best_state = {x, y, heading, {0.0, 0.0}};
                    }
                }
            }
        }

        if (found)
            return best_state;

        return best_state;
    }

    int drawScene(canvas::Canvas &canvas,
                  const environment::Environment &environment,
                  const lidar::Lidar &lidar,
                  const geometry::RobotState &state)
    {
        const auto hits = lidar.scan(state);

        canvas.reset();
        for (const auto &obstacle : environment.getCircleObstacles())
            canvas.drawCircleObstacle(obstacle);
        for (const auto &obstacle : environment.getRectangleObstacles())
            canvas.drawRectangleObstacle(obstacle);
        if (environment.getStation().has_value())
            canvas.drawStation(*environment.getStation());

        canvas.drawRays(state.x, state.y, hits);
        canvas.drawLidarPoints(hits);
        canvas.drawRobot(state.x, state.y, state.theta, environment.getRobotRadius());
        return canvas.show();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <map_file|config.yml>\n";
        rclcpp::shutdown();
        return 1;
    }

    const std::string input_file = argv[1];

    lidar::Config lidar_config = {
        .max_range = 200,
        .beam_count = 36,
        .first_ray_angle = -M_PI,
        .last_ray_angle = M_PI,
    };

    std::shared_ptr<environment::Environment> environment;
    if (input_file.ends_with(".yaml") || input_file.ends_with(".yml"))
    {
        const YAMLDocument document = YAMLParser::parseFile(input_file);
        environment = std::make_shared<environment::Environment>(document, input_file);
    }
    else
    {
        environment::Config environment_config = {
            .map_filename = input_file,
            .resolution = 1.0};
        environment = std::make_shared<environment::Environment>(environment_config);
    }

    auto lidar = std::make_shared<lidar::Lidar>(lidar_config, environment);

    const double robot_radius = environment->getRobotRadius();
    const geometry::RobotState initial_state = findInitialRobotState(*environment, robot_radius);

    robot::Config robot_config{
        .accelerations = {80.0, 2.0},
        .emergency_decelerations = {300.0, 8.0},
        .command_duration = 0.08,
        .simulation_period_ms = 20,
        .initial_state = initial_state};

    robot::Robot robot(robot_config, [&environment, robot_radius](geometry::RobotState s) -> bool
                       {
        return robotCollides(*environment, robot_radius, s); });

    canvas::Canvas canvas(environment->getMapFilename(), environment->getResolution());

    const double linear_speed = 60.0;
    const double angular_speed = 1.5;

    auto controls = std::make_shared<Controls>(linear_speed, angular_speed);
    auto command_subscriber = std::make_shared<RobotCommandSubscriber>(robot);
    auto state_publisher = std::make_shared<RobotStatePublisher>(robot);
    auto laser_scan_publisher = std::make_shared<LaserScanPublisher>(robot, *lidar);
    auto environment_marker_publisher = std::make_shared<EnvironmentMarkerPublisher>(*environment);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controls);
    executor.add_node(command_subscriber);
    executor.add_node(state_publisher);
    executor.add_node(laser_scan_publisher);
    executor.add_node(environment_marker_publisher);

    while (rclcpp::ok())
    {
        const int window_key = drawScene(canvas, *environment, *lidar, robot.getState());
        if (!controls->handleKey(window_key))
            break;
        if (!controls->handleInput(1))
            break;
        controls->publishActiveCommand();
        executor.spin_some();
    }

    rclcpp::shutdown();
    return 0;
}
