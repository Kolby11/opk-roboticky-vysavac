#include <iostream>
#include <memory>
#include <cmath>
#include <string>

#include "Canvas.h"
#include "controls/Controls.hpp"
#include "environment/Environment.h"
#include "parser/Parser.h"
#include "robot/Robot.h"
#include "robot/RobotNodes.h"
#include "robot/lidar.h"

namespace
{
    void drawScene(canvas::Canvas &canvas,
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
        canvas.drawRobot(state.x, state.y, state.theta);
        canvas.show();
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

    robot::Config robot_config{
        .accelerations = {80.0, 2.0},
        .emergency_decelerations = {150.0, 4.0},
        .command_duration = 0.15,
        .simulation_period_ms = 20,
        .initial_state = {
            environment->getWidth() / 2.0,
            environment->getHeight() / 2.0,
            0.0,
            {0.0, 0.0}}};

    const double robot_radius = environment->getRobotRadius();
    robot::Robot robot(robot_config, [&environment, robot_radius](geometry::RobotState s) -> bool
                       {
        const int points = 16;
        for (int i = 0; i < points; ++i)
        {
            double angle = 2.0 * M_PI * i / points;
            double px = s.x + robot_radius * std::cos(angle);
            double py = s.y + robot_radius * std::sin(angle);
            if (environment->isOccupied(px, py))
                return true;
        }
        return false; });

    canvas::Canvas canvas(environment->getMapFilename(), environment->getResolution());

    const double linear_speed = 60.0;
    const double angular_speed = 1.5;

    auto controls = std::make_shared<Controls>(linear_speed, angular_speed);
    auto command_subscriber = std::make_shared<RobotCommandSubscriber>(robot);
    auto state_publisher = std::make_shared<RobotStatePublisher>(robot);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controls);
    executor.add_node(command_subscriber);
    executor.add_node(state_publisher);

    while (rclcpp::ok())
    {
        drawScene(canvas, *environment, *lidar, robot.getState());
        if (!controls->handleInput())
            break;
        executor.spin_some();
    }

    rclcpp::shutdown();
    return 0;
}
