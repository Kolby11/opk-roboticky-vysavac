#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>
#include <string>
#include <vector>

#include "controls/Controls.hpp"
#include "environment/Environment.h"
#include "game/Game.h"
#include "parser/Parser.h"
#include "robot/Robot.h"
#include "robot/RobotNodes.h"
#include "robot/lidar.h"
#include "visualization/OpenCvSceneRenderer.h"
#include "web/WebServer.h"

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

    struct RuntimeOptions
    {
        std::string input_file;
        bool opencv_renderer{false};
        bool web_server{true};
        int web_port{8080};
        bool force_keep_clean{false};
    };

    RuntimeOptions parseOptions(int argc, char *argv[])
    {
        RuntimeOptions options;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--opencv")
                options.opencv_renderer = true;
            else if (arg == "--no-opencv")
                options.opencv_renderer = false;
            else if (arg == "--no-web")
                options.web_server = false;
            else if (arg == "--web-port")
            {
                if (i + 1 >= argc)
                    throw std::runtime_error("--web-port requires a port number");
                options.web_port = std::stoi(argv[++i]);
            }
            else if (arg == "--mode")
            {
                if (i + 1 >= argc)
                    throw std::runtime_error("--mode requires a mode name");
                const std::string mode = argv[++i];
                if (mode == "keep-clean" || mode == "keep_clean")
                    options.force_keep_clean = true;
                else
                    throw std::runtime_error("Unsupported mode: " + mode);
            }
            else if (options.input_file.empty())
                options.input_file = arg;
            else
                throw std::runtime_error("Unexpected argument: " + arg);
        }

        return options;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    RuntimeOptions options;
    try
    {
        options = parseOptions(argc, argv);
    }
    catch (const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    if (options.input_file.empty())
    {
        std::cerr << "Usage: " << argv[0] << " <map_file|config.yml> [--mode keep-clean] [--opencv] [--no-web] [--web-port 8080]\n";
        rclcpp::shutdown();
        return 1;
    }

    const std::string input_file = options.input_file;

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
    game::Game game(*environment);
    if (options.force_keep_clean || game.getMode() == game::GameMode::KeepClean)
    {
        game.startKeepClean();
        std::cout << "Game mode: Keep Clean\n";
    }
    else
    {
        std::cout << "Game mode was drawn, but only Keep Clean is implemented in this build.\n";
    }

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

    std::unique_ptr<web::WebServer> web_server;
    if (options.web_server)
    {
        web_server = std::make_unique<web::WebServer>(web::ServerConfig{.port = options.web_port}, robot, *environment, *lidar, &game);
        web_server->start();
        std::cout << "Web UI/API listening on http://localhost:" << web_server->port() << "\n";
    }

    std::unique_ptr<visualization::OpenCvSceneRenderer> opencv_renderer;
    if (options.opencv_renderer)
        opencv_renderer = std::make_unique<visualization::OpenCvSceneRenderer>(*environment, *lidar);

    const double linear_speed = 60.0;
    const double angular_speed = 1.5;

    auto controls = std::make_shared<Controls>(linear_speed, angular_speed);
    auto command_subscriber = std::make_shared<RobotCommandSubscriber>(robot);
    auto state_publisher = std::make_shared<RobotStatePublisher>(robot);
    auto robot_marker_publisher = std::make_shared<RobotMarkerPublisher>(robot, robot_radius);
    auto laser_scan_publisher = std::make_shared<LaserScanPublisher>(robot, *lidar);
    auto waste_marker_publisher = std::make_shared<WasteMarkerPublisher>(game);
    auto environment_map_publisher = std::make_shared<EnvironmentMapPublisher>(*environment);
    auto environment_marker_publisher = std::make_shared<EnvironmentMarkerPublisher>(*environment);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controls);
    executor.add_node(command_subscriber);
    executor.add_node(state_publisher);
    executor.add_node(robot_marker_publisher);
    executor.add_node(laser_scan_publisher);
    executor.add_node(waste_marker_publisher);
    executor.add_node(environment_map_publisher);
    executor.add_node(environment_marker_publisher);

    std::thread ros_thread([&executor]()
                           { executor.spin(); });

    while (rclcpp::ok())
    {
        if (opencv_renderer)
        {
            if (!opencv_renderer->render(robot.getState()))
                break;
            if (!controls->handleKey(opencv_renderer->lastKey()))
                break;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        if (!controls->handleInput(1))
            break;
        controls->publishActiveCommand();
        game.updateRobotState(robot.getState());
        if (game.getState().finished)
        {
            std::cout << "Game finished: "
                      << (game.getState().success ? "success" : "failed")
                      << " (" << game.getState().end_reason << ")\n";
            break;
        }
    }

    executor.cancel();
    if (ros_thread.joinable())
        ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
