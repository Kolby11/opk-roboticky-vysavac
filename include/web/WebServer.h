#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "environment/Environment.h"
#include "game/Game.h"
#include "robot/Robot.h"
#include "robot/lidar.h"

namespace web
{
    struct ServerConfig
    {
        int port{8080};
        std::filesystem::path static_root{"web/dist"};
    };

    class WebServer
    {
    public:
        WebServer(const ServerConfig &config,
                  robot::Robot &robot,
                  const environment::Environment &environment,
                  const lidar::Lidar &lidar,
                  const game::Game *game = nullptr);
        ~WebServer();

        WebServer(const WebServer &) = delete;
        WebServer &operator=(const WebServer &) = delete;

        void start();
        void stop();
        int port() const;

    private:
        struct Request;

        void serve();
        void handleClient(int client_fd);
        std::string route(const Request &request) const;
        std::string stateJson() const;
        std::string environmentJson() const;
        std::string sceneJson() const;
        std::string handleCommand(const Request &request) const;
        std::string serveFile(const std::filesystem::path &path) const;

        ServerConfig config_;
        robot::Robot &robot_;
        const environment::Environment &environment_;
        const lidar::Lidar &lidar_;
        const game::Game *game_;
        int server_fd_{-1};
        std::thread server_thread_;
        std::atomic<bool> running_{false};
    };
} // namespace web
