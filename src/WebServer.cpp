#include "web/WebServer.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace web
{
    namespace
    {
        struct ParsedTarget
        {
            std::string path;
            std::string query;
        };

        std::string header(int status, const std::string &status_text, const std::string &content_type, std::size_t size)
        {
            std::ostringstream response;
            response << "HTTP/1.1 " << status << ' ' << status_text << "\r\n"
                     << "Content-Length: " << size << "\r\n"
                     << "Content-Type: " << content_type << "\r\n"
                     << "Access-Control-Allow-Origin: *\r\n"
                     << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
                     << "Access-Control-Allow-Headers: Content-Type\r\n"
                     << "Connection: close\r\n\r\n";
            return response.str();
        }

        std::string textResponse(int status, const std::string &status_text, const std::string &body, const std::string &content_type = "text/plain")
        {
            return header(status, status_text, content_type, body.size()) + body;
        }

        ParsedTarget parseTarget(const std::string &target)
        {
            const std::size_t separator = target.find('?');
            if (separator == std::string::npos)
                return {target, ""};
            return {target.substr(0, separator), target.substr(separator + 1)};
        }

        std::string decodePath(std::string path)
        {
            std::replace(path.begin(), path.end(), '\\', '/');
            while (path.find("..") != std::string::npos)
                path.erase(path.find(".."), 2);
            if (!path.empty() && path.front() == '/')
                path.erase(path.begin());
            if (path.empty())
                path = "index.html";
            return path;
        }

        std::string mimeType(const std::filesystem::path &path)
        {
            const std::string ext = path.extension().string();
            if (ext == ".html")
                return "text/html";
            if (ext == ".js")
                return "text/javascript";
            if (ext == ".css")
                return "text/css";
            if (ext == ".png")
                return "image/png";
            if (ext == ".svg")
                return "image/svg+xml";
            if (ext == ".json")
                return "application/json";
            return "application/octet-stream";
        }

        bool extractNumber(const std::string &source, const std::string &key, double &value)
        {
            std::size_t position = source.find(key);
            if (position == std::string::npos)
                return false;
            position = source.find_first_of("=-:", position + key.size());
            if (position == std::string::npos)
                return false;
            ++position;
            while (position < source.size() && (source[position] == ' ' || source[position] == '"' || source[position] == '\''))
                ++position;

            std::size_t consumed = 0;
            try
            {
                value = std::stod(source.substr(position), &consumed);
                return consumed > 0;
            }
            catch (const std::exception &)
            {
                return false;
            }
        }

        void sendAll(int client_fd, const std::string &response)
        {
            std::size_t sent = 0;
            while (sent < response.size())
            {
                const ssize_t result = send(client_fd, response.data() + sent, response.size() - sent, MSG_NOSIGNAL);
                if (result <= 0)
                    return;
                sent += static_cast<std::size_t>(result);
            }
        }
    } // namespace

    struct WebServer::Request
    {
        std::string method;
        std::string target;
        std::string path;
        std::string query;
        std::string body;
    };

    WebServer::WebServer(const ServerConfig &config,
                         robot::Robot &robot,
                         const environment::Environment &environment,
                         const lidar::Lidar &lidar,
                         game::Game *game)
        : config_(config),
          robot_(robot),
          environment_(environment),
          lidar_(lidar),
          game_(game)
    {
    }

    WebServer::~WebServer()
    {
        stop();
    }

    void WebServer::start()
    {
        if (running_)
            return;

        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0)
            throw std::runtime_error("Failed to create web server socket");

        int reuse = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<uint16_t>(config_.port));

        if (bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
            const std::string message = std::strerror(errno);
            close(server_fd_);
            server_fd_ = -1;
            throw std::runtime_error("Failed to bind web server port " + std::to_string(config_.port) + ": " + message);
        }

        if (listen(server_fd_, 16) < 0)
        {
            close(server_fd_);
            server_fd_ = -1;
            throw std::runtime_error("Failed to listen on web server socket");
        }

        running_ = true;
        server_thread_ = std::thread(&WebServer::serve, this);
    }

    void WebServer::stop()
    {
        running_ = false;
        if (server_fd_ >= 0)
        {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_fd_ = -1;
        }
        if (server_thread_.joinable())
            server_thread_.join();
    }

    int WebServer::port() const
    {
        return config_.port;
    }

    void WebServer::serve()
    {
        while (running_)
        {
            const int client_fd = accept(server_fd_, nullptr, nullptr);
            if (client_fd < 0)
            {
                if (running_)
                    std::cerr << "Web server accept failed: " << std::strerror(errno) << "\n";
                continue;
            }
            std::thread(&WebServer::handleClient, this, client_fd).detach();
        }
    }

    void WebServer::handleClient(int client_fd)
    {
        std::string raw;
        char buffer[4096];
        ssize_t received = recv(client_fd, buffer, sizeof(buffer), 0);
        if (received > 0)
            raw.assign(buffer, buffer + received);

        std::istringstream stream(raw);
        Request request;
        stream >> request.method >> request.target;
        const ParsedTarget target = parseTarget(request.target);
        request.path = target.path;
        request.query = target.query;

        const std::size_t body_start = raw.find("\r\n\r\n");
        if (body_start != std::string::npos)
            request.body = raw.substr(body_start + 4);

        const std::string response = route(request);
        sendAll(client_fd, response);
        close(client_fd);
    }

    std::string WebServer::route(const Request &request) const
    {
        if (request.method == "OPTIONS")
            return textResponse(204, "No Content", "");
        if (request.method == "GET" && request.path == "/api/state")
            return textResponse(200, "OK", stateJson(), "application/json");
        if (request.method == "GET" && request.path == "/api/environment")
            return textResponse(200, "OK", environmentJson(), "application/json");
        if (request.method == "GET" && request.path == "/api/scene")
            return textResponse(200, "OK", sceneJson(), "application/json");
        if (request.method == "GET" && request.path == "/api/map-image")
            return serveFile(environment_.getMapFilename());
        if (request.method == "POST" && request.path == "/api/command")
            return handleCommand(request);
        if (request.method == "POST" && request.path == "/api/game/restart")
            return handleGameRestart();

        const std::filesystem::path static_path = config_.static_root / decodePath(request.path);
        if (std::filesystem::exists(static_path) && std::filesystem::is_regular_file(static_path))
            return serveFile(static_path);

        const std::filesystem::path index_path = config_.static_root / "index.html";
        if (std::filesystem::exists(index_path))
            return serveFile(index_path);

        return textResponse(404, "Not Found", "Not found\n");
    }

    std::string WebServer::stateJson() const
    {
        const geometry::RobotState state = robot_.getState();
        std::ostringstream body;
        body << std::fixed << std::setprecision(3)
             << "{\"robot\":{"
             << "\"x\":" << state.x << ','
             << "\"y\":" << state.y << ','
             << "\"theta\":" << state.theta << ','
             << "\"linear\":" << state.velocity.linear << ','
             << "\"angular\":" << state.velocity.angular << ','
             << "\"collision\":" << (robot_.isInCollision() ? "true" : "false")
             << "},\"scan\":[";

        const std::vector<geometry::Point2d> hits = lidar_.scan(state);
        for (std::size_t i = 0; i < hits.size(); ++i)
        {
            if (i > 0)
                body << ',';
            body << "{\"x\":" << hits[i].x << ",\"y\":" << hits[i].y << '}';
        }
        body << "],\"waste\":[";
        if (game_)
        {
            const auto waste = game_->getWaste();
            for (std::size_t i = 0; i < waste.size(); ++i)
            {
                if (i > 0)
                    body << ',';
                body << "{\"x\":" << waste[i]->getPosition().x
                     << ",\"y\":" << waste[i]->getPosition().y
                     << ",\"radius\":" << waste[i]->getRadius()
                     << ",\"type\":\"" << waste[i]->getType()
                     << "\",\"color\":\"" << waste[i]->getColor() << "\"}";
            }
        }

        body << "],\"game\":";
        if (game_)
        {
            const game::GameState &game_state = game_->getState();
            body << "{\"mode\":\"keep_clean\","
                 << "\"running\":" << (game_state.running ? "true" : "false") << ','
                 << "\"finished\":" << (game_state.finished ? "true" : "false") << ','
                 << "\"success\":" << (game_state.success ? "true" : "false") << ','
                 << "\"capacity\":" << game_state.current_capacity << ','
                 << "\"maxCapacity\":" << game_state.max_capacity << ','
                 << "\"currentWave\":" << game_state.current_wave << ','
                 << "\"totalWaves\":" << game_state.total_waves << ','
                 << "\"collectedInWave\":" << game_state.collected_in_wave << ','
                 << "\"requiredPerWave\":" << game_state.required_per_wave << ','
                 << "\"waveElapsedSeconds\":" << game_state.wave_elapsed_seconds << ','
                 << "\"waveTimeLimitSeconds\":" << game_state.wave_time_limit_seconds << ','
                 << "\"elapsedSeconds\":" << game_state.elapsed_seconds << ','
                 << "\"score\":" << game_state.score << ','
                 << "\"endReason\":\"" << game_state.end_reason << "\"}";
        }
        else
        {
            body << "null";
        }
        body << '}';
        return body.str();
    }

    std::string WebServer::environmentJson() const
    {
        std::ostringstream body;
        body << std::fixed << std::setprecision(3)
             << "{\"width\":" << environment_.getWidth() << ','
             << "\"height\":" << environment_.getHeight() << ','
             << "\"resolution\":" << environment_.getResolution() << ','
             << "\"robotRadius\":" << environment_.getRobotRadius() << ','
             << "\"mapImage\":\"/api/map-image\"}";
        return body.str();
    }

    std::string WebServer::sceneJson() const
    {
        std::ostringstream body;
        body << std::fixed << std::setprecision(3)
             << "{\"environment\":" << environmentJson() << ",\"obstacles\":{";

        body << "\"circles\":[";
        const auto &circles = environment_.getCircleObstacles();
        for (std::size_t i = 0; i < circles.size(); ++i)
        {
            if (i > 0)
                body << ',';
            body << "{\"x\":" << circles[i].center.x
                 << ",\"y\":" << circles[i].center.y
                 << ",\"radius\":" << circles[i].radius << '}';
        }

        body << "],\"rectangles\":[";
        const auto &rectangles = environment_.getRectangleObstacles();
        for (std::size_t i = 0; i < rectangles.size(); ++i)
        {
            if (i > 0)
                body << ',';
            body << "{\"x\":" << rectangles[i].origin.x
                 << ",\"y\":" << rectangles[i].origin.y
                 << ",\"width\":" << rectangles[i].width
                 << ",\"height\":" << rectangles[i].height << '}';
        }

        body << "]},\"station\":";
        if (environment_.getStation().has_value())
        {
            const environment::Station &station = *environment_.getStation();
            body << "{\"x\":" << station.origin.x
                 << ",\"y\":" << station.origin.y
                 << ",\"radius\":" << station.radius << '}';
        }
        else
        {
            body << "null";
        }

        body << ",\"waste\":[";
        if (game_)
        {
            const auto waste = game_->getWaste();
            for (std::size_t i = 0; i < waste.size(); ++i)
            {
                if (i > 0)
                    body << ',';
                body << "{\"x\":" << waste[i]->getPosition().x
                     << ",\"y\":" << waste[i]->getPosition().y
                     << ",\"radius\":" << waste[i]->getRadius()
                     << ",\"type\":\"" << waste[i]->getType()
                     << "\",\"color\":\"" << waste[i]->getColor() << "\"}";
            }
        }
        body << "]}";
        return body.str();
    }

    std::string WebServer::handleCommand(const Request &request) const
    {
        double linear = 0.0;
        double angular = 0.0;
        const std::string source = request.query + "&" + request.body;

        extractNumber(source, "linear", linear);
        extractNumber(source, "angular", angular);

        robot_.setVelocity({linear, angular});
        return textResponse(200, "OK", "{\"ok\":true}\n", "application/json");
    }

    std::string WebServer::handleGameRestart() const
    {
        if (!game_)
            return textResponse(409, "Conflict", "{\"ok\":false,\"error\":\"game unavailable\"}\n", "application/json");

        robot_.setVelocity({0.0, 0.0});
        game_->startKeepClean();
        return textResponse(200, "OK", "{\"ok\":true}\n", "application/json");
    }

    std::string WebServer::serveFile(const std::filesystem::path &path) const
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
            return textResponse(404, "Not Found", "Not found\n");

        std::ostringstream body;
        body << file.rdbuf();
        const std::string data = body.str();
        return header(200, "OK", mimeType(path), data.size()) + data;
    }
} // namespace web
