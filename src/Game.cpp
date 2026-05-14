#include "game/Game.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>

#include "game/GameException.h"

namespace game
{
    namespace
    {
        double distance(const geometry::Point2d &a, const geometry::Point2d &b)
        {
            const double dx = a.x - b.x;
            const double dy = a.y - b.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        geometry::Point2d robotPoint(const geometry::RobotState &state)
        {
            return {state.x, state.y};
        }

        bool circleIntersectsRectangle(const geometry::Point2d &center,
                                       double radius,
                                       const environment::RectangleObstacle &rectangle)
        {
            const double closest_x = std::clamp(center.x, rectangle.origin.x, rectangle.origin.x + rectangle.width);
            const double closest_y = std::clamp(center.y, rectangle.origin.y, rectangle.origin.y + rectangle.height);
            return distance(center, {closest_x, closest_y}) <= radius;
        }
    } // namespace

    Game::Game(const environment::Environment &environment)
        : Game(environment, std::random_device{}())
    {
    }

    Game::Game(const environment::Environment &environment, unsigned int seed)
        : environment_(environment),
          random_(seed)
    {
        state_.max_capacity = environment_.getMaxRobotCapacity();
        game_started_at_ = std::chrono::steady_clock::now();
    }

    const GameState &Game::getState() const { return state_; }

    std::vector<const Waste *> Game::getWaste() const
    {
        std::vector<const Waste *> result;
        result.reserve(waste_.size());
        for (const auto &waste : waste_)
            result.push_back(waste.get());
        return result;
    }

    void Game::addWaste(std::unique_ptr<Waste> waste)
    {
        if (!waste)
            throw GameException("Cannot add empty waste object");

        waste_.push_back(std::move(waste));
    }

    void Game::generateWaste(std::size_t count)
    {
        const auto &types = environment_.getWasteTypes();
        if (types.empty())
            throw GameException("Cannot generate waste without configured waste types");

        const auto &radius_config = environment_.getWasteRadius();
        if (radius_config.min <= 0.0 || radius_config.max < radius_config.min)
            throw GameException("Invalid waste radius configuration");

        std::uniform_int_distribution<std::size_t> type_distribution(0, types.size() - 1);
        std::uniform_real_distribution<double> radius_distribution(radius_config.min, radius_config.max);

        for (std::size_t i = 0; i < count; ++i)
        {
            const double radius = radius_distribution(random_);
            const auto &type = types[type_distribution(random_)];
            addWaste(WasteFactory::create(type, drawFreePosition(radius), radius));
        }
    }

    void Game::startKeepClean()
    {
        const environment::KeepCleanConfig &config = environment_.getKeepCleanConfig();
        if (config.waves <= 0)
            throw GameException("Keep Clean requires at least one wave");
        if (config.waste_per_wave <= 0)
            throw GameException("Keep Clean requires at least one waste item per wave");
        if (config.required_per_wave <= 0 || config.required_per_wave > config.waste_per_wave)
            throw GameException("Keep Clean required_per_wave must be between 1 and waste_per_wave");
        if (config.wave_time_limit_seconds <= 0.0)
            throw GameException("Keep Clean requires positive wave time limit");

        waste_.clear();
        state_.running = true;
        state_.finished = false;
        state_.success = false;
        state_.current_wave = 0;
        state_.total_waves = config.waves;
        state_.waste_per_wave = config.waste_per_wave;
        state_.required_per_wave = config.required_per_wave;
        state_.collected_in_wave = 0;
        state_.wave_time_limit_seconds = config.wave_time_limit_seconds;
        state_.elapsed_seconds = 0.0;
        state_.wave_elapsed_seconds = 0.0;
        state_.end_reason.clear();
        state_.current_capacity = 0;
        state_.score = 0;
        state_.collected_by_type.clear();
        state_.delivered_by_type.clear();
        state_.path.clear();

        game_started_at_ = std::chrono::steady_clock::now();
        beginKeepCleanWave(0.0);
    }

    void Game::updateRobotState(const geometry::RobotState &robot_state)
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_seconds = std::chrono::duration<double>(now - game_started_at_).count();
        updateRobotState(robot_state, elapsed_seconds);
    }

    void Game::updateRobotState(const geometry::RobotState &robot_state, double elapsed_seconds)
    {
        const std::size_t waste_before = waste_.size();
        state_.path.push_back(robotPoint(robot_state));
        collectReachableWaste(robot_state);
        unloadIfInStation(robot_state);
        if (state_.running)
        {
            state_.collected_in_wave += static_cast<int>(waste_before - waste_.size());
            updateKeepClean(elapsed_seconds);
        }
    }

    void Game::beginKeepCleanWave(double elapsed_seconds)
    {
        ++state_.current_wave;
        state_.collected_in_wave = 0;
        state_.wave_elapsed_seconds = 0.0;
        wave_started_at_seconds_ = elapsed_seconds;
        generateWaste(static_cast<std::size_t>(state_.waste_per_wave));
    }

    void Game::updateKeepClean(double elapsed_seconds)
    {
        state_.elapsed_seconds = elapsed_seconds;
        state_.wave_elapsed_seconds = elapsed_seconds - wave_started_at_seconds_;

        if (state_.collected_in_wave >= state_.required_per_wave)
        {
            if (state_.current_wave >= state_.total_waves)
            {
                finishKeepClean(true, "completed all waves");
                return;
            }

            beginKeepCleanWave(elapsed_seconds);
            return;
        }

        if (state_.wave_elapsed_seconds > state_.wave_time_limit_seconds)
        {
            finishKeepClean(false, "wave time limit exceeded");
        }
    }

    void Game::finishKeepClean(bool success, const std::string &reason)
    {
        if (state_.finished)
            return;

        state_.running = false;
        state_.finished = true;
        state_.success = success;
        state_.end_reason = reason;
        writeKeepCleanResult();
    }

    void Game::writeKeepCleanResult() const
    {
        std::ofstream output(environment_.getKeepCleanConfig().result_file, std::ios::app);
        if (!output)
            return;

        output << "mode=keep_clean"
               << " success=" << (state_.success ? "true" : "false")
               << " score=" << state_.score
               << " capacity=" << state_.current_capacity
               << " waves_completed=" << (state_.success ? state_.total_waves : std::max(0, state_.current_wave - 1))
               << " total_waves=" << state_.total_waves
               << " elapsed_seconds=" << state_.elapsed_seconds
               << " reason=\"" << state_.end_reason << "\"\n";
    }

    geometry::Point2d Game::drawFreePosition(double radius)
    {
        std::uniform_real_distribution<double> x_distribution(radius, std::max(radius, environment_.getWidth() - radius));
        std::uniform_real_distribution<double> y_distribution(radius, std::max(radius, environment_.getHeight() - radius));

        for (int attempt = 0; attempt < 1000; ++attempt)
        {
            geometry::Point2d point{x_distribution(random_), y_distribution(random_)};
            if (canPlaceWaste(point, radius))
                return point;
        }

        throw GameException("Could not find free position for generated waste");
    }

    bool Game::canPlaceWaste(const geometry::Point2d &position, double radius) const
    {
        if (environment_.isOccupied(position.x, position.y))
            return false;

        const double step = std::max(0.1, environment_.getResolution() * 0.5);
        for (double dy = -radius; dy <= radius; dy += step)
        {
            for (double dx = -radius; dx <= radius; dx += step)
            {
                if (dx * dx + dy * dy > radius * radius)
                    continue;

                if (environment_.isOccupied(position.x + dx, position.y + dy))
                    return false;
            }
        }

        for (const auto &obstacle : environment_.getCircleObstacles())
        {
            if (distance(position, obstacle.center) <= radius + obstacle.radius)
                return false;
        }

        for (const auto &obstacle : environment_.getRectangleObstacles())
        {
            if (circleIntersectsRectangle(position, radius, obstacle))
                return false;
        }

        if (environment_.getStation().has_value())
        {
            const auto &station = *environment_.getStation();
            if (distance(position, station.origin) <= radius + station.radius)
                return false;
        }

        for (const auto &existing : waste_)
        {
            if (distance(position, existing->getPosition()) < radius + existing->getRadius())
                return false;
        }

        return true;
    }

    void Game::collectReachableWaste(const geometry::RobotState &robot_state)
    {
        if (state_.current_capacity >= state_.max_capacity)
            return;

        const geometry::Point2d robot_position = robotPoint(robot_state);
        const double robot_radius = environment_.getRobotRadius();
        const CircleCollider robot_collider(robot_position, robot_radius);

        for (auto it = waste_.begin(); it != waste_.end() && state_.current_capacity < state_.max_capacity;)
        {
            const Waste &item = **it;
            if (robot_collider.intersects(item.getCollider()))
            {
                ++state_.current_capacity;
                ++state_.collected_by_type[item.getType()];
                it = waste_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void Game::unloadIfInStation(const geometry::RobotState &robot_state)
    {
        if (state_.current_capacity == 0 || !environment_.getStation().has_value())
            return;

        const auto &station = *environment_.getStation();
        if (distance(robotPoint(robot_state), station.origin) > station.radius + environment_.getRobotRadius())
            return;

        state_.score += state_.current_capacity;
        state_.current_capacity = 0;
        state_.delivered_by_type = state_.collected_by_type;
    }
} // namespace game
