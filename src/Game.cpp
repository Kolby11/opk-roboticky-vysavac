#include "game/Game.h"

#include <algorithm>
#include <cmath>
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
    } // namespace

    Game::Game(const environment::Environment &environment)
        : Game(environment, std::random_device{}())
    {
    }

    Game::Game(const environment::Environment &environment, unsigned int seed)
        : environment_(environment),
          random_(seed)
    {
        state_.mode = drawMode();
        state_.max_capacity = environment_.getMaxRobotCapacity();
    }

    GameMode Game::getMode() const { return state_.mode; }
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

    void Game::updateRobotState(const geometry::RobotState &robot_state)
    {
        state_.path.push_back(robotPoint(robot_state));
        collectReachableWaste(robot_state);
        unloadIfInStation(robot_state);
    }

    GameMode Game::drawMode()
    {
        std::uniform_int_distribution<int> mode_distribution(0, 2);
        switch (mode_distribution(random_))
        {
        case 0:
            return GameMode::KeepClean;
        case 1:
            return GameMode::BestWorker;
        default:
            return GameMode::Duel;
        }
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
        const int samples = 12;
        for (int i = 0; i < samples; ++i)
        {
            const double angle = 2.0 * std::numbers::pi * i / samples;
            const double x = position.x + radius * std::cos(angle);
            const double y = position.y + radius * std::sin(angle);
            if (environment_.isOccupied(x, y))
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

        for (auto it = waste_.begin(); it != waste_.end() && state_.current_capacity < state_.max_capacity;)
        {
            const Waste &item = **it;
            if (distance(robot_position, item.getPosition()) <= robot_radius + item.getRadius())
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
