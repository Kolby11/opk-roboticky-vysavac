#pragma once

#include <map>
#include <memory>
#include <random>
#include <vector>

#include "environment/Environment.h"
#include "game/Waste.h"
#include "types/Geometry.h"

namespace game
{
    enum class GameMode
    {
        KeepClean,
        BestWorker,
        Duel
    };

    struct GameState
    {
        GameMode mode{GameMode::KeepClean};
        int current_capacity{0};
        int max_capacity{0};
        int score{0};
        std::map<std::string, int> collected_by_type;
        std::map<std::string, int> delivered_by_type;
        std::vector<geometry::Point2d> path;
    };

    class Game
    {
    public:
        explicit Game(const environment::Environment &environment);
        Game(const environment::Environment &environment, unsigned int seed);

        GameMode getMode() const;
        const GameState &getState() const;
        std::vector<const Waste *> getWaste() const;

        void addWaste(std::unique_ptr<Waste> waste);
        void generateWaste(std::size_t count);
        void updateRobotState(const geometry::RobotState &robot_state);

    private:
        const environment::Environment &environment_;
        GameState state_;
        std::vector<std::unique_ptr<Waste>> waste_;
        std::mt19937 random_;

        GameMode drawMode();
        geometry::Point2d drawFreePosition(double radius);
        bool canPlaceWaste(const geometry::Point2d &position, double radius) const;
        void collectReachableWaste(const geometry::RobotState &robot_state);
        void unloadIfInStation(const geometry::RobotState &robot_state);
    };
} // namespace game
