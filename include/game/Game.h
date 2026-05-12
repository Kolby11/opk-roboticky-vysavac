#pragma once

#include <map>
#include <memory>
#include <random>
#include <chrono>
#include <string>
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
        bool running{false};
        bool finished{false};
        bool success{false};
        int current_wave{0};
        int total_waves{0};
        int waste_per_wave{0};
        int required_per_wave{0};
        int collected_in_wave{0};
        double wave_time_limit_seconds{0.0};
        double elapsed_seconds{0.0};
        double wave_elapsed_seconds{0.0};
        std::string end_reason;
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

        void startKeepClean();
        void addWaste(std::unique_ptr<Waste> waste);
        void generateWaste(std::size_t count);
        void updateRobotState(const geometry::RobotState &robot_state);
        void updateRobotState(const geometry::RobotState &robot_state, double elapsed_seconds);

    private:
        const environment::Environment &environment_;
        GameState state_;
        std::vector<std::unique_ptr<Waste>> waste_;
        std::mt19937 random_;
        std::chrono::steady_clock::time_point game_started_at_{};
        double wave_started_at_seconds_{0.0};

        GameMode drawMode();
        void beginKeepCleanWave(double elapsed_seconds);
        void updateKeepClean(double elapsed_seconds);
        void finishKeepClean(bool success, const std::string &reason);
        void writeKeepCleanResult() const;
        geometry::Point2d drawFreePosition(double radius);
        bool canPlaceWaste(const geometry::Point2d &position, double radius) const;
        void collectReachableWaste(const geometry::RobotState &robot_state);
        void unloadIfInStation(const geometry::RobotState &robot_state);
    };
} // namespace game
