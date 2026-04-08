#pragma once

#include "types/Geometry.h"
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

namespace robot
{
    struct Config
    {
        geometry::Twist accelerations;
        geometry::Twist emergency_decelerations;
        double command_duration;
        int simulation_period_ms;
        geometry::RobotState initial_state{};
    };

    class Robot
    {
    public:
        using CollisionCb = std::function<bool(geometry::RobotState)>;

        Robot(const Config &config, const CollisionCb &collision_cb = nullptr);
        ~Robot();
        void setVelocity(const geometry::Twist &velocity);
        geometry::RobotState getState() const;
        bool isInCollision() const;

    protected:
        void update(const geometry::Twist &velocity, double dt);

    private:
        Config config_;
        CollisionCb collision_cb_;
        geometry::RobotState state_;
        bool in_collision_;
        geometry::Twist target_velocity_;
        std::chrono::steady_clock::time_point command_deadline_;
        std::thread simulation_thread_;
        mutable std::mutex mutex_;
        std::atomic<bool> running_;

        void simulationLoop();
    };
} // namespace robot
