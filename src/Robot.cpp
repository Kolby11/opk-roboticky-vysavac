#include "robot/Robot.h"
#include <cmath>
#include <algorithm>

namespace robot
{

    Robot::Robot(const Config &config, const CollisionCb &collision_cb)
        : config_(config),
          collision_cb_(collision_cb),
          state_(config.initial_state),
          in_collision_(false),
          target_velocity_{0.0, 0.0},
          command_deadline_(std::chrono::steady_clock::now()),
          running_(true)
    {
        simulation_thread_ = std::thread(&Robot::simulationLoop, this);
    }

    Robot::~Robot()
    {
        running_ = false;
        if (simulation_thread_.joinable())
            simulation_thread_.join();
    }

    void Robot::setVelocity(const geometry::Twist &velocity)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_velocity_ = velocity;
        command_deadline_ = std::chrono::steady_clock::now() +
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::duration<double>(config_.command_duration));
    }

    geometry::RobotState Robot::getState() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    bool Robot::isInCollision() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return in_collision_;
    }

    void Robot::update(const geometry::Twist &velocity, double dt)
    {
        const double linear_step = velocity.linear * dt;
        const double angular_step = velocity.angular * dt;
        const int substeps = std::max(1, static_cast<int>(std::ceil(std::abs(linear_step) / 0.25)));
        const double sub_dt = dt / substeps;

        for (int i = 0; i < substeps; ++i)
        {
            double new_x = state_.x + velocity.linear * std::cos(state_.theta) * sub_dt;
            double new_y = state_.y + velocity.linear * std::sin(state_.theta) * sub_dt;
            double new_theta = state_.theta + velocity.angular * sub_dt;

            if (collision_cb_)
            {
                geometry::RobotState test{new_x, new_y, new_theta, velocity};
                if (collision_cb_(test))
                {
                    in_collision_ = true;
                    return;
                }
                in_collision_ = false;
            }

            state_.x = new_x;
            state_.y = new_y;
            state_.theta = new_theta;
        }
    }

    void Robot::simulationLoop()
    {
        const double dt = config_.simulation_period_ms / 1000.0;

        while (running_)
        {
            auto loop_start = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(mutex_);

                bool command_active = std::chrono::steady_clock::now() < command_deadline_;
                geometry::Twist desired = command_active ? target_velocity_ : geometry::Twist{0.0, 0.0};

                const geometry::Twist &accel = command_active
                                                   ? config_.accelerations
                                                   : config_.emergency_decelerations;

                auto clamp = [](double current, double target, double max_delta)
                {
                    double diff = target - current;
                    double step = std::clamp(diff, -max_delta, max_delta);
                    return current + step;
                };

                geometry::Twist new_velocity;
                new_velocity.linear = clamp(state_.velocity.linear, desired.linear, accel.linear * dt);
                new_velocity.angular = clamp(state_.velocity.angular, desired.angular, accel.angular * dt);

                state_.velocity = new_velocity;
                update(state_.velocity, dt);
            }

            std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(config_.simulation_period_ms));
        }
    }

} // namespace robot
