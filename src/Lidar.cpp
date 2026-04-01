#include <cmath>
#include <numbers>
#include "environment/Lidar.h"

static double deg2rad(double deg) { return deg * std::numbers::pi / 180.0; }

namespace lidar
{

    Lidar::Lidar(const Config &config, std::shared_ptr<environment::Environment> env)
        : config_(config), env_(env)
    {
    }

    std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState &state) const
    {
        std::vector<geometry::Point2d> hits;

        const double step_size = 0.5;

        for (int i = 0; i < config_.beam_count; ++i)
        {
            double ray_angle;
            if (config_.beam_count == 1)
                ray_angle = deg2rad(config_.first_ray_angle);
            else
                ray_angle = deg2rad(config_.first_ray_angle +
                            i * config_.last_ray_angle / (config_.beam_count - 1));

            double absolute_angle = state.theta + ray_angle;
            double dx = std::cos(absolute_angle) * step_size;
            double dy = std::sin(absolute_angle) * step_size;

            double x = state.x;
            double y = state.y;
            double distance = 0.0;

            while (distance < config_.max_range)
            {
                x += dx;
                y += dy;
                distance += step_size;

                if (env_->isOccupied(x, y))
                {
                    hits.push_back({x, y});
                    break;
                }
            }
        }

        return hits;
    }

} // namespace lidar
