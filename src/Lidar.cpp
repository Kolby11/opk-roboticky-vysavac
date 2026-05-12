#include <cmath>
#include "robot/lidar.h"

namespace lidar
{

    Lidar::Lidar(const Config &config, std::shared_ptr<environment::Environment> env)
        : config_(config), env_(env)
    {
    }

    std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState &state) const
    {
        std::vector<geometry::Point2d> hits;

        const std::vector<double> ranges = scanRanges(state);

        for (int i = 0; i < config_.beam_count; ++i)
        {
            double ray_angle;
            if (config_.beam_count == 1)
                ray_angle = config_.first_ray_angle;
            else
                ray_angle = config_.first_ray_angle +
                            i * (config_.last_ray_angle - config_.first_ray_angle) /
                                (config_.beam_count - 1);

            double absolute_angle = state.theta + ray_angle;
            if (ranges[i] >= config_.max_range)
                continue;

            hits.push_back({
                state.x + std::cos(absolute_angle) * ranges[i],
                state.y + std::sin(absolute_angle) * ranges[i],
            });
        }

        return hits;
    }

    std::vector<double> Lidar::scanRanges(const geometry::RobotState &state) const
    {
        std::vector<double> ranges;
        ranges.reserve(config_.beam_count);

        const double step_size = 0.5;

        for (int i = 0; i < config_.beam_count; ++i)
        {
            double ray_angle;
            if (config_.beam_count == 1)
                ray_angle = config_.first_ray_angle;
            else
                ray_angle = config_.first_ray_angle +
                            i * (config_.last_ray_angle - config_.first_ray_angle) /
                                (config_.beam_count - 1);

            double absolute_angle = state.theta + ray_angle;
            double dx = std::cos(absolute_angle) * step_size;
            double dy = std::sin(absolute_angle) * step_size;

            double x = state.x;
            double y = state.y;
            double distance = 0.0;
            double measured_range = config_.max_range;

            while (distance < config_.max_range)
            {
                x += dx;
                y += dy;
                distance += step_size;

                if (env_->isOccupied(x, y))
                {
                    measured_range = distance;
                    break;
                }
            }

            ranges.push_back(measured_range);
        }

        return ranges;
    }

    const Config &Lidar::getConfig() const
    {
        return config_;
    }

} // namespace lidar
