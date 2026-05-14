#include <cmath>
#include "robot/lidar.h"

namespace lidar
{

    Lidar::Lidar(const Config &config, std::shared_ptr<environment::Environment> env)
        : config_(config), env_(env)
    {
        if (config_.beam_count <= 0)
            return;

        beams_.reserve(static_cast<std::size_t>(config_.beam_count));
        const double angle_increment = config_.beam_count > 1
                                           ? (config_.last_ray_angle - config_.first_ray_angle) / (config_.beam_count - 1)
                                           : 0.0;

        for (int i = 0; i < config_.beam_count; ++i)
        {
            const double angle = config_.first_ray_angle + i * angle_increment;
            beams_.push_back({std::cos(angle), std::sin(angle)});
        }
    }

    std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState &state) const
    {
        std::vector<geometry::Point2d> hits;
        hits.reserve(beams_.size());

        const double cos_theta = std::cos(state.theta);
        const double sin_theta = std::sin(state.theta);

        for (const Beam &beam : beams_)
        {
            const double direction_x = cos_theta * beam.cos_angle - sin_theta * beam.sin_angle;
            const double direction_y = sin_theta * beam.cos_angle + cos_theta * beam.sin_angle;
            const double range = traceBeam(state.x, state.y, direction_x, direction_y);
            if (range >= config_.max_range)
                continue;

            hits.push_back({
                state.x + direction_x * range,
                state.y + direction_y * range,
            });
        }

        return hits;
    }

    std::vector<double> Lidar::scanRanges(const geometry::RobotState &state) const
    {
        std::vector<double> ranges;
        ranges.reserve(beams_.size());

        const double cos_theta = std::cos(state.theta);
        const double sin_theta = std::sin(state.theta);

        for (const Beam &beam : beams_)
        {
            const double direction_x = cos_theta * beam.cos_angle - sin_theta * beam.sin_angle;
            const double direction_y = sin_theta * beam.cos_angle + cos_theta * beam.sin_angle;
            ranges.push_back(traceBeam(state.x, state.y, direction_x, direction_y));
        }

        return ranges;
    }

    double Lidar::traceBeam(double start_x, double start_y, double direction_x, double direction_y) const
    {
        constexpr double step_size = 0.5;

        double x = start_x;
        double y = start_y;
        double distance = 0.0;

        while (distance < config_.max_range)
        {
            x += direction_x * step_size;
            y += direction_y * step_size;
            distance += step_size;

            if (env_->isOccupied(x, y))
                return distance;
        }

        return config_.max_range;
    }

    const Config &Lidar::getConfig() const
    {
        return config_;
    }

} // namespace lidar
