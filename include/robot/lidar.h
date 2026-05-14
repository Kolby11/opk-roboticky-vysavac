#pragma once

#include <vector>
#include <memory>
#include "types/Geometry.h"
#include "environment/Environment.h"
#include "robot/Robot.h"

namespace lidar
{

    struct Config
    {
        double max_range;
        int beam_count;
        double first_ray_angle;
        double last_ray_angle;
    };

    class Lidar
    {
    public:
        Lidar(const Config &config, std::shared_ptr<environment::Environment> env);

        std::vector<geometry::Point2d> scan(const geometry::RobotState &state) const;
        std::vector<double> scanRanges(const geometry::RobotState &state) const;
        const Config &getConfig() const;

    private:
        struct Beam
        {
            double cos_angle{1.0};
            double sin_angle{0.0};
        };

        double traceBeam(double start_x, double start_y, double direction_x, double direction_y) const;

        Config config_;
        std::shared_ptr<environment::Environment> env_;
        std::vector<Beam> beams_;
    };

} // namespace lidar
