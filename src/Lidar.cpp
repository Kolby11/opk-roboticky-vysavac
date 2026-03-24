#include "environment/Lidar.h"
#include <cmath>

namespace lidar {

Lidar::Lidar(const Config& config, std::shared_ptr<environment::Environment> env)
    : config(config), env(env)
{}

std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState& state) const {

    std::vector<geometry::Point2d> points;

    if (config.beam_count <= 0) {
        return points;
    }

    double angle_step = 0.0;
    if (config.beam_count > 1) {
        angle_step = (config.last_ray_angle - config.first_ray_angle) / (config.beam_count - 1);
    }

    double step = env->getResolution(); // шаг по лучу

    for (int i = 0; i < config.beam_count; ++i) {

        double angle = state.theta + config.first_ray_angle + i * angle_step;

        for (double r = 0.0; r <= config.max_range; r += step) {

            double x = state.x + r * cos(angle);
            double y = state.y + r * sin(angle);

            if (env->isOccupied(x, y)) {
                points.push_back({x, y});
                break;
            }
        }
    }

    return points;
}

}