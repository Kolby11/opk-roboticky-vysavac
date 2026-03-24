#include "environment/Lidar.h"
#include <cmath>
namespace lidar
{
    Lidar::Lidar(const Config &config, std::shared_ptr<environment::Environment> env)
    {
        config_ = config;
        env_ = env;
    }

    std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState &state) const
    {
        std::vector<geometry::Point2d> points;

        for (int i = 0; i < config_.beam_count; i++)
        {
            double angle = state.theta + config_.first_ray_angle +
                           i * (config_.last_ray_angle - config_.first_ray_angle) / (config_.beam_count - 1);
        
            for (double r = 0.0; r <= config_.max_range; r += 0.1)
            {
                double x = state.x + r * cos(angle);
                double y = state.y + r * sin(angle);

                if (env_->isOccupied(x, y))
                {
                    points.push_back({x, y});
                    break;
                }
                
            }
        }
        return points;
    }
}