#include "Lidar.h"
#include <cmath>
using namespace lidar;
Lidar::Lidar(const Config& config,
             std::shared_ptr<environment::Environment> env)
    : config_(config), env_(env)
{
}
std::vector<geometry::Point2d>
Lidar::scan(const geometry::RobotState& state) const
{
    std::vector<geometry::Point2d> hits;
    double step = 0.01;
    for(int i=0;i<config_.beam_count;i++)
    {
        double t = (double)i/(config_.beam_count-1);
        double angle =config_.first_ray_angle +t*(config_.last_ray_angle/config_.beam_count);
        angle += state.theta;

        for(double r=0;r<config_.max_range;r+=step)
        {
            double x = state.x + r*cos(angle);
            double y = state.y + r*sin(angle);
            if(env_->isOccupied(x,y))
            {
                hits.push_back({x,y});
                break;
            }
        }
    }
    return hits;
}