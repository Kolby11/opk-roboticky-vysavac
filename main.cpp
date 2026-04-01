#include <iostream>
#include <memory>

#include "Canvas.h"
#include "environment/Environment.h"
#include "environment/Lidar.h"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <map_file>\n";
        return 1;
    }

    environment::Config environment_config = {
        .map_filename = argv[1],
        .resolution = 1.0};

    lidar::Config lidar_config = {
        .max_range = 200,
        .beam_count = 20,
        .first_ray_angle = 0,
        .last_ray_angle = 180,
    };

    auto environment = std::make_shared<environment::Environment>(environment_config);
    auto lidar = std::make_shared<lidar::Lidar>(lidar_config, environment);

    canvas::Canvas canvas(argv[1], environment_config.resolution);

    geometry::RobotState state{static_cast<double>(environment->getWidth() / 2),
                               static_cast<double>(environment->getHeight() / 2),
                               0.0, {0.0, 0.0}};

    auto hits = lidar->scan(state);
    canvas.drawRobot(state.x, state.y);
    canvas.drawLidarPoints(hits);
    canvas.show();

    return 0;
}
