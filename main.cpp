#include "environment/Environment.h"
#include "environment/Lidar.h"
#include "Canvas.h"
#include <iostream>
#include <memory>
#include <cmath>

int main(){

    environment::Config envCfg;
    envCfg.map_filename = "../resources/opk-map.png";
    envCfg.resolution = 0.05;
    auto env = std::make_shared<environment::Environment>(envCfg);
    std::cout << "Map loaded!" << std::endl;

    lidar::Config lidarCfg;
    lidarCfg.max_range = 100.0;
    lidarCfg.beam_count = 360;
    lidarCfg.first_ray_angle = -3.14;
    lidarCfg.last_ray_angle = 3.14;
    lidar::Lidar lidar(lidarCfg, env);
    std::cout << "Lidar created!" << std::endl;

    canvas::Canvas cv(envCfg.map_filename, envCfg.resolution);
    cv.drawRobot(30.0, 30.0);

    geometry::RobotState rob{
        30.0,
        30.0,
        1.5,
        {0.0, 0.0}
    };
    cv.drawRobot(35.0, 35.0);
    geometry::RobotState rob1{
        35.0,
        35.0,
        1.5,
        {0.0, 0.0}
    };

    std::vector<geometry::Point2d> points = lidar.scan(rob);
    cv.drawLidarPoints(points);
    std::vector<geometry::Point2d> points1 = lidar.scan(rob1);
    cv.drawLidarPoints(points1);

    cv.show();
}
