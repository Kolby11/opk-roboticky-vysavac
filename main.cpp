#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

#include "environment/Environment.h"
#include "environment/Lidar.h"
#include "Canvas.h"
#include "types/Geometry.h"

struct ScanRecord {
    geometry::Point2d robot_pixel;
    std::vector<geometry::Point2d> hit_pixels;
};

std::shared_ptr<environment::Environment> g_env;
lidar::Lidar* g_lidar = nullptr;
Canvas* g_canvas = nullptr;
environment::Config g_env_config;

std::vector<ScanRecord> g_scans;

void redrawAll() {
    g_canvas->clear();

    for (const auto& scan : g_scans) {
        for (const auto& p : scan.hit_pixels) {
            g_canvas->drawLine(scan.robot_pixel.x, scan.robot_pixel.y, p.x, p.y);
        }

        g_canvas->drawLidar(scan.hit_pixels);
        g_canvas->drawPoint(scan.robot_pixel.x, scan.robot_pixel.y);
    }

    g_canvas->show();
}

void onMouse(int event, int x, int y, int, void*) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }

    if (!g_lidar || !g_canvas || !g_env) {
        return;
    }

    double mx = x * g_env_config.resolution;
    double my = y * g_env_config.resolution;

    if (g_env->isOccupied(mx, my)) {
        return;
    }

    geometry::RobotState state;
    state.x = mx;
    state.y = my;
    state.theta = 0.0;
    state.velocity = {0.0, 0.0};

    auto hits = g_lidar->scan(state);

    ScanRecord record;
    record.robot_pixel = {static_cast<double>(x), static_cast<double>(y)};

    for (const auto& p : hits) {
        double px = p.x / g_env_config.resolution;
        double py = p.y / g_env_config.resolution;
        record.hit_pixels.push_back({px, py});
    }

    g_scans.push_back(record);
    redrawAll();
}

int main() {
    try {
        environment::Config env_config;
        env_config.map_filename = "../resources/opk-map.png";
        env_config.resolution = 0.1;

        auto env = std::make_shared<environment::Environment>(env_config);

        lidar::Config lidar_config;
        lidar_config.max_range = 60.0;
        lidar_config.beam_count = 160;
        lidar_config.first_ray_angle = -M_PI;
        lidar_config.last_ray_angle = M_PI;

        lidar::Lidar lidar(lidar_config, env);
        Canvas canvas(env_config.map_filename);

        g_env = env;
        g_lidar = &lidar;
        g_canvas = &canvas;
        g_env_config = env_config;

        cv::namedWindow("Map");
        cv::setMouseCallback("Map", onMouse);

        redrawAll();

        while (true) {
            int key = cv::waitKey(30);
            if (key == 27) {
                break;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}