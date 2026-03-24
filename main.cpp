#include "Environment.h"
#include "Lidar.h"
#include "Canvas.h"
#include <opencv2/opencv.hpp>
#include <memory>

using namespace geometry;

std::shared_ptr<environment::Environment> env;
std::shared_ptr<lidar::Lidar> lidar_sensor;
Canvas* canvas_ptr;
double resolution;

void mouseCallback(int event, int x, int y, int, void*)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        RobotState state;
        state.x=x*resolution;
        state.y=y*resolution;
        state.theta=0;
        auto hits=lidar_sensor->scan(state);
        canvas_ptr->drawPoint({state.x,state.y});
        canvas_ptr->drawHits(hits);
        canvas_ptr->show();
    }
}

int main()
{
environment::Config env_cfg;
env_cfg.map_filename = "opk-map.png";
env_cfg.resolution = 0.1;
resolution = env_cfg.resolution;
env = std::make_shared<environment::Environment>(env_cfg);

lidar::Config lidar_cfg;
lidar_cfg.max_range = 10.0;
lidar_cfg.beam_count = 100;
lidar_cfg.first_ray_angle = 0;
lidar_cfg.last_ray_angle = 180;

lidar_sensor =std::make_shared<lidar::Lidar>(lidar_cfg,env);

Canvas canvas("opk-map.png",resolution);
canvas_ptr = &canvas;
cv::namedWindow("simulation", cv::WINDOW_NORMAL);
cv::resizeWindow("simulation", 800, 600);
cv::setMouseCallback("simulation",mouseCallback);
canvas.show();

while(true)
{
    if(cv::waitKey(30)==27)
        break;
}

}