#include "environment/Environment.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

namespace environment {

struct MouseData {
    cv::Mat map;
    std::vector<cv::Point> robots;
    std::string window_name;
};

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        MouseData* data = (MouseData*)userdata;
        data->robots.push_back(cv::Point(x, y));
        
        cv::Mat temp_view = data->map.clone();
        for (const auto& r : data->robots) {
            cv::circle(temp_view, r, 10, cv::Scalar(0, 255, 0), -1);
        }
        cv::imshow(data->window_name, temp_view);
    }
}

Environment::Environment(const Config& config) {
    map_data = cv::imread(config.map_filename, cv::IMREAD_COLOR);

    if (map_data.empty()) {
        std::cerr << "CHYBA: Subor sa nenacital!" << std::endl;
        return;
    }

    std::string win_name = "Simulator robota";
    cv::namedWindow(win_name);
    static MouseData data; 
    data.map = map_data;
    data.window_name = win_name;
    data.robots.clear(); 

    cv::setMouseCallback(win_name, onMouse, &data);

    //vykreslenie mapy
    cv::imshow(win_name, map_data);

    cv::waitKey(0); 
}

//zatial nepouzite
bool Environment::isOccupied(double x, double y) const { return false; }
double Environment::getWidth() const { return map_data.cols; }
double Environment::getHeight() const { return map_data.rows; }

}