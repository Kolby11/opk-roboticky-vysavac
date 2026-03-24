#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "types/Geometry.h"

class Canvas {
public:
    Canvas(const std::string& filename);

    void clear();
    void show();

    void drawPoint(double x, double y);
    void drawLine(double x1, double y1, double x2, double y2);
    void drawLidar(const std::vector<geometry::Point2d>& points);

private:
    cv::Mat map;
    cv::Mat display;
};