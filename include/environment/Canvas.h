#pragma once
#include <opencv2/opencv.hpp>
#include "Geometry.h"
#include <vector>

class Canvas
{
private:
    cv::Mat map_;
    double resolution_;

public:
    Canvas(const std::string &map_file, double resolution);
    void drawPoint(const geometry::Point2d &p);
    void drawHits(const std::vector<geometry::Point2d> &hits);
    void show();
};