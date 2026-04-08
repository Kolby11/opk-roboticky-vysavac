#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "types/Geometry.h"

namespace canvas
{

    class Canvas
    {
    public:
        Canvas(const std::string &map_filename, double resolution);
        void drawPoint(double x, double y);
        void drawRobot(double x, double y);
        void drawRobot(double x, double y, double theta);
        void drawLidarPoints(const std::vector<geometry::Point2d> &points);
        void drawRays(double x, double y, const std::vector<geometry::Point2d> &hits);
        void reset();
        void show() const;

    private:
        cv::Mat base_image_;
        cv::Mat image_;
        double resolution_;
    };
}
