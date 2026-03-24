#include "Canvas.h"
#include <iostream>

namespace canvas
{

    Canvas::Canvas(const std::string& map_filename, double resolution)
        : resolution_(resolution)
    {
        cv::Mat gray = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
        if (gray.empty())
        {
            std::cerr << "error" << map_filename << "\n";
            return;
        }
        cv::cvtColor(gray, image_, cv::COLOR_GRAY2BGR);
        std::cout << "Canvas loaded!" << std::endl;
    }

    void Canvas::drawPoint(double x, double y)
    {
        int px = static_cast<int>(x / resolution_);
        int py = image_.rows - 1 - static_cast<int>(y / resolution_);
        cv::circle(image_, {px, py}, 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }

    void Canvas::drawRobot(double x, double y)
    {
        int px = static_cast<int>(x / resolution_);
        int py = image_.rows - 1 - static_cast<int>(y / resolution_);
        cv::circle(image_, {px, py}, 8, cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
    }

    void Canvas::drawLidarPoints(const std::vector<geometry::Point2d> &points)
    {
        for (const auto &pt : points)
        {
            int px = static_cast<int>(pt.x / resolution_);
            int py = image_.rows - 1 - static_cast<int>(pt.y / resolution_);
            cv::circle(image_, {px, py}, 3, cv::Scalar(0, 50, 255), cv::FILLED, cv::LINE_AA);
        }
    }

    void Canvas::show() const
    {
        if (image_.empty())
        {
            std::cerr << "error\n";
            return;
        }
        cv::imshow("Canvas", image_);
        cv::waitKey(0);
    }

} 