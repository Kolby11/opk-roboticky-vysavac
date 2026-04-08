#include "Canvas.h"
#include <cmath>
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
        image_.copyTo(base_image_);
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

    void Canvas::drawRobot(double x, double y, double theta)
    {
        drawRobot(x, y);
        int px = static_cast<int>(x / resolution_);
        int py = image_.rows - 1 - static_cast<int>(y / resolution_);
        int ex = px + static_cast<int>(12 * std::cos(theta));
        int ey = py - static_cast<int>(12 * std::sin(theta));
        cv::arrowedLine(image_, {px, py}, {ex, ey}, cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
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

    void Canvas::reset()
    {
        base_image_.copyTo(image_);
    }

    void Canvas::drawRays(double x, double y, const std::vector<geometry::Point2d> &hits)
    {
        int px = static_cast<int>(x / resolution_);
        int py = image_.rows - 1 - static_cast<int>(y / resolution_);
        cv::Point from(px, py);
        for (const auto &h : hits)
        {
            int hx = static_cast<int>(h.x / resolution_);
            int hy = image_.rows - 1 - static_cast<int>(h.y / resolution_);
            cv::line(image_, from, cv::Point(hx, hy), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
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
    }

}
