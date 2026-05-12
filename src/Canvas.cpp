#include "Canvas.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace canvas
{
    namespace
    {
        constexpr const char *WINDOW_NAME = "Canvas";
        bool highgui_disabled = cv::currentUIFramework().empty();
        bool highgui_warning_printed = false;

        cv::Point toPixel(const cv::Mat &image, double x, double y, double resolution)
        {
            const int px = static_cast<int>(x / resolution);
            const int py = image.rows - 1 - static_cast<int>(y / resolution);
            return {px, py};
        }
    }

    Canvas::Canvas(const std::string &map_filename, double resolution)
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
        cv::circle(image_, toPixel(image_, x, y, resolution_), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }

    void Canvas::drawRobot(double x, double y)
    {
        const int radius_pixels = 8;
        cv::circle(image_, toPixel(image_, x, y, resolution_), radius_pixels, cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
    }

    void Canvas::drawRobot(double x, double y, double theta, double radius)
    {
        const int radius_pixels = std::max(1, static_cast<int>(std::round(radius / resolution_)));
        cv::circle(image_, toPixel(image_, x, y, resolution_), radius_pixels, cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
        const cv::Point center = toPixel(image_, x, y, resolution_);
        const int px = center.x;
        const int py = center.y;
        const int heading_length = std::max(radius_pixels + 4, static_cast<int>(radius_pixels * 1.4));
        int ex = px + static_cast<int>(heading_length * std::cos(theta));
        int ey = py - static_cast<int>(heading_length * std::sin(theta));
        cv::arrowedLine(image_, {px, py}, {ex, ey}, cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
    }

    void Canvas::drawLidarPoints(const std::vector<geometry::Point2d> &points)
    {
        for (const auto &pt : points)
            cv::circle(image_, toPixel(image_, pt.x, pt.y, resolution_), 3, cv::Scalar(0, 50, 255), cv::FILLED, cv::LINE_AA);
    }

    void Canvas::reset()
    {
        base_image_.copyTo(image_);
    }

    void Canvas::drawRays(double x, double y, const std::vector<geometry::Point2d> &hits)
    {
        const cv::Point from = toPixel(image_, x, y, resolution_);
        for (const auto &h : hits)
        {
            cv::line(image_, from, toPixel(image_, h.x, h.y, resolution_), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }

    void Canvas::drawCircleObstacle(const environment::CircleObstacle &obstacle)
    {
        const cv::Point center = toPixel(image_, obstacle.center.x, obstacle.center.y, resolution_);
        const int radius = static_cast<int>(obstacle.radius / resolution_);
        cv::circle(image_, center, radius, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    }

    void Canvas::drawRectangleObstacle(const environment::RectangleObstacle &obstacle)
    {
        const cv::Point top_left = toPixel(image_, obstacle.origin.x, obstacle.origin.y + obstacle.height, resolution_);
        const cv::Point bottom_right = toPixel(image_, obstacle.origin.x + obstacle.width, obstacle.origin.y, resolution_);
        cv::rectangle(image_, top_left, bottom_right, cv::Scalar(255, 140, 0), 2, cv::LINE_AA);
    }

    void Canvas::drawStation(const environment::Station &station)
    {
        const cv::Point station_center = toPixel(image_, station.origin.x, station.origin.y, resolution_);
        const int radius_pixels = std::max(1, static_cast<int>(std::round(station.radius / resolution_)));
        cv::circle(image_, station_center, radius_pixels, cv::Scalar(0, 180, 0), cv::FILLED, cv::LINE_AA);
    }

    int Canvas::show(int wait_ms) const
    {
        if (highgui_disabled)
        {
            if (!highgui_warning_printed)
            {
                highgui_warning_printed = true;
                std::cerr << "OpenCV HighGUI backend is unavailable; running without canvas window.\n";
            }
            return -1;
        }

        if (image_.empty())
        {
            std::cerr << "error\n";
            return -1;
        }

        try
        {
            cv::imshow(WINDOW_NAME, image_);
            return cv::waitKeyEx(wait_ms);
        }
        catch (const cv::Exception &exception)
        {
            highgui_disabled = true;
            if (!highgui_warning_printed)
            {
                highgui_warning_printed = true;
                std::cerr << "OpenCV HighGUI backend is unavailable; running without canvas window.\n";
            }
            return -1;
        }
    }

}
