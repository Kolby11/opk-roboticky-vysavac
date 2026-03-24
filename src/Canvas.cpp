#include "Canvas.h"
#include <stdexcept>

Canvas::Canvas(const std::string& filename) {
    map = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (map.empty()) {
        throw std::runtime_error("Failed to load image");
    }

    cv::cvtColor(map, display, cv::COLOR_GRAY2BGR);
}

void Canvas::clear() {
    cv::cvtColor(map, display, cv::COLOR_GRAY2BGR);
}

void Canvas::show() {
    cv::imshow("Map", display);
}

void Canvas::drawPoint(double x, double y) {
    cv::circle(display, cv::Point((int)x, (int)y), 8, cv::Scalar(0, 255, 0), -1);
}

void Canvas::drawLine(double x1, double y1, double x2, double y2) {
    cv::line(display,
             cv::Point((int)x1, (int)y1),
             cv::Point((int)x2, (int)y2),
             cv::Scalar(255, 0, 0), 1);
}

void Canvas::drawLidar(const std::vector<geometry::Point2d>& points) {
    for (const auto& p : points) {
        cv::circle(display, cv::Point((int)p.x, (int)p.y), 4, cv::Scalar(0, 0, 255), -1);
    }
}