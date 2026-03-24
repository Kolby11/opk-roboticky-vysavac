#pragma once
#include <opencv2/opencv.hpp>
#include <string>
namespace environment {

struct Config {
    std::string map_filename;
    double resolution;
};

class Environment {
private:
    cv::Mat map_;
    double resolution_;
public:
    explicit Environment(const Config& config);

    bool isOccupied(double x, double y) const;

    double getWidth() const;
    double getHeight() const;
};

} // namespace environment
