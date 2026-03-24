#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace environment {

    struct Config {
        std::string map_filename;
        double resolution;
    };

    class Environment {
    public:
        explicit Environment(const Config& config);

        bool isOccupied(double x, double y) const;

        double getWidth() const;
        double getHeight() const;

        const cv::Mat& getMap() const;
        double getResolution() const;

    private:
        Config config;
        cv::Mat map;
    };

} // namespace environment
