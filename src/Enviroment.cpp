#include "environment/Environment.h"

namespace environment {

    Environment::Environment(const Config& config)
    : config(config)
    {
        map = cv::imread(config.map_filename, cv::IMREAD_GRAYSCALE);

        if (map.empty()) {
            throw std::runtime_error("Failed to load map");
        }
    }

    const cv::Mat& Environment::getMap() const {
        return map;
    }

    double Environment::getResolution() const {
        return config.resolution;
    }

    double Environment::getWidth() const {
        return map.cols * config.resolution;
    }

    double Environment::getHeight() const {
        return map.rows * config.resolution;
    }

    bool Environment::isOccupied(double x, double y) const {

        int px = x / config.resolution;
        int py = y / config.resolution;

        if (px < 0 || py < 0 || px >= this->map.cols || py >= this->map.rows) {
            return true;
        }
    
        return map.at<uchar>(py, px) < 128;
    }
}

