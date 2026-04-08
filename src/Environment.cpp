#include "environment/Environment.h"
#include <stdexcept>

namespace environment
{
    Environment::Environment(const Config &config)
        : resolution_(config.resolution)
    {
        map_ = cv::imread(config.map_filename, cv::IMREAD_GRAYSCALE);
        if (map_.empty())
            throw std::runtime_error("Failed to load map: " + config.map_filename);
    }

    bool Environment::isOccupied(double x, double y) const
    {
        int col = static_cast<int>(x / resolution_);
        int row = map_.rows - 1 - static_cast<int>(y / resolution_);

        if (col < 0 || row < 0 || col >= map_.cols || row >= map_.rows)
            return true;

        return map_.at<uchar>(row, col) < 128;
    }

    double Environment::getWidth() const  { return map_.cols * resolution_; }
    double Environment::getHeight() const { return map_.rows * resolution_; }

} // namespace environment
