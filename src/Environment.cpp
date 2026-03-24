#include "environment/Environment.h"
#include <opencv2/opencv.hpp>
namespace environment
{
    Environment::Environment(const Config &config)
    {
        resolution_ = config.resolution;
        cv::Mat map = cv::imread(config.map_filename, cv::IMREAD_GRAYSCALE);
        width_ = map.cols;
        height_ = map.rows;
        occupancy_.resize(height_, std::vector<bool>(width_, false));
        for (int y = 0; y < height_; ++y)
        {
            for (int x = 0; x < width_; ++x)
            {
                occupancy_[y][x] = (map.at<uchar>(y, x) < 128);
            }
        }
    }
    bool Environment::isOccupied(double x, double y) const
    {
        int pixel_x = static_cast<int>(x / resolution_);
        int pixel_y = height_ - 1 - static_cast<int>(y / resolution_);
        if (pixel_x < 0 || pixel_x >= width_ || pixel_y < 0 || pixel_y >= height_)
        {
            return true;
        }
        return occupancy_[pixel_y][pixel_x];
    }
    double Environment::getWidth() const
    {
        return width_ * resolution_;
    }
    double Environment::getHeight() const
    {
        return height_ * resolution_;
    }
}
