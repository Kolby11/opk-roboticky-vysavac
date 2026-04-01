#include "environment/Environment.h"

namespace environment
{
    Environment::Environment(const Config &config)
    {
        this->resolution = config.resolution;
        this->LoadMap(config.map_filename, config.resolution);
    }

    void Environment::LoadMap(const std::string &map_filename, const double resolution)
    {
        this->map = cv::imread(map_filename);

        if (this->map.empty())
        {
            std::cerr << "Failed to load map: " << map_filename << std::endl;
            return;
        }

        int width = static_cast<int>(this->map.cols * resolution);
        int height = static_cast<int>(this->map.rows * resolution);
        cv::resize(this->map, this->map, cv::Size(width, height));
    }

    bool Environment::isOccupied(double x, double y) const
    {
        int col = static_cast<int>(x);
        int row = static_cast<int>(y);

        if (col < 0 || row < 0 || col >= map.cols || row >= map.rows)
            return true;

        cv::Vec3b pixel = map.at<cv::Vec3b>(row, col);
        return pixel[0] < 128 && pixel[1] < 128 && pixel[2] < 128;
    }

    double Environment::getWidth() const { return map.cols; }
    double Environment::getHeight() const { return map.rows; }

} // namespace environment
