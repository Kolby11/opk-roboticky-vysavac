#pragma once

#include <string>

#include <opencv2/opencv.hpp>

namespace environment
{

    struct Config
    {
        std::string map_filename;
        double resolution;
    };

    class Environment
    {
    public:
        cv::Mat map;
        double resolution;

        explicit Environment(const Config &config);
        void LoadMap(const std::string &map_filename, const double resolution);

        bool isOccupied(double x, double y) const;

        double getWidth() const;
        double getHeight() const;

    private:
        void LoadMap();
    };

} // namespace environment
