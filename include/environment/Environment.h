#pragma once

#include <optional>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "parser/Parser.h"
#include "types/Geometry.h"

namespace environment
{
    struct CircleObstacle
    {
        geometry::Point2d center;
        double radius{0.0};
    };

    struct RectangleObstacle
    {
        geometry::Point2d origin;
        double width{0.0};
        double height{0.0};
    };

    struct Station
    {
        geometry::Point2d origin;
        double width{0.0};
        double height{0.0};
    };

    struct Config
    {
        std::string map_filename;
        double resolution{1.0}; // metres per pixel
        std::vector<CircleObstacle> circle_obstacles;
        std::vector<RectangleObstacle> rectangle_obstacles;
        std::optional<Station> waste_station;

        static Config fromYamlFile(const std::string &filename);
    };

    class Environment
    {
    public:
        explicit Environment(const Config &config);
        Environment(const YAMLDocument &document, const std::string &source_filename);

        bool isOccupied(double x, double y) const;
        double getWidth() const;
        double getHeight() const;
        double getResolution() const;
        const std::string &getMapFilename() const;
        const std::vector<CircleObstacle> &getCircleObstacles() const;
        const std::vector<RectangleObstacle> &getRectangleObstacles() const;
        const std::optional<Station> &getStation() const;

    private:
        cv::Mat map_;
        std::string map_filename_;
        double resolution_;
        std::vector<CircleObstacle> circle_obstacles_;
        std::vector<RectangleObstacle> rectangle_obstacles_;
        std::optional<Station> station_;
    };

} // namespace environment
