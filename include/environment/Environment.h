#pragma once

#include <optional>
#include <string>
#include <vector>
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
        double radius{0.0};
    };

    struct WasteRadius
    {
        double min{1.0};
        double max{3.0};
    };

    struct WasteType
    {
        std::string name;
        std::string color;
    };

    struct Config
    {
        std::string map_filename;
        double resolution{1.0}; // metres per pixel
        std::vector<CircleObstacle> circle_obstacles;
        std::vector<RectangleObstacle> rectangle_obstacles;
        std::optional<Station> station;
        double robot_radius{8.0};
        WasteRadius waste_radius;
        std::vector<WasteType> waste_types;
        int max_robot_capacity{10};

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
        int getMapWidthPixels() const;
        int getMapHeightPixels() const;
        unsigned char getMapPixel(int x, int y_from_bottom) const;
        double getResolution() const;
        const std::string &getMapFilename() const;
        const std::vector<CircleObstacle> &getCircleObstacles() const;
        const std::vector<RectangleObstacle> &getRectangleObstacles() const;
        const std::optional<Station> &getStation() const;
        double getRobotRadius() const;
        const WasteRadius &getWasteRadius() const;
        const std::vector<WasteType> &getWasteTypes() const;
        int getMaxRobotCapacity() const;

    private:
        std::vector<unsigned char> occupancy_;
        int map_width_pixels_;
        int map_height_pixels_;
        std::string map_filename_;
        double resolution_;
        std::vector<CircleObstacle> circle_obstacles_;
        std::vector<RectangleObstacle> rectangle_obstacles_;
        std::optional<Station> station_;
        double robot_radius_;
        WasteRadius waste_radius_;
        std::vector<WasteType> waste_types_;
        int max_robot_capacity_;
    };

} // namespace environment
