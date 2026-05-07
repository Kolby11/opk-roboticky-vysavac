#include "environment/Environment.h"
#include "parser/Parser.h"

#include <filesystem>
#include <stdexcept>

namespace environment
{
    namespace
    {
        Config configFromYamlDocument(const YAMLDocument &document, const std::filesystem::path &base_dir);

        const YAMLValue &requireMapEntry(const YAMLValue &node, const std::string &key)
        {
            if (!node.isMap())
                throw std::runtime_error("Expected YAML map");

            const auto it = node.map.find(key);
            if (it == node.map.end())
                throw std::runtime_error("Missing YAML key: " + key);

            return it->second;
        }

        std::string requireScalar(const YAMLValue &node, const std::string &field_name)
        {
            if (!node.isScalar())
                throw std::runtime_error("Expected scalar for: " + field_name);
            return node.scalar;
        }

        double requireDouble(const YAMLValue &node, const std::string &field_name)
        {
            const std::string value = requireScalar(node, field_name);
            try
            {
                std::size_t consumed = 0;
                const double result = std::stod(value, &consumed);
                if (consumed != value.size())
                    throw std::runtime_error("");
                return result;
            }
            catch (const std::exception &)
            {
                throw std::runtime_error("Invalid numeric YAML value for: " + field_name);
            }
        }

        int requireInt(const YAMLValue &node, const std::string &field_name)
        {
            const std::string value = requireScalar(node, field_name);
            try
            {
                std::size_t consumed = 0;
                const int result = std::stoi(value, &consumed);
                if (consumed != value.size())
                    throw std::runtime_error("");
                return result;
            }
            catch (const std::exception &)
            {
                throw std::runtime_error("Invalid integer YAML value for: " + field_name);
            }
        }

        Config configFromYamlDocument(const YAMLDocument &document, const std::filesystem::path &base_dir)
        {
            const YAMLValue &root = document.root;

            Config config;

            const YAMLValue &map_node = requireMapEntry(root, "map");
            config.map_filename = (base_dir / requireScalar(requireMapEntry(map_node, "filename"), "map.filename")).lexically_normal().string();
            config.resolution = requireDouble(requireMapEntry(map_node, "resolution"), "map.resolution");

            const auto obstacles_it = root.map.find("obstacles");
            if (obstacles_it != root.map.end())
            {
                if (!obstacles_it->second.isSequence())
                    throw std::runtime_error("Expected sequence for: obstacles");

                for (const YAMLValue &obstacle_node : obstacles_it->second.sequence)
                {
                    const std::string type = requireScalar(requireMapEntry(obstacle_node, "type"), "obstacles.type");
                    if (type == "circle")
                    {
                        CircleObstacle obstacle;
                        obstacle.center.x = requireDouble(requireMapEntry(obstacle_node, "center_x"), "obstacles.center_x");
                        obstacle.center.y = requireDouble(requireMapEntry(obstacle_node, "center_y"), "obstacles.center_y");
                        obstacle.radius = requireDouble(requireMapEntry(obstacle_node, "radius"), "obstacles.radius");
                        config.circle_obstacles.push_back(obstacle);
                    }
                    else if (type == "rectangle")
                    {
                        RectangleObstacle obstacle;
                        obstacle.origin.x = requireDouble(requireMapEntry(obstacle_node, "x"), "obstacles.x");
                        obstacle.origin.y = requireDouble(requireMapEntry(obstacle_node, "y"), "obstacles.y");
                        obstacle.width = requireDouble(requireMapEntry(obstacle_node, "width"), "obstacles.width");
                        obstacle.height = requireDouble(requireMapEntry(obstacle_node, "height"), "obstacles.height");
                        config.rectangle_obstacles.push_back(obstacle);
                    }
                    else
                    {
                        throw std::runtime_error("Unsupported obstacle type: " + type);
                    }
                }
            }

            const auto station_it = root.map.find("station");
            if (station_it != root.map.end())
            {
                Station station;
                station.origin.x = requireDouble(requireMapEntry(station_it->second, "x"), "station.x");
                station.origin.y = requireDouble(requireMapEntry(station_it->second, "y"), "station.y");
                station.radius = requireDouble(requireMapEntry(station_it->second, "radius"), "station.radius");
                config.station = station;
            }

            const auto robot_it = root.map.find("robot");
            if (robot_it != root.map.end())
            {
                config.robot_radius = requireDouble(requireMapEntry(robot_it->second, "radius"), "robot.radius");
                config.max_robot_capacity = requireInt(requireMapEntry(robot_it->second, "max_capacity"), "robot.max_capacity");
            }

            const auto waste_it = root.map.find("waste");
            if (waste_it != root.map.end())
            {
                const YAMLValue &radius_node = requireMapEntry(waste_it->second, "radius");
                config.waste_radius.min = requireDouble(requireMapEntry(radius_node, "min"), "waste.radius.min");
                config.waste_radius.max = requireDouble(requireMapEntry(radius_node, "max"), "waste.radius.max");

                const YAMLValue &types_node = requireMapEntry(waste_it->second, "types");
                if (!types_node.isSequence())
                    throw std::runtime_error("Expected sequence for: waste.types");

                for (const YAMLValue &type_node : types_node.sequence)
                {
                    WasteType waste_type;
                    waste_type.name = requireScalar(requireMapEntry(type_node, "name"), "waste.types.name");
                    waste_type.color = requireScalar(requireMapEntry(type_node, "color"), "waste.types.color");
                    config.waste_types.push_back(waste_type);
                }
            }

            return config;
        }
    }

    Config Config::fromYamlFile(const std::string &filename)
    {
        const YAMLDocument document = YAMLParser::parseFile(filename);
        return configFromYamlDocument(document, std::filesystem::path(filename).parent_path());
    }

    Environment::Environment(const Config &config)
        : map_filename_(config.map_filename),
          resolution_(config.resolution),
          circle_obstacles_(config.circle_obstacles),
          rectangle_obstacles_(config.rectangle_obstacles),
          station_(config.station),
          robot_radius_(config.robot_radius),
          waste_radius_(config.waste_radius),
          waste_types_(config.waste_types),
          max_robot_capacity_(config.max_robot_capacity)
    {
        map_ = cv::imread(map_filename_, cv::IMREAD_GRAYSCALE);
        if (map_.empty())
            throw std::runtime_error("Failed to load map: " + map_filename_);
    }

    Environment::Environment(const YAMLDocument &document, const std::string &source_filename)
        : Environment(configFromYamlDocument(document, std::filesystem::path(source_filename).parent_path()))
    {
    }

    bool Environment::isOccupied(double x, double y) const
    {
        int col = static_cast<int>(x / resolution_);
        int row = map_.rows - 1 - static_cast<int>(y / resolution_);

        if (col < 0 || row < 0 || col >= map_.cols || row >= map_.rows)
            return true;

        return map_.at<uchar>(row, col) < 128;
    }

    double Environment::getWidth() const { return map_.cols * resolution_; }
    double Environment::getHeight() const { return map_.rows * resolution_; }
    double Environment::getResolution() const { return resolution_; }
    const std::string &Environment::getMapFilename() const { return map_filename_; }
    const std::vector<CircleObstacle> &Environment::getCircleObstacles() const { return circle_obstacles_; }
    const std::vector<RectangleObstacle> &Environment::getRectangleObstacles() const { return rectangle_obstacles_; }
    const std::optional<Station> &Environment::getStation() const { return station_; }
    double Environment::getRobotRadius() const { return robot_radius_; }
    const WasteRadius &Environment::getWasteRadius() const { return waste_radius_; }
    const std::vector<WasteType> &Environment::getWasteTypes() const { return waste_types_; }
    int Environment::getMaxRobotCapacity() const { return max_robot_capacity_; }

} // namespace environment
