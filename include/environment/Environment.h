#pragma once
#include <vector>
#include <string>

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
        explicit Environment(const Config &config);

        bool isOccupied(double x, double y) const;

        double getWidth() const;
        double getHeight() const;

    private:
        std::vector<std::vector<bool>> occupancy_;
        double resolution_;
        int width_;
        int height_;
    };

}
