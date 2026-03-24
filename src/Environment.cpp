#include "Environment.h"
#include <stdexcept>
using namespace environment;

Environment::Environment(const Config& config)
{
    map_ = cv::imread(config.map_filename, cv::IMREAD_GRAYSCALE);
    if (map_.empty())
        throw std::runtime_error("Map file not found");

    resolution_ = config.resolution;
}
bool Environment::isOccupied(double x,double y) const
{
    int px = static_cast<int>(x/resolution_);
    int py = static_cast<int>(y/resolution_);

    if (px<0||py<0||px>=map_.cols||py>= map_.rows)
        return true;
    return map_.at<uchar>(py,px) == 0;
}
double Environment::getWidth() const
{
    return map_.cols*resolution_;
}
double Environment::getHeight() const
{
    return map_.rows*resolution_;
}