#pragma once

namespace geometry
{

    struct Point2d
    {
        double x{0.0};
        double y{0.0};
    };

    struct Twist
    {
        double linear{0.0};
        double angular{0.0};
    };

    struct RobotState
    {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
        Twist velocity;
    };

} // namespace geometry
