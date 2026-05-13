#pragma once

#include "types/Geometry.h"

namespace game
{
    class CircleCollider
    {
    public:
        CircleCollider(geometry::Point2d center, double radius)
            : center_(center),
              radius_(radius)
        {
        }

        const geometry::Point2d &getCenter() const { return center_; }
        double getRadius() const { return radius_; }

        bool intersects(const CircleCollider &other) const
        {
            const double dx = center_.x - other.center_.x;
            const double dy = center_.y - other.center_.y;
            const double radius_sum = radius_ + other.radius_;
            return dx * dx + dy * dy <= radius_sum * radius_sum;
        }

    private:
        geometry::Point2d center_;
        double radius_;
    };
} // namespace game
