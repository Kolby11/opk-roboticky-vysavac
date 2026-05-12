#pragma once

#include "types/Geometry.h"

namespace visualization
{
    class SceneRenderer
    {
    public:
        virtual ~SceneRenderer() = default;

        // Returns false when the renderer requests the simulator to stop.
        virtual bool render(const geometry::RobotState &state) = 0;
    };
} // namespace visualization
