#pragma once

#include <memory>

#include "environment/Environment.h"
#include "robot/lidar.h"
#include "visualization/SceneRenderer.h"

namespace visualization
{
    class OpenCvSceneRenderer : public SceneRenderer
    {
    public:
        OpenCvSceneRenderer(const environment::Environment &environment, const lidar::Lidar &lidar);
        ~OpenCvSceneRenderer() override;

        bool render(const geometry::RobotState &state) override;
        int lastKey() const;

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace visualization
