#include "visualization/OpenCvSceneRenderer.h"

#include "Canvas.h"

namespace visualization
{
    class OpenCvSceneRenderer::Impl
    {
    public:
        Impl(const environment::Environment &environment, const lidar::Lidar &lidar)
            : environment_(environment),
              lidar_(lidar),
              canvas_(environment.getMapFilename(), environment.getResolution())
        {
        }

        bool render(const geometry::RobotState &state)
        {
            const auto hits = lidar_.scan(state);

            canvas_.reset();
            for (const auto &obstacle : environment_.getCircleObstacles())
                canvas_.drawCircleObstacle(obstacle);
            for (const auto &obstacle : environment_.getRectangleObstacles())
                canvas_.drawRectangleObstacle(obstacle);
            if (environment_.getStation().has_value())
                canvas_.drawStation(*environment_.getStation());

            canvas_.drawRays(state.x, state.y, hits);
            canvas_.drawLidarPoints(hits);
            canvas_.drawRobot(state.x, state.y, state.theta, environment_.getRobotRadius());

            last_key_ = canvas_.show();
            return true;
        }

        int lastKey() const { return last_key_; }

    private:
        const environment::Environment &environment_;
        const lidar::Lidar &lidar_;
        canvas::Canvas canvas_;
        int last_key_{-1};
    };

    OpenCvSceneRenderer::OpenCvSceneRenderer(const environment::Environment &environment, const lidar::Lidar &lidar)
        : impl_(std::make_unique<Impl>(environment, lidar))
    {
    }

    OpenCvSceneRenderer::~OpenCvSceneRenderer() = default;

    bool OpenCvSceneRenderer::render(const geometry::RobotState &state)
    {
        return impl_->render(state);
    }

    int OpenCvSceneRenderer::lastKey() const
    {
        return impl_->lastKey();
    }
} // namespace visualization
