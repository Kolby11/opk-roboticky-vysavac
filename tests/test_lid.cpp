#include <gtest/gtest.h>
#include "environment/Lidar.h"
#include "environment/Environment.h"
#include "types/Geometry.h"

TEST(LidarTest, ScanReturnsPointsOnFreePosition)
{
      environment::Config env_con{
         "../resources/opk-map.png",
         0.05
    };
     lidar::Config lidar_con {
        100,
        360,
        1.5,
        -1.5
     };
     auto env = std::make_shared<environment::Environment>(env_con);

     lidar::Lidar lidar(lidar_con, env);
    geometry::RobotState rob{2.0, 2.0, 0.0, {0.0, 0.0}};
    auto points = lidar.scan(rob);
    EXPECT_FALSE(points.empty());
}

TEST(LidarTest, ZeroBeamsReturnsEmpty)
{
     environment::Config env_con{
         "../resources/opk-map.png",
         0.05
    };
     lidar::Config lidar_con {
        100,
        360,
        1.5,
        -1.5
     };
     auto env = std::make_shared<environment::Environment>(env_con);
    lidar::Lidar lidar(lidar_con, env);
    geometry::RobotState rob{2.0, 2.0, 0.0, {0.0, 0.0}};
    auto points = lidar.scan(rob);
    EXPECT_TRUE(points.empty());
}