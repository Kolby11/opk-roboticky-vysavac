#include <gtest/gtest.h>
#include <memory>
#include <cmath>

#include "robot/lidar.h"

TEST(LidarTest, BeamReturnsEmptyVector)
{
   environment::Config env_config;
   env_config.map_filename = TEST_RESOURCES_DIR "/opk-map.png";
   env_config.resolution = 0.1;

   auto env = std::make_shared<environment::Environment>(env_config);

   lidar::Config lidar_config;
   lidar_config.max_range = 20.0;
   lidar_config.beam_count = 0;
   lidar_config.first_ray_angle = -M_PI;
   lidar_config.last_ray_angle = M_PI;

   lidar::Lidar lidar(lidar_config, env);

   geometry::RobotState state;
   state.x = 20.0;
   state.y = 20.0;
   state.theta = 0.0;
   state.velocity = {0.0, 0.0};

   auto hits = lidar.scan(state);

   EXPECT_TRUE(hits.empty());
}

TEST(LidarTest, ScanReturnsPoints)
{
   environment::Config env_config;
   env_config.map_filename = TEST_RESOURCES_DIR "/opk-map.png";
   env_config.resolution = 0.1;

   auto env = std::make_shared<environment::Environment>(env_config);
   lidar::Config lidar_config;
   lidar_config.max_range = 20.0;
   lidar_config.beam_count = 180;
   lidar_config.first_ray_angle = -M_PI;
   lidar_config.last_ray_angle = M_PI;

   lidar::Lidar lidar(lidar_config, env);
   geometry::RobotState state;
   state.x = 20.0;
   state.y = 20.0;
   state.theta = 0.0;
   state.velocity = {0.0, 0.0};

   auto hits = lidar.scan(state);

   EXPECT_FALSE(hits.empty());
}
