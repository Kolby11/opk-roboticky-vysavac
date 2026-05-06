#include <gtest/gtest.h>
#include <stdexcept>
#include "environment/Environment.h"

TEST(EnvironmentTest, InvalidMap)
{
    environment::Config config;
    config.map_filename = TEST_RESOURCES_DIR "/does_not_exist.png";
    config.resolution = 0.1;

    EXPECT_THROW({ environment::Environment env(config); }, std::runtime_error);
}

TEST(EnvironmentTest, IsOccupied)
{
    environment::Config config;
    config.map_filename = TEST_RESOURCES_DIR "/opk-map.png";
    config.resolution = 0.1;

    environment::Environment env(config);

    EXPECT_TRUE(env.isOccupied(-1.0, -1.0));
}

TEST(EnvironmentTest, LoadPrimitiveYamlConfig)
{
    const auto config = environment::Config::fromYamlFile(TEST_RESOURCES_DIR "/environment.yaml");

    EXPECT_EQ(config.map_filename, std::string(TEST_RESOURCES_DIR "/opk-map.png"));
    EXPECT_DOUBLE_EQ(config.resolution, 0.1);

    ASSERT_EQ(config.circle_obstacles.size(), 1u);
    EXPECT_DOUBLE_EQ(config.circle_obstacles[0].center.x, 10.0);
    EXPECT_DOUBLE_EQ(config.circle_obstacles[0].center.y, 12.0);
    EXPECT_DOUBLE_EQ(config.circle_obstacles[0].radius, 2.5);

    ASSERT_EQ(config.rectangle_obstacles.size(), 1u);
    EXPECT_DOUBLE_EQ(config.rectangle_obstacles[0].origin.x, 20.0);
    EXPECT_DOUBLE_EQ(config.rectangle_obstacles[0].origin.y, 21.0);
    EXPECT_DOUBLE_EQ(config.rectangle_obstacles[0].width, 4.0);
    EXPECT_DOUBLE_EQ(config.rectangle_obstacles[0].height, 3.0);

    ASSERT_TRUE(config.waste_station.has_value());
    EXPECT_DOUBLE_EQ(config.waste_station->origin.x, 30.0);
    EXPECT_DOUBLE_EQ(config.waste_station->origin.y, 40.0);
    EXPECT_DOUBLE_EQ(config.waste_station->width, 3.0);
    EXPECT_DOUBLE_EQ(config.waste_station->height, 3.0);
}
