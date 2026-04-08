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
