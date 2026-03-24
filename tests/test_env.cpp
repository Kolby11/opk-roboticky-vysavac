#include <gtest/gtest.h>
#include "environment/Environment.h"

TEST(EnvironmentTest, OutOfBoundsIsOccupied)
{
    environment::Config env_con{
        "../resources/opk-map.png",
        0.05
    };
    environment::Environment env(env_con);
    EXPECT_TRUE(env.isOccupied(-999.0, -999.0));
    EXPECT_TRUE(env.isOccupied(9999.0, 9999.0));
}
TEST(EnvironmentTest, NonExistentFileDoesNotCrash)
{
    environment::Config cfg;
    cfg.map_filename = "nonexistent.png";
    cfg.resolution = 0.05;
    EXPECT_NO_THROW(environment::Environment env(cfg));
}
