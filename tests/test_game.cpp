#include <gtest/gtest.h>

#include <filesystem>

#include "game/Collider.h"
#include "game/Game.h"
#include "game/GameException.h"

namespace
{
    environment::Config makeConfig()
    {
        environment::Config config;
        config.map_filename = TEST_RESOURCES_DIR "/opk-map.png";
        config.resolution = 0.1;
        config.robot_radius = 1.0;
        config.max_robot_capacity = 2;
        config.station = environment::Station{{10.0, 10.0}, 2.0};
        config.waste_radius = {0.5, 1.0};
        config.waste_types = {
            {"sklo", "green"},
            {"papier", "blue"},
            {"plast", "yellow"}};
        config.keep_clean.result_file = "/tmp/opk_keep_clean_test_result.txt";
        return config;
    }
} // namespace

TEST(WasteFactoryTest, CreatesSpecializedWasteTypes)
{
    const auto paper = game::WasteFactory::create({"papier", "blue"}, {1.0, 2.0}, 0.5);
    const auto plastic = game::WasteFactory::create({"plast", "yellow"}, {1.0, 2.0}, 0.5);
    const auto glass = game::WasteFactory::create({"sklo", "green"}, {1.0, 2.0}, 0.5);

    EXPECT_EQ(paper->getType(), "papier");
    EXPECT_EQ(plastic->getType(), "plast");
    EXPECT_EQ(glass->getType(), "sklo");
}

TEST(ColliderTest, DetectsCircleIntersection)
{
    const game::CircleCollider robot({5.0, 5.0}, 1.0);
    const game::CircleCollider reachable_waste({6.4, 5.0}, 0.5);
    const game::CircleCollider unreachable_waste({6.6, 5.0}, 0.5);

    EXPECT_TRUE(robot.intersects(reachable_waste));
    EXPECT_FALSE(robot.intersects(unreachable_waste));
}

TEST(GameTest, GeneratesConfiguredWaste)
{
    environment::Environment environment(makeConfig());
    game::Game game(environment, 123);

    game.generateWaste(3);

    EXPECT_EQ(game.getWaste().size(), 3u);
}

TEST(GameTest, CollectsWasteUpToCapacity)
{
    environment::Environment environment(makeConfig());
    game::Game game(environment, 123);

    game.addWaste(game::WasteFactory::create({"papier", "blue"}, {5.0, 5.0}, 0.5));
    game.addWaste(game::WasteFactory::create({"plast", "yellow"}, {5.2, 5.0}, 0.5));
    game.addWaste(game::WasteFactory::create({"sklo", "green"}, {5.4, 5.0}, 0.5));

    game.updateRobotState({5.0, 5.0, 0.0, {0.0, 0.0}});

    EXPECT_EQ(game.getState().current_capacity, 2);
    EXPECT_EQ(game.getWaste().size(), 1u);
    EXPECT_EQ(game.getState().collected_by_type.at("papier"), 1);
    EXPECT_EQ(game.getState().collected_by_type.at("plast"), 1);
}

TEST(GameTest, UnloadsWasteAtStationAndScores)
{
    environment::Environment environment(makeConfig());
    game::Game game(environment, 123);

    game.addWaste(game::WasteFactory::create({"papier", "blue"}, {5.0, 5.0}, 0.5));
    game.updateRobotState({5.0, 5.0, 0.0, {0.0, 0.0}});
    game.updateRobotState({10.0, 10.0, 0.0, {0.0, 0.0}});

    EXPECT_EQ(game.getState().current_capacity, 0);
    EXPECT_EQ(game.getState().score, 1);
    EXPECT_EQ(game.getState().delivered_by_type.at("papier"), 1);
    ASSERT_EQ(game.getState().path.size(), 2u);
}

TEST(GameTest, ThrowsWhenWasteTypesAreMissing)
{
    auto config = makeConfig();
    config.waste_types.clear();
    environment::Environment environment(config);
    game::Game game(environment, 123);

    EXPECT_THROW(game.generateWaste(1), game::GameException);
}

TEST(GameTest, DoesNotGenerateWasteInsideConfiguredObstacles)
{
    auto config = makeConfig();
    config.rectangle_obstacles.push_back({{0.0, 0.0}, 153.6, 102.4});
    config.station.reset();
    environment::Environment environment(config);
    game::Game game(environment, 123);

    EXPECT_THROW(game.generateWaste(1), game::GameException);
}

TEST(GameTest, KeepCleanCompletesAllWavesAndWritesResult)
{
    auto config = makeConfig();
    config.keep_clean.waves = 2;
    config.keep_clean.waste_per_wave = 1;
    config.keep_clean.required_per_wave = 1;
    config.keep_clean.wave_time_limit_seconds = 10.0;
    std::filesystem::remove(config.keep_clean.result_file);

    environment::Environment environment(config);
    game::Game game(environment, 123);

    game.startKeepClean();
    ASSERT_TRUE(game.getState().running);
    ASSERT_EQ(game.getWaste().size(), 1u);

    auto waste = game.getWaste().front();
    game.updateRobotState({waste->getPosition().x, waste->getPosition().y, 0.0, {0.0, 0.0}}, 1.0);
    EXPECT_TRUE(game.getState().running);
    EXPECT_EQ(game.getState().current_wave, 2);

    waste = game.getWaste().front();
    game.updateRobotState({waste->getPosition().x, waste->getPosition().y, 0.0, {0.0, 0.0}}, 2.0);

    EXPECT_TRUE(game.getState().finished);
    EXPECT_TRUE(game.getState().success);
    EXPECT_TRUE(std::filesystem::exists(config.keep_clean.result_file));
}

TEST(GameTest, KeepCleanFailsWhenWaveTimesOut)
{
    auto config = makeConfig();
    config.keep_clean.waves = 1;
    config.keep_clean.waste_per_wave = 1;
    config.keep_clean.required_per_wave = 1;
    config.keep_clean.wave_time_limit_seconds = 5.0;
    std::filesystem::remove(config.keep_clean.result_file);

    environment::Environment environment(config);
    game::Game game(environment, 123);

    game.startKeepClean();
    game.updateRobotState({0.0, 0.0, 0.0, {0.0, 0.0}}, 6.0);

    EXPECT_TRUE(game.getState().finished);
    EXPECT_FALSE(game.getState().success);
    EXPECT_EQ(game.getState().end_reason, "wave time limit exceeded");
    EXPECT_TRUE(std::filesystem::exists(config.keep_clean.result_file));
}
