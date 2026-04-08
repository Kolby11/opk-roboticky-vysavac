#include <gtest/gtest.h>
#include <cmath>
#include <chrono>
#include <thread>

#include "robot/Robot.h"

class RobotPhysicsTest : public ::testing::Test
{
protected:
    robot::Config cfg{
        .accelerations = {1000.0, 1000.0},
        .emergency_decelerations = {1000.0, 1000.0},
        .command_duration = 10.0,
        .simulation_period_ms = 10};
};

TEST_F(RobotPhysicsTest, ForwardMovement)
{
    robot::Robot r(cfg);
    r.setVelocity({1.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto s = r.getState();
    EXPECT_GT(s.x, 0.0);
    EXPECT_NEAR(s.y, 0.0, 0.1);
}

TEST_F(RobotPhysicsTest, BackwardMovement)
{
    robot::Robot r(cfg);
    r.setVelocity({-1.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto s = r.getState();
    EXPECT_LT(s.x, 0.0);
}

TEST_F(RobotPhysicsTest, RotationChangesTheta)
{
    robot::Robot r(cfg);
    r.setVelocity({0.0, 1.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto s = r.getState();
    EXPECT_GT(s.theta, 0.0);
}

TEST(RobotTimeoutTest, StopsAfterCommandExpires)
{
    robot::Config cfg{
        .accelerations = {1000.0, 1000.0},
        .emergency_decelerations = {1000.0, 1000.0},
        .command_duration = 0.05,
        .simulation_period_ms = 10};
    robot::Robot r(cfg);
    r.setVelocity({1.0, 0.0});

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    auto s = r.getState();
    EXPECT_NEAR(s.velocity.linear, 0.0, 0.05);
    EXPECT_NEAR(s.velocity.angular, 0.0, 0.05);
}

TEST(RobotCollisionTest, CallbackTriggered)
{
    bool hit = false;
    robot::Config cfg{
        .accelerations = {1000.0, 1000.0},
        .emergency_decelerations = {1000.0, 1000.0},
        .command_duration = 10.0,
        .simulation_period_ms = 10,
        .initial_state = {0.0, 0.0, 0.0, {0.0, 0.0}}};

    robot::Robot r(cfg, [&hit](geometry::RobotState s) -> bool
                   {
        if (s.x > 0.05) { hit = true; return true; }
        return false; });

    r.setVelocity({1.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_TRUE(hit);
    EXPECT_TRUE(r.isInCollision());
}