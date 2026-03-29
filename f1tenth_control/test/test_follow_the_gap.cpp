#include <gtest/gtest.h>
#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include "f1tenth_control/common/types.hpp"
#include <cmath>
#include <vector>

using namespace f1tenth_control;

class FollowTheGapTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config for tests (weighted free-space FTG)
        config_.wheelbase = 0.3302;
        config_.car_width = 0.30;
        config_.max_speed = 2.0;
        config_.min_speed = 1.0;
        config_.speed_full_range = 4.0;
        config_.steer_slowdown_gain = 0.5;
        config_.max_steering = 0.4189;
        config_.steering_gain = 1.0;
        config_.max_steering_rate = 100.0;  // Very high for deterministic tests
        config_.target_ema_alpha = 1.0;     // No smoothing for deterministic tests
        config_.emergency_brake_distance = 0.3;

        // Weighted free-space scoring
        config_.heading_weight = 1.0;
        config_.score_power = 2.0;
        config_.clearance_cone_scale = 1.5;
        config_.min_score_range = 0.3;

        // LiDAR processing
        config_.disparity_threshold = 0.5;
        config_.wall_margin = 0.0;  // Disable for tests
        config_.gap_threshold = 0.8;
        config_.min_gap_width = 0.15;

        // Generic LiDAR preprocessing config
        config_.lidar_config.range_min = 0.1;
        config_.lidar_config.range_max = 12.0;
        config_.lidar_config.angle_min = -constants::PI / 2;
        config_.lidar_config.angle_max = constants::PI / 2;
        config_.lidar_config.apply_median_filter = false;  // Deterministic tests

        ftg_ = std::make_unique<FollowTheGap>(config_);
    }

    // Helper to create a scan with a clear path ahead
    std::vector<float> createOpenScan(size_t num_points) {
        return std::vector<float>(num_points, 8.0f);
    }

    // Helper to create a scan with walls on sides and gap in front
    std::vector<float> createCorridorScan(size_t num_points) {
        std::vector<float> ranges(num_points, 1.0f);  // Walls everywhere
        size_t quarter = num_points / 4;
        for (size_t i = quarter; i < 3 * quarter; ++i) {
            ranges[i] = 6.0f;  // Open in front
        }
        return ranges;
    }

    // Helper to create a scan with obstacle directly ahead
    std::vector<float> createObstacleAheadScan(size_t num_points, float obstacle_dist) {
        std::vector<float> ranges(num_points, 6.0f);
        size_t center = num_points / 2;
        size_t width = num_points / 10;
        for (size_t i = center - width; i <= center + width; ++i) {
            ranges[i] = obstacle_dist;
        }
        return ranges;
    }

    // Standard scan parameters for 180-degree scan with 1-degree resolution
    static constexpr size_t NUM_POINTS = 181;
    static constexpr double ANGLE_MIN = -constants::PI / 2;
    static constexpr double ANGLE_MAX = constants::PI / 2;
    static constexpr double ANGLE_INC = constants::PI / 180.0;

    FTGConfig config_;
    std::unique_ptr<FollowTheGap> ftg_;
};

// =====================================================================
// Basic output tests
// =====================================================================

TEST_F(FollowTheGapTest, ComputeReturnsValidOutput) {
    auto ranges = createOpenScan(NUM_POINTS);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_GT(output.command.speed, 0.0);
    EXPECT_FALSE(output.emergency_stop);
    EXPECT_FALSE(output.processed_scan.filtered_ranges.empty());
}

TEST_F(FollowTheGapTest, OpenPathDrivesStraight) {
    auto ranges = createOpenScan(NUM_POINTS);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    // In a symmetric open path the weighted centroid should be near zero
    EXPECT_NEAR(output.command.steering_angle, 0.0, 0.05);
    EXPECT_GT(output.command.speed, 0.0);
}

TEST_F(FollowTheGapTest, CorridorDrivesStraight) {
    auto ranges = createCorridorScan(NUM_POINTS);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    // Corridor: open in front, walls on sides -> should steer roughly straight
    EXPECT_NEAR(output.command.steering_angle, 0.0, 0.15);
    EXPECT_GT(output.command.speed, 0.0);
}

// =====================================================================
// Emergency stop tests
// =====================================================================

TEST_F(FollowTheGapTest, EmergencyStopWhenObstacleTooClose) {
    // All obstacles very close
    std::vector<float> ranges(NUM_POINTS, 0.05f);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_TRUE(output.emergency_stop);
    EXPECT_DOUBLE_EQ(output.command.speed, 0.0);
}

TEST_F(FollowTheGapTest, EmergencyStopDistanceConfigurable) {
    config_.emergency_brake_distance = 0.5;
    ftg_->setConfig(config_);

    std::vector<float> ranges(NUM_POINTS, 0.4f);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_TRUE(output.emergency_stop);
}

TEST_F(FollowTheGapTest, NoEmergencyStopWhenFarEnough) {
    config_.emergency_brake_distance = 0.1;
    ftg_->setConfig(config_);

    auto ranges = createOpenScan(NUM_POINTS);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_FALSE(output.emergency_stop);
}

// =====================================================================
// Gap detection for visualisation
// =====================================================================

TEST_F(FollowTheGapTest, DetectsGaps) {
    auto ranges = createCorridorScan(NUM_POINTS);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    // Should find at least one gap in the corridor
    EXPECT_FALSE(output.all_gaps.empty());
}

// =====================================================================
// Steering direction tests
// =====================================================================

TEST_F(FollowTheGapTest, SteersTowardOpenSpace) {
    // Gap only on the right (indices 0..quarter), walls elsewhere
    std::vector<float> ranges(NUM_POINTS, 0.5f);  // close walls
    size_t quarter = NUM_POINTS / 4;
    for (size_t i = 0; i < quarter; ++i) {
        ranges[i] = 6.0f;  // Open on the right (negative angles)
    }
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    // Steering angle should be negative (right)
    EXPECT_LT(output.command.steering_angle, -0.01);
}

TEST_F(FollowTheGapTest, SteersLeftWhenGapOnLeft) {
    std::vector<float> ranges(NUM_POINTS, 0.5f);
    size_t three_quarter = 3 * NUM_POINTS / 4;
    for (size_t i = three_quarter; i < NUM_POINTS; ++i) {
        ranges[i] = 6.0f;  // Open on the left (positive angles)
    }
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_GT(output.command.steering_angle, 0.01);
}

TEST_F(FollowTheGapTest, SteeringAngleClamped) {
    // Very extreme gap scenario — steering must stay within max_steering
    std::vector<float> ranges(NUM_POINTS, 0.5f);
    // Only last 10 beams are open
    for (size_t i = NUM_POINTS - 10; i < NUM_POINTS; ++i) {
        ranges[i] = 8.0f;
    }
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_LE(std::abs(output.command.steering_angle), config_.max_steering + 0.001);
}

// =====================================================================
// Speed tests
// =====================================================================

TEST_F(FollowTheGapTest, SpeedWithinLimits) {
    auto ranges = createOpenScan(NUM_POINTS);
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_GE(output.command.speed, config_.min_speed);
    EXPECT_LE(output.command.speed, config_.max_speed);
}

TEST_F(FollowTheGapTest, SpeedReducesWhenTurning) {
    // Open path -> near max speed with small steering
    auto open_ranges = createOpenScan(NUM_POINTS);
    auto open_output = ftg_->compute(open_ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);

    // Gap on one side -> turns -> speed should be <= open-path speed
    std::vector<float> ranges(NUM_POINTS, 0.5f);
    for (size_t i = 0; i < NUM_POINTS / 4; ++i) {
        ranges[i] = 6.0f;
    }
    ftg_->reset();
    auto turn_output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_LE(turn_output.command.speed, open_output.command.speed + 0.01);
}

// =====================================================================
// Configuration tests
// =====================================================================

TEST_F(FollowTheGapTest, ConfigurationCanBeUpdated) {
    auto ranges = createOpenScan(NUM_POINTS);
    auto output1 = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);

    config_.max_speed = 1.5;
    ftg_->setConfig(config_);
    ftg_->reset();
    auto output2 = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_LE(output2.command.speed, 1.5);
}

// =====================================================================
// Empty / degenerate scan tests
// =====================================================================

TEST_F(FollowTheGapTest, HandlesEmptyScan) {
    std::vector<float> empty_ranges;
    auto output = ftg_->compute(empty_ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_TRUE(output.emergency_stop);
    EXPECT_DOUBLE_EQ(output.command.speed, 0.0);
}

TEST_F(FollowTheGapTest, HandlesSinglePoint) {
    std::vector<float> ranges = {5.0f};
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    // Single point: should not crash
    EXPECT_FALSE(output.processed_scan.filtered_ranges.empty());
}

// =====================================================================
// Weighted free-space specific tests
// =====================================================================

TEST_F(FollowTheGapTest, SymmetricScanProducesCenteredSteering) {
    // Perfectly symmetric corridor -> target should be near 0
    std::vector<float> ranges(NUM_POINTS);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        double angle = ANGLE_MIN + i * ANGLE_INC;
        // Symmetric U-shape: close on sides, far in front
        ranges[i] = static_cast<float>(2.0 + 4.0 * std::cos(angle) * std::cos(angle));
    }
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_NEAR(output.command.steering_angle, 0.0, 0.05);
}

TEST_F(FollowTheGapTest, HeadingWeightAffectsStraightPreference) {
    // Gap on the right but heading_weight very high -> should still go more
    // straight than with low heading weight
    std::vector<float> ranges(NUM_POINTS, 2.0f);
    size_t quarter = NUM_POINTS / 4;
    for (size_t i = 0; i < quarter; ++i) {
        ranges[i] = 6.0f;
    }

    config_.heading_weight = 0.1;  // Low: will steer more toward gap
    ftg_->setConfig(config_);
    ftg_->reset();
    auto output_low = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);

    config_.heading_weight = 5.0;  // High: will prefer straight
    ftg_->setConfig(config_);
    ftg_->reset();
    auto output_high = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);

    // With higher heading weight the steering should be less extreme (closer to 0)
    EXPECT_LT(std::abs(output_high.command.steering_angle),
              std::abs(output_low.command.steering_angle) + 0.01);
}

TEST_F(FollowTheGapTest, ResetClearsState) {
    auto ranges = createOpenScan(NUM_POINTS);
    ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    ftg_->reset();
    // After reset, first_compute should be true again -> no crash
    auto output = ftg_->compute(ranges, ANGLE_MIN, ANGLE_MAX, ANGLE_INC);
    EXPECT_FALSE(output.emergency_stop);
}

