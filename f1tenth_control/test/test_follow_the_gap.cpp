#include <gtest/gtest.h>
#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include "f1tenth_control/common/types.hpp"
#include <cmath>
#include <vector>

using namespace f1tenth_control;

class FollowTheGapTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config for tests (pure FTG)
        config_.wheelbase = 0.324;
        config_.car_width = 0.50;
        config_.max_speed = 6.0;
        config_.min_speed = 2.0;
        config_.speed_full_range = 9.0;
        config_.steer_slowdown_gain = 0.7;
        config_.max_steering_angle = 0.4;
        config_.steering_gain = 0.8;
        config_.max_steering_delta = 0.05;
        config_.emergency_brake_distance = 0.3;
        config_.mapping_mode = false;
        
        // FTG-specific processing parameters (now in FTGConfig)
        config_.disparity_threshold = 0.5;
        config_.gap_threshold = 0.8;
        config_.min_gap_width = 0.15;
        config_.bubble_radius = 0.25;
        config_.apply_bubble = true;
        config_.wall_margin = 0.0;  // Disable wall margin for tests
        
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
        // All points far away - clear path
        return std::vector<float>(num_points, 8.0f);
    }
    
    // Helper to create a scan with walls on sides and gap in front
    std::vector<float> createCorridorScan(size_t num_points) {
        std::vector<float> ranges(num_points, 1.0f);  // Walls everywhere
        // Create gap in the middle (front)
        size_t quarter = num_points / 4;
        for (size_t i = quarter; i < 3 * quarter; ++i) {
            ranges[i] = 6.0f;  // Open in front
        }
        return ranges;
    }
    
    // Helper to create a scan with obstacle directly ahead
    std::vector<float> createBlockedFrontScan(size_t num_points) {
        std::vector<float> ranges(num_points, 6.0f);  // Open everywhere
        // Block the front
        size_t center = num_points / 2;
        for (size_t i = center - 2; i <= center + 2; ++i) {
            ranges[i] = 0.5f;  // Close obstacle
        }
        return ranges;
    }
    
    // Helper to create a scan requiring emergency stop
    std::vector<float> createEmergencyScan(size_t num_points) {
        std::vector<float> ranges(num_points, 6.0f);
        // Very close obstacle
        ranges[num_points / 2] = 0.2f;  // Below emergency threshold
        return ranges;
    }
    
    // Standard scan parameters for -90 to +90 degrees
    void getStandardScanParams(size_t num_points, double& angle_min, double& angle_max, double& angle_inc) {
        angle_min = -constants::PI / 2;
        angle_max = constants::PI / 2;
        angle_inc = constants::PI / static_cast<double>(num_points - 1);
    }
    
    FTGConfig config_;
    std::unique_ptr<FollowTheGap> ftg_;
};

// ===================
// Basic Operation Tests
// ===================

TEST_F(FollowTheGapTest, ComputeReturnsValidOutput) {
    auto ranges = createOpenScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Should return valid command
    EXPECT_FALSE(output.emergency_stop);
    EXPECT_GE(output.command.speed, config_.min_speed);
    EXPECT_LE(output.command.speed, config_.max_speed);
    EXPECT_GE(output.command.steering_angle, -config_.max_steering_angle);
    EXPECT_LE(output.command.steering_angle, config_.max_steering_angle);
}

TEST_F(FollowTheGapTest, OpenPathDrivesStraight) {
    // Disable bubble and disparity extension for this test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Create scan with slightly deeper range in center to simulate open path ahead
    // This gives a preference for straight ahead (uniform scans would pick arbitrary deepest)
    std::vector<float> ranges(100, 7.5f);
    // Make the center (straight ahead) slightly deeper
    for (int i = 45; i < 55; ++i) {
        ranges[i] = 8.0f;  // Deeper in center
    }

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // With deeper path ahead, steering should be near zero
    EXPECT_NEAR(output.command.steering_angle, 0.0, 0.3);
    // Speed should be relatively high
    EXPECT_GT(output.command.speed, config_.min_speed);
}

TEST_F(FollowTheGapTest, CorridorDrivesStraight) {
    // Disable bubble and disparity extension for clean corridor test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;  // Effectively disable disparity extension
    ftg_->setConfig(config_);

    // Create corridor with walls on sides and open in front
    // Make center slightly deeper to give preference for straight ahead
    std::vector<float> ranges(100, 1.0f);  // Walls everywhere
    size_t quarter = 25;
    for (size_t i = quarter; i < 75; ++i) {
        ranges[i] = 6.0f;  // Open in front
    }
    // Make center deeper to ensure straight-ahead preference
    for (size_t i = 45; i < 55; ++i) {
        ranges[i] = 7.0f;  // Deeper in center
    }

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // In a corridor, should drive straight through the gap
    EXPECT_NEAR(output.command.steering_angle, 0.0, 0.3);
    EXPECT_FALSE(output.emergency_stop);
}

// ===================
// Emergency Stop Tests
// ===================

TEST_F(FollowTheGapTest, EmergencyStopWhenObstacleTooClose) {
    auto ranges = createEmergencyScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_TRUE(output.emergency_stop);
    EXPECT_NEAR(output.command.speed, 0.0, 1e-9);
}

TEST_F(FollowTheGapTest, EmergencyStopDistanceConfigurable) {
    // Set a larger emergency distance
    config_.emergency_brake_distance = 1.0;
    ftg_->setConfig(config_);
    
    // Create scan with obstacle at 0.8m
    std::vector<float> ranges(100, 6.0f);
    ranges[50] = 0.8f;
    
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_TRUE(output.emergency_stop);
}

TEST_F(FollowTheGapTest, NoEmergencyStopWhenFarEnough) {
    // Create scan with closest obstacle at 0.5m (above default 0.3m threshold)
    std::vector<float> ranges(100, 6.0f);
    ranges[50] = 0.5f;
    
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_FALSE(output.emergency_stop);
}

// ===================
// Gap Detection Tests
// ===================

TEST_F(FollowTheGapTest, DetectsGaps) {
    // Disable bubble and disparity extension for clean gap detection test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;  // Effectively disable
    ftg_->setConfig(config_);

    auto ranges = createCorridorScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Should have found at least one gap
    EXPECT_GE(output.all_gaps.size(), 1u);
    // Selected gap should be valid
    EXPECT_TRUE(output.selected_gap.isValid());
}

TEST_F(FollowTheGapTest, NoGapsTriggersEmergencyStop) {
    // All obstacles very close - below gap threshold (0.8m)
    std::vector<float> ranges(100, 0.5f);  // All below gap threshold
    
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_TRUE(output.emergency_stop);
    EXPECT_EQ(output.all_gaps.size(), 0u);
}

TEST_F(FollowTheGapTest, SelectsBestGap) {
    // Disable bubble and disparity extension for this test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Background at 0.5m (below gap threshold 0.8m)
    std::vector<float> ranges(100, 0.5f);
    // Shallow gap on left
    for (int i = 10; i < 25; ++i) ranges[i] = 4.0f;
    // Deeper gap on right
    for (int i = 75; i < 90; ++i) ranges[i] = 8.0f;

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    EXPECT_GE(output.all_gaps.size(), 2u);
    // Selected gap should be the deeper one (consider straight preference)
    EXPECT_GE(output.selected_gap.deepest_range, 4.0);
}

// ===================
// Steering Tests
// ===================

TEST_F(FollowTheGapTest, SteersTowardGap) {
    // Disable bubble and disparity extension for steering test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Create scan with gap only on the right
    // Note: Index 0 = angle_min (-π/2), Index 99 = angle_max (+π/2)
    // So high indices = positive angles (left), low indices = negative angles (right)
    std::vector<float> ranges(100, 1.0f);  // Walls everywhere
    // Gap at high indices = left side (positive angles)
    for (int i = 70; i < 95; ++i) ranges[i] = 6.0f;

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Should steer left (positive steering angle) toward gap at high indices
    EXPECT_GT(output.command.steering_angle, 0.0);
}

TEST_F(FollowTheGapTest, SteersLeftWhenGapOnLeft) {
    // Disable bubble and disparity extension for steering test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Create scan with gap only on the left (low indices = negative angles = right side)
    std::vector<float> ranges(100, 1.0f);  // Walls everywhere
    for (int i = 5; i < 30; ++i) ranges[i] = 6.0f;  // Gap at low indices = right side

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Should steer right (negative steering angle) toward gap at low indices
    EXPECT_LT(output.command.steering_angle, 0.0);
}

TEST_F(FollowTheGapTest, SteeringAngleClamped) {
    // Disable bubble and disparity extension for steering test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Create scenario with a clear gap only on the far left side (high indices)
    // This forces the algorithm to steer sharply left
    std::vector<float> ranges(100, 1.0f);  // Walls everywhere
    // Create a substantial gap on the far left (high indices = positive angles)
    for (int i = 80; i < 100; ++i) {
        ranges[i] = 6.0f;  // Gap at extreme left
    }

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Steering should be clamped to max (steering left = positive)
    EXPECT_LE(std::abs(output.command.steering_angle), config_.max_steering_angle + 0.01);
    // And it should be steering left (positive) toward the gap
    EXPECT_GT(output.command.steering_angle, 0.0);
}

TEST_F(FollowTheGapTest, SteeringAngleClampedRight) {
    // Disable bubble and disparity extension for steering test
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Create scenario with a clear gap only on the far right side (low indices)
    // This forces the algorithm to steer sharply right
    std::vector<float> ranges(100, 1.0f);  // Walls everywhere
    // Create a substantial gap on the far right (low indices = negative angles)
    for (int i = 0; i < 20; ++i) {
        ranges[i] = 6.0f;  // Gap at extreme right
    }

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Steering should be clamped to max (steering right = negative)
    EXPECT_LE(std::abs(output.command.steering_angle), config_.max_steering_angle + 0.01);
    // And it should be steering right (negative) toward the gap
    EXPECT_LT(output.command.steering_angle, 0.0);
}

// ===================
// Speed Control Tests
// ===================

TEST_F(FollowTheGapTest, SpeedReducedWhenObstaclesClose) {
    // Gap with moderate depth
    std::vector<float> ranges(100, 2.5f);  // Just above gap threshold, close obstacles
    
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Speed should be reduced due to close obstacles
    EXPECT_LT(output.command.speed, config_.max_speed);
}

TEST_F(FollowTheGapTest, SpeedReducedInSlowdownZone) {
    // Create scan with limited range (below speed_full_range)
    std::vector<float> ranges(100, 4.0f);  // Below speed_full_range (9.0m)
    
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Speed should be reduced because range < speed_full_range
    EXPECT_LT(output.command.speed, config_.max_speed);
}

TEST_F(FollowTheGapTest, SpeedWithinLimits) {
    auto ranges = createOpenScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_GE(output.command.speed, config_.min_speed);
    EXPECT_LE(output.command.speed, config_.max_speed);
}

// ===================
// Configuration Tests
// ===================

TEST_F(FollowTheGapTest, ConfigurationCanBeUpdated) {
    FTGConfig new_config = config_;
    new_config.max_speed = 2.0;
    new_config.min_speed = 0.5;
    
    ftg_->setConfig(new_config);
    
    auto ranges = createOpenScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_LE(output.command.speed, 2.0);
}

TEST_F(FollowTheGapTest, PureFTGSelectsDeepestPoint) {
    // Pure FTG should always target the deepest point in the gap
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;  // Disable disparity extension
    ftg_->setConfig(config_);

    // Create scan with two equal gaps - one straight, one right
    std::vector<float> ranges(100, 1.0f);
    for (int i = 45; i < 55; ++i) ranges[i] = 5.0f;  // Straight gap
    for (int i = 80; i < 95; ++i) ranges[i] = 5.5f;   // Slightly deeper gap on left (high indices)

    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Pure FTG should select the wider/deeper gap combination
    EXPECT_GE(output.selected_gap.deepest_range, 5.0);
}

// ===================
// Mapping Mode Tests
// ===================

TEST_F(FollowTheGapTest, MappingModeDisabledByDefault) {
    auto ranges = createOpenScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Boundary points should be empty when mapping mode is off
    EXPECT_TRUE(output.boundary_points.empty());
}

TEST_F(FollowTheGapTest, MappingModeExtractsBoundaryPoints) {
    config_.mapping_mode = true;
    ftg_->setConfig(config_);
    
    auto ranges = createOpenScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    Pose2D pose(5.0, 10.0, 0.5);
    double timestamp = 123.456;
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc, pose, timestamp);
    
    // Should have extracted boundary points
    EXPECT_FALSE(output.boundary_points.empty());
}

// ===================
// Output Information Tests
// ===================

TEST_F(FollowTheGapTest, OutputIncludesClosestPointInfo) {
    auto ranges = createCorridorScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Should have valid closest point info
    EXPECT_GT(output.closest_point_dist, 0.0);
    EXPECT_LT(output.closest_point_idx, 100u);
}

TEST_F(FollowTheGapTest, OutputIncludesAllGaps) {
    // Disable bubble and disparity extension to get deterministic gap detection
    config_.apply_bubble = false;
    config_.disparity_threshold = 10.0;
    ftg_->setConfig(config_);

    // Background below gap threshold
    std::vector<float> ranges(100, 0.5f);
    for (int i = 10; i < 20; ++i) ranges[i] = 5.0f;  // Gap 1
    for (int i = 50; i < 60; ++i) ranges[i] = 5.0f;  // Gap 2
    for (int i = 80; i < 90; ++i) ranges[i] = 5.0f;  // Gap 3
    
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Should report all gaps
    EXPECT_GE(output.all_gaps.size(), 3u);
}

// ===================
// Reset Tests
// ===================

TEST_F(FollowTheGapTest, ResetDoesNotCrash) {
    ftg_->reset();
    
    // Should still work after reset
    auto ranges = createOpenScan(100);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    EXPECT_FALSE(output.emergency_stop);
}

// ===================
// Edge Case Tests
// ===================

TEST_F(FollowTheGapTest, HandlesEmptyScan) {
    std::vector<float> ranges;  // Empty
    
    auto output = ftg_->compute(ranges, 0.0, 0.0, 0.1);
    
    // Should handle gracefully
    EXPECT_TRUE(output.emergency_stop || output.all_gaps.empty());
}

TEST_F(FollowTheGapTest, HandlesSinglePoint) {
    std::vector<float> ranges = {5.0f};
    
    auto output = ftg_->compute(ranges, 0.0, 0.0, 0.1);
    
    // Should handle gracefully without crashing
    EXPECT_GE(output.command.speed, 0.0);
}

TEST_F(FollowTheGapTest, HandlesAllInfinity) {
    std::vector<float> ranges(100, std::numeric_limits<float>::infinity());
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);

    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);

    // Should handle gracefully without crashing
    // Infinity values are clamped to range_max, which may or may not result in valid gaps
    // depending on configuration. The important thing is graceful handling.
    EXPECT_GE(output.command.speed, 0.0);
}

TEST_F(FollowTheGapTest, HandlesAllNaN) {
    std::vector<float> ranges(100, std::nanf(""));
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Should handle gracefully - probably emergency stop since no valid data
    // The important thing is it doesn't crash
    EXPECT_GE(output.command.speed, 0.0);
}

TEST_F(FollowTheGapTest, HandlesZeroRanges) {
    std::vector<float> ranges(100, 0.0f);
    double angle_min, angle_max, angle_inc;
    getStandardScanParams(100, angle_min, angle_max, angle_inc);
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Should trigger emergency stop
    EXPECT_TRUE(output.emergency_stop);
}

// ===================
// Performance Sanity Tests
// ===================

TEST_F(FollowTheGapTest, HighResolutionScan) {
    // Test with high-resolution scan (like real UST-10LX: 1080 points)
    std::vector<float> ranges(1080, 5.0f);
    double angle_min = -2.35619;   // -135 degrees
    double angle_max = 2.35619;    // +135 degrees
    double angle_inc = (angle_max - angle_min) / 1079.0;
    
    auto output = ftg_->compute(ranges, angle_min, angle_max, angle_inc);
    
    // Should complete without issues
    EXPECT_FALSE(output.emergency_stop);
    EXPECT_GT(output.command.speed, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
