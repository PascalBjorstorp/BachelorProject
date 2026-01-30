#include <gtest/gtest.h>
#include "f1tenth_control/common/lidar_processor.hpp"
#include "f1tenth_control/common/types.hpp"
#include <cmath>
#include <vector>

using namespace f1tenth_control;

class LidarProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config for tests (generic preprocessing only)
        config_.range_min = 0.1;
        config_.range_max = 10.0;
        config_.angle_min = -constants::PI / 2;  // -90 degrees
        config_.angle_max = constants::PI / 2;   // +90 degrees
        config_.apply_median_filter = false;     // Disable for deterministic tests

        processor_ = std::make_unique<LidarProcessor>(config_);
    }

    // Helper to create a simple scan
    std::vector<float> createUniformScan(float range, size_t num_points) {
        return std::vector<float>(num_points, range);
    }

    // Helper to create scan angles
    void getScanParams(size_t num_points, double& angle_min, double& angle_max, double& angle_inc) {
        angle_min = -constants::PI;
        angle_max = constants::PI;
        angle_inc = constants::TWO_PI / static_cast<double>(num_points);
    }

    LidarProcessorConfig config_;
    std::unique_ptr<LidarProcessor> processor_;
};

// ===================
// Process Scan Tests
// ===================

TEST_F(LidarProcessorTest, ProcessScanBasic) {
    std::vector<float> ranges = createUniformScan(5.0f, 100);
    double angle_min, angle_max, angle_inc;
    getScanParams(100, angle_min, angle_max, angle_inc);

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);

    EXPECT_EQ(scan.ranges.size(), 100u);
    EXPECT_EQ(scan.filtered_ranges.size(), 100u);
    EXPECT_EQ(scan.angles.size(), 100u);
    EXPECT_EQ(scan.valid.size(), 100u);
}

TEST_F(LidarProcessorTest, ProcessScanInvalidRanges) {
    std::vector<float> ranges = {1.0f, 0.0f, 5.0f, 100.0f, 5.0f};  // 0 and 100 are invalid
    double angle_inc = constants::PI / 4;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);

    // Invalid ranges should be clipped and marked invalid
    EXPECT_FALSE(scan.valid[1]);  // 0.0 is below range_min
    EXPECT_FALSE(scan.valid[3]);  // 100.0 is above range_max (but range is clipped)
}

TEST_F(LidarProcessorTest, ProcessScanNaNHandling) {
    std::vector<float> ranges = {5.0f, std::nanf(""), 5.0f};
    double angle_inc = constants::PI / 2;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);

    EXPECT_FALSE(scan.valid[1]);  // NaN should be marked invalid
}

// ===================
// Find Closest Point Tests
// ===================

TEST_F(LidarProcessorTest, FindClosestPointBasic) {
    std::vector<float> ranges = {5.0f, 3.0f, 1.0f, 4.0f, 6.0f};
    double angle_inc = constants::PI / 4;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    size_t closest = processor_->findClosestPoint(scan);

    EXPECT_NEAR(scan.filtered_ranges[closest], 1.0, 1e-9);
}

// ===================
// Cartesian Conversion Tests
// ===================

TEST_F(LidarProcessorTest, ScanPointToCartesian) {
    std::vector<float> ranges = {5.0f};
    double angle_min = 0.0;
    double angle_max = 0.0;
    double angle_inc = 1.0;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto point = processor_->scanPointToCartesian(scan, 0);

    // At angle 0, point should be directly ahead
    EXPECT_NEAR(point.x, 5.0, 1e-9);
    EXPECT_NEAR(point.y, 0.0, 1e-9);
}

TEST_F(LidarProcessorTest, ScanPointToCartesian90Degrees) {
    std::vector<float> ranges = {5.0f};
    double angle_min = constants::PI / 2;
    double angle_max = constants::PI / 2;
    double angle_inc = 1.0;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto point = processor_->scanPointToCartesian(scan, 0);

    // At 90 degrees, point should be to the left
    EXPECT_NEAR(point.x, 0.0, 1e-6);
    EXPECT_NEAR(point.y, 5.0, 1e-6);
}

// ===================
// Boundary Extraction Tests
// ===================

TEST_F(LidarProcessorTest, ExtractBoundaryPoints) {
    std::vector<float> ranges = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    double angle_inc = constants::PI / 4;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    Pose2D robot_pose(10.0, 20.0, 0.0);
    double timestamp = 123.456;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto boundary = processor_->extractBoundaryPoints(scan, robot_pose, timestamp);

    EXPECT_EQ(boundary.size(), 5u);

    // Check that points are transformed to global frame
    // Point at angle 0 (straight ahead) should be at robot_x + range
    bool found_ahead = false;
    for (const auto& bp : boundary) {
        if (std::abs(bp.position.y - 20.0) < 0.1) {
            EXPECT_NEAR(bp.position.x, 15.0, 0.5);  // 10 + 5
            found_ahead = true;
        }
        EXPECT_NEAR(bp.timestamp, timestamp, 1e-9);
    }
    EXPECT_TRUE(found_ahead);
}

// ===================
// Configuration Tests
// ===================

TEST_F(LidarProcessorTest, ConfigUpdate) {
    LidarProcessorConfig new_config;
    new_config.range_min = 0.5;
    new_config.range_max = 8.0;

    processor_->setConfig(new_config);

    auto retrieved_config = processor_->getConfig();
    EXPECT_NEAR(retrieved_config.range_min, 0.5, 1e-9);
    EXPECT_NEAR(retrieved_config.range_max, 8.0, 1e-9);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
