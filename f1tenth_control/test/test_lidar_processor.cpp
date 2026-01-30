#include <gtest/gtest.h>
#include "f1tenth_control/common/lidar_processor.hpp"
#include "f1tenth_control/common/types.hpp"
#include <cmath>
#include <vector>

using namespace f1tenth_control;

class LidarProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config for tests
        config_.range_min = 0.1;
        config_.range_max = 10.0;
        config_.angle_min = -constants::PI / 2;  // -90 degrees
        config_.angle_max = constants::PI / 2;   // +90 degrees
        config_.apply_median_filter = false;     // Disable for deterministic tests
        config_.disparity_threshold = 0.5;
        config_.gap_threshold = 2.0;
        config_.min_gap_width = 0.2;
        config_.bubble_radius = 0.3;
        config_.apply_bubble = true;

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
// Safety Bubble Tests
// ===================

TEST_F(LidarProcessorTest, SafetyBubbleApplication) {
    // Create scan with one close point
    std::vector<float> ranges(21, 5.0f);
    ranges[10] = 1.0f;  // Close point in the middle

    double angle_inc = constants::PI / 20;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    processor_->applySafetyBubble(scan);

    // Points near the closest should be zeroed
    EXPECT_NEAR(scan.filtered_ranges[10], 0.0, 1e-9);
    // Points far from closest should remain
    EXPECT_NEAR(scan.filtered_ranges[0], 5.0, 1e-9);
    EXPECT_NEAR(scan.filtered_ranges[20], 5.0, 1e-9);
}

// ===================
// Disparity Extension Tests
// ===================

TEST_F(LidarProcessorTest, DisparityExtensionBasic) {
    // Create scan with a disparity (wall corner)
    std::vector<float> ranges(10, 5.0f);
    ranges[5] = 2.0f;  // Suddenly closer
    ranges[6] = 2.0f;
    ranges[7] = 2.0f;

    double angle_inc = constants::PI / 9;  // 20 degrees
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    processor_->applyDisparityExtension(scan, 0.3);  // 30cm car width

    // The disparity should cause extension
    // Points before the disparity should be extended closer
    EXPECT_LE(scan.filtered_ranges[4], 5.0);
}

// ===================
// Gap Finding Tests
// ===================

TEST_F(LidarProcessorTest, FindGapsSingleGap) {
    // Create scan with walls on sides and gap in middle
    std::vector<float> ranges(21, 1.0f);  // Close obstacles
    for (int i = 8; i <= 12; ++i) {
        ranges[i] = 5.0f;  // Gap in middle
    }

    double angle_inc = constants::PI / 20;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto gaps = processor_->findGaps(scan);

    EXPECT_GE(gaps.size(), 1u);
    if (!gaps.empty()) {
        // Gap should be around the middle
        EXPECT_GE(gaps[0].start_idx, 7u);
        EXPECT_LE(gaps[0].end_idx, 13u);
    }
}

TEST_F(LidarProcessorTest, FindGapsNoGaps) {
    // All obstacles close - no gaps
    std::vector<float> ranges(10, 1.0f);

    double angle_inc = constants::PI / 9;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto gaps = processor_->findGaps(scan);

    EXPECT_EQ(gaps.size(), 0u);
}

TEST_F(LidarProcessorTest, FindGapsMultiple) {
    // Two gaps
    std::vector<float> ranges(21, 1.0f);
    for (int i = 2; i <= 5; ++i) ranges[i] = 5.0f;   // First gap
    for (int i = 15; i <= 18; ++i) ranges[i] = 5.0f; // Second gap

    double angle_inc = constants::PI / 20;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto gaps = processor_->findGaps(scan);

    EXPECT_GE(gaps.size(), 2u);
}

TEST_F(LidarProcessorTest, FindBestGapDeepest) {
    // Create gaps with different depths
    std::vector<float> ranges(21, 1.0f);
    for (int i = 2; i <= 5; ++i) ranges[i] = 3.0f;   // Shallow gap
    for (int i = 15; i <= 18; ++i) ranges[i] = 8.0f; // Deep gap

    double angle_inc = constants::PI / 20;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto gaps = processor_->findGaps(scan);

    // Default scorer should prefer deeper gaps
    auto best = processor_->findBestGap(gaps);
    EXPECT_GE(best.deepest_range, 7.0);  // Should be the deeper gap
}

TEST_F(LidarProcessorTest, FindBestGapCustomScorer) {
    std::vector<float> ranges(21, 1.0f);
    for (int i = 2; i <= 5; ++i) ranges[i] = 5.0f;   // Gap 1
    for (int i = 15; i <= 18; ++i) ranges[i] = 5.0f; // Gap 2

    double angle_inc = constants::PI / 20;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto gaps = processor_->findGaps(scan);

    // Custom scorer that prefers gaps closer to center (angle 0)
    auto scorer = [](const Gap& g) {
        return -std::abs(g.centerAngle());  // Closer to 0 = higher score
    };

    auto best = processor_->findBestGap(gaps, scorer);
    EXPECT_LT(std::abs(best.centerAngle()), constants::PI / 2);
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
    new_config.gap_threshold = 4.0;

    processor_->setConfig(new_config);

    auto retrieved_config = processor_->getConfig();
    EXPECT_NEAR(retrieved_config.range_min, 0.5, 1e-9);
    EXPECT_NEAR(retrieved_config.range_max, 8.0, 1e-9);
    EXPECT_NEAR(retrieved_config.gap_threshold, 4.0, 1e-9);
}

// ===================
// Gap Properties Tests
// ===================

TEST_F(LidarProcessorTest, GapPropertiesCorrect) {
    std::vector<float> ranges(21, 1.0f);
    // Create a gap with known properties
    ranges[8] = 3.0f;
    ranges[9] = 4.0f;
    ranges[10] = 6.0f;  // Deepest
    ranges[11] = 5.0f;
    ranges[12] = 3.0f;

    double angle_inc = constants::PI / 20;
    double angle_min = -constants::PI / 2;
    double angle_max = constants::PI / 2;

    auto scan = processor_->processScan(ranges, angle_min, angle_max, angle_inc);
    auto gaps = processor_->findGaps(scan);

    ASSERT_GE(gaps.size(), 1u);

    // Check gap properties
    const Gap& gap = gaps[0];
    EXPECT_NEAR(gap.deepest_range, 6.0, 0.1);
    EXPECT_GE(gap.min_range, 3.0);
    EXPECT_LE(gap.max_range, 6.0);
    EXPECT_GT(gap.angular_width, 0.0);
    EXPECT_TRUE(gap.isValid());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
