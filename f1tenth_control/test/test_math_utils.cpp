#include <gtest/gtest.h>
#include "common/math_utils.hpp"
#include "common/types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace f1tenth_control;
using namespace f1tenth_control::math;

// ===================
// Angle Normalization Tests
// ===================

TEST(MathUtilsTest, NormalizeAngleWithinRange) {
    // Angles within [-PI, PI] should remain unchanged
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-6);
    EXPECT_NEAR(normalizeAngle(1.0), 1.0, 1e-6);
    EXPECT_NEAR(normalizeAngle(-1.0), -1.0, 1e-6);
}

TEST(MathUtilsTest, NormalizeAngleLargePositive) {
    // Angles > PI should wrap to negative
    EXPECT_NEAR(normalizeAngle(constants::PI + 0.5), -constants::PI + 0.5, 1e-6);
    EXPECT_NEAR(normalizeAngle(constants::TWO_PI), 0.0, 1e-6);
}

TEST(MathUtilsTest, NormalizeAngleLargeNegative) {
    // Angles < -PI should wrap appropriately
    EXPECT_NEAR(normalizeAngle(-constants::PI - 0.5), constants::PI - 0.5, 1e-6);
    EXPECT_NEAR(normalizeAngle(-constants::TWO_PI), 0.0, 1e-6);
}

// ===================
// Clamp Tests
// ===================

TEST(MathUtilsTest, ClampWithinRange) {
    EXPECT_EQ(std::clamp(5, 0, 10), 5);
    EXPECT_NEAR(std::clamp(0.5, 0.0, 1.0), 0.5, 1e-9);
}

TEST(MathUtilsTest, ClampBelowMin) {
    EXPECT_EQ(std::clamp(-5, 0, 10), 0);
    EXPECT_NEAR(std::clamp(-0.5, 0.0, 1.0), 0.0, 1e-9);
}

TEST(MathUtilsTest, ClampAboveMax) {
    EXPECT_EQ(std::clamp(15, 0, 10), 10);
    EXPECT_NEAR(std::clamp(1.5, 0.0, 1.0), 1.0, 1e-9);
}

// ===================
// Linear Interpolation Tests
// ===================

TEST(MathUtilsTest, LerpBasic) {
    EXPECT_NEAR(lerp(0.0, 10.0, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(lerp(0.0, 10.0, 1.0), 10.0, 1e-9);
    EXPECT_NEAR(lerp(0.0, 10.0, 0.5), 5.0, 1e-9);
    EXPECT_NEAR(lerp(0.0, 10.0, 0.25), 2.5, 1e-9);
}

TEST(MathUtilsTest, LerpNegative) {
    EXPECT_NEAR(lerp(-10.0, 10.0, 0.5), 0.0, 1e-9);
    EXPECT_NEAR(lerp(-10.0, -5.0, 0.5), -7.5, 1e-9);
}

// ===================
// Distance Tests
// ===================

TEST(MathUtilsTest, DistanceBasic) {
    Point2D a(0, 0);
    Point2D b(3, 4);
    EXPECT_NEAR(distance(a, b), 5.0, 1e-9);
}

TEST(MathUtilsTest, DistanceSamePoint) {
    Point2D a(5, 5);
    EXPECT_NEAR(distance(a, a), 0.0, 1e-9);
}

TEST(MathUtilsTest, DistanceNegative) {
    Point2D a(-3, -4);
    Point2D b(0, 0);
    EXPECT_NEAR(distance(a, b), 5.0, 1e-9);
}

// ===================
// Coordinate Transform Tests
// ===================

TEST(MathUtilsTest, LocalToGlobalNoRotation) {
    Pose2D robot_pose(10, 20, 0);  // At (10, 20), facing +X
    Point2D local_point(5, 0);     // 5m ahead in robot frame
    
    Point2D global = localToGlobal(local_point, robot_pose);
    EXPECT_NEAR(global.x, 15.0, 1e-9);
    EXPECT_NEAR(global.y, 20.0, 1e-9);
}

TEST(MathUtilsTest, LocalToGlobal90Degrees) {
    Pose2D robot_pose(0, 0, constants::PI / 2);  // Facing +Y
    Point2D local_point(1, 0);  // 1m ahead in robot frame
    
    Point2D global = localToGlobal(local_point, robot_pose);
    EXPECT_NEAR(global.x, 0.0, 1e-9);
    EXPECT_NEAR(global.y, 1.0, 1e-9);
}

TEST(MathUtilsTest, LocalToGlobalWithOffset) {
    Pose2D robot_pose(5, 5, constants::PI / 4);  // 45 degrees
    Point2D local_point(std::sqrt(2), 0);        // sqrt(2) ahead
    
    Point2D global = localToGlobal(local_point, robot_pose);
    EXPECT_NEAR(global.x, 6.0, 1e-9);
    EXPECT_NEAR(global.y, 6.0, 1e-9);
}

// ===================
// Median Filter Tests
// ===================

TEST(MathUtilsTest, MedianFilterBasic) {
    std::vector<double> data = {1, 2, 100, 4, 5};
    auto filtered = medianFilter(data, 3);
    
    EXPECT_EQ(filtered.size(), data.size());
    EXPECT_NEAR(filtered[2], 4.0, 1e-9);  // Spike should be removed
}

TEST(MathUtilsTest, MedianFilterPreservesMonotonic) {
    std::vector<double> data = {1, 2, 3, 4, 5};
    auto filtered = medianFilter(data, 3);

    // Middle points should be preserved exactly
    for (size_t i = 1; i < data.size() - 1; ++i) {
        EXPECT_NEAR(filtered[i], data[i], 1e-9);
    }
}

TEST(MathUtilsTest, MedianFilterEmptyInput) {
    std::vector<double> empty;
    auto filtered = medianFilter(empty, 3);
    EXPECT_TRUE(filtered.empty());
}

TEST(MathUtilsTest, MedianFilterWindowSizeOne) {
    std::vector<double> data = {1, 2, 3, 4, 5};
    auto filtered = medianFilter(data, 1);
    
    // Should return original data unchanged
    for (size_t i = 0; i < data.size(); ++i) {
        EXPECT_NEAR(filtered[i], data[i], 1e-9);
    }
}

// ===================
// Point2D Tests
// ===================

TEST(TypesTest, Point2DNorm) {
    Point2D p(3, 4);
    EXPECT_NEAR(p.norm(), 5.0, 1e-9);
}

TEST(TypesTest, Point2DDot) {
    Point2D a(1, 2);
    Point2D b(3, 4);
    EXPECT_NEAR(a.dot(b), 11.0, 1e-9);  // 1*3 + 2*4 = 11
}

TEST(TypesTest, Point2DOperators) {
    Point2D a(1, 2);
    Point2D b(3, 4);
    
    Point2D sum = a + b;
    EXPECT_NEAR(sum.x, 4.0, 1e-9);
    EXPECT_NEAR(sum.y, 6.0, 1e-9);
    
    Point2D diff = a - b;
    EXPECT_NEAR(diff.x, -2.0, 1e-9);
    EXPECT_NEAR(diff.y, -2.0, 1e-9);
    
    Point2D scaled = a * 2.0;
    EXPECT_NEAR(scaled.x, 2.0, 1e-9);
    EXPECT_NEAR(scaled.y, 4.0, 1e-9);
}

// ===================
// PolarPoint Tests
// ===================

TEST(TypesTest, PolarToCartesian) {
    PolarPoint p(5.0, 0.0);  // 5m at 0 degrees
    Point2D cart = p.toCartesian();
    EXPECT_NEAR(cart.x, 5.0, 1e-9);
    EXPECT_NEAR(cart.y, 0.0, 1e-9);
}

TEST(TypesTest, PolarToCartesian45Degrees) {
    PolarPoint p(std::sqrt(2), constants::PI / 4);  // sqrt(2) at 45 degrees
    Point2D cart = p.toCartesian();
    EXPECT_NEAR(cart.x, 1.0, 1e-9);
    EXPECT_NEAR(cart.y, 1.0, 1e-9);
}

// ===================
// Gap Tests
// ===================

TEST(TypesTest, GapCenterAngle) {
    Gap gap;
    gap.start_angle = -0.5;
    gap.end_angle = 0.5;
    EXPECT_NEAR(gap.centerAngle(), 0.0, 1e-9);
}

TEST(TypesTest, GapValidity) {
    Gap valid_gap;
    valid_gap.start_idx = 0;
    valid_gap.end_idx = 10;
    valid_gap.angular_width = 0.5;
    EXPECT_TRUE(valid_gap.isValid());
    
    Gap invalid_gap;
    invalid_gap.start_idx = 10;
    invalid_gap.end_idx = 5;  // End before start
    invalid_gap.angular_width = 0.5;
    EXPECT_FALSE(invalid_gap.isValid());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
