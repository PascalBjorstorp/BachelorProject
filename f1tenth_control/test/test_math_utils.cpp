#include <gtest/gtest.h>
#include "f1tenth_control/common/math_utils.hpp"
#include "f1tenth_control/common/types.hpp"
#include <cmath>
#include <vector>

using namespace f1tenth_control;
using namespace f1tenth_control::math;

// ===================
// Angle Normalization Tests
// ===================

TEST(MathUtilsTest, NormalizeAnglePositive) {
    // Angle within range should remain unchanged
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(1.0), 1.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(-1.0), -1.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(constants::PI), constants::PI, 1e-9);
}

TEST(MathUtilsTest, NormalizeAngleLargePositive) {
    // Angles > PI should wrap to negative
    EXPECT_NEAR(normalizeAngle(constants::PI + 0.5), -constants::PI + 0.5, 1e-9);
    EXPECT_NEAR(normalizeAngle(constants::TWO_PI), 0.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(3.0 * constants::PI), constants::PI, 1e-9);
}

TEST(MathUtilsTest, NormalizeAngleLargeNegative) {
    // Angles < -PI should wrap to positive
    EXPECT_NEAR(normalizeAngle(-constants::PI - 0.5), constants::PI - 0.5, 1e-9);
    EXPECT_NEAR(normalizeAngle(-constants::TWO_PI), 0.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(-3.0 * constants::PI), constants::PI, 1e-9);
}

TEST(MathUtilsTest, NormalizeAnglePositiveRange) {
    EXPECT_NEAR(normalizeAnglePositive(0.0), 0.0, 1e-9);
    EXPECT_NEAR(normalizeAnglePositive(-constants::PI), constants::PI, 1e-9);
    EXPECT_NEAR(normalizeAnglePositive(constants::TWO_PI + 1.0), 1.0, 1e-9);
}

// ===================
// Clamp Tests
// ===================

TEST(MathUtilsTest, ClampWithinRange) {
    EXPECT_EQ(clamp(5, 0, 10), 5);
    EXPECT_NEAR(clamp(0.5, 0.0, 1.0), 0.5, 1e-9);
}

TEST(MathUtilsTest, ClampBelowMin) {
    EXPECT_EQ(clamp(-5, 0, 10), 0);
    EXPECT_NEAR(clamp(-0.5, 0.0, 1.0), 0.0, 1e-9);
}

TEST(MathUtilsTest, ClampAboveMax) {
    EXPECT_EQ(clamp(15, 0, 10), 10);
    EXPECT_NEAR(clamp(1.5, 0.0, 1.0), 1.0, 1e-9);
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
// Map Range Tests
// ===================

TEST(MathUtilsTest, MapRangeBasic) {
    // Map 0-10 to 0-100
    EXPECT_NEAR(mapRange(5.0, 0.0, 10.0, 0.0, 100.0), 50.0, 1e-9);
    EXPECT_NEAR(mapRange(0.0, 0.0, 10.0, 0.0, 100.0), 0.0, 1e-9);
    EXPECT_NEAR(mapRange(10.0, 0.0, 10.0, 0.0, 100.0), 100.0, 1e-9);
}

TEST(MathUtilsTest, MapRangeInverse) {
    // Map 0-10 to 100-0 (inverse)
    EXPECT_NEAR(mapRange(5.0, 0.0, 10.0, 100.0, 0.0), 50.0, 1e-9);
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

TEST(MathUtilsTest, DistanceFromPose) {
    Pose2D pose(0, 0, 0);
    Point2D point(3, 4);
    EXPECT_NEAR(distance(pose, point), 5.0, 1e-9);
}

// ===================
// Angle To Tests
// ===================

TEST(MathUtilsTest, AngleToBasic) {
    Point2D origin(0, 0);
    Point2D right(1, 0);
    Point2D up(0, 1);
    Point2D left(-1, 0);
    Point2D down(0, -1);
    
    EXPECT_NEAR(angleTo(origin, right), 0.0, 1e-9);
    EXPECT_NEAR(angleTo(origin, up), constants::PI / 2.0, 1e-9);
    EXPECT_NEAR(angleTo(origin, left), constants::PI, 1e-9);
    EXPECT_NEAR(angleTo(origin, down), -constants::PI / 2.0, 1e-9);
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

TEST(MathUtilsTest, GlobalToLocalNoRotation) {
    Pose2D robot_pose(10, 20, 0);
    Point2D global_point(15, 20);
    
    Point2D local = globalToLocal(global_point, robot_pose);
    EXPECT_NEAR(local.x, 5.0, 1e-9);
    EXPECT_NEAR(local.y, 0.0, 1e-9);
}

TEST(MathUtilsTest, GlobalToLocal90Degrees) {
    Pose2D robot_pose(0, 0, constants::PI / 2);  // Facing +Y
    Point2D global_point(0, 5);  // Point is "ahead" in global +Y
    
    Point2D local = globalToLocal(global_point, robot_pose);
    EXPECT_NEAR(local.x, 5.0, 1e-9);  // Should be ahead (positive x in local)
    EXPECT_NEAR(local.y, 0.0, 1e-9);
}

TEST(MathUtilsTest, TransformRoundTrip) {
    Pose2D robot_pose(7.5, -3.2, 1.23);
    Point2D original(2.5, 1.5);
    
    Point2D global = localToGlobal(original, robot_pose);
    Point2D recovered = globalToLocal(global, robot_pose);
    
    EXPECT_NEAR(recovered.x, original.x, 1e-9);
    EXPECT_NEAR(recovered.y, original.y, 1e-9);
}

// ===================
// Sign Tests
// ===================

TEST(MathUtilsTest, SignFunction) {
    EXPECT_EQ(sign(5), 1);
    EXPECT_EQ(sign(-5), -1);
    EXPECT_EQ(sign(0), 0);
    EXPECT_EQ(sign(0.001), 1);
    EXPECT_EQ(sign(-0.001), -1);
}

// ===================
// Filter Tests
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
    
    for (size_t i = 0; i < data.size(); ++i) {
        EXPECT_NEAR(filtered[i], data[i], 1e-9);
    }
}

TEST(MathUtilsTest, MovingAverageBasic) {
    std::vector<double> data = {2, 2, 2, 2, 2};
    movingAverageFilter(data, 3);
    
    for (const auto& v : data) {
        EXPECT_NEAR(v, 2.0, 1e-9);
    }
}

TEST(MathUtilsTest, MovingAverageSmoothing) {
    std::vector<double> data = {0, 0, 10, 0, 0};
    movingAverageFilter(data, 3);
    
    // Spike should be smoothed
    EXPECT_LT(data[2], 10.0);
    EXPECT_GT(data[2], 0.0);
}

// ===================
// Argmin/Argmax Tests
// ===================

TEST(MathUtilsTest, ArgminBasic) {
    std::vector<double> data = {5, 3, 1, 4, 2};
    EXPECT_EQ(argmin(data), 2u);
}

TEST(MathUtilsTest, ArgmaxBasic) {
    std::vector<double> data = {5, 3, 1, 4, 2};
    EXPECT_EQ(argmax(data), 0u);
}

TEST(MathUtilsTest, ArgminWithRange) {
    std::vector<double> data = {5, 3, 1, 4, 2};
    EXPECT_EQ(argmin(data, 1, 4), 2u);  // Search indices 1-3
}

// ===================
// Curvature Tests
// ===================

TEST(MathUtilsTest, CurvatureStraightLine) {
    Point2D p1(0, 0);
    Point2D p2(1, 0);
    Point2D p3(2, 0);
    
    EXPECT_NEAR(curvature(p1, p2, p3), 0.0, 1e-9);
}

TEST(MathUtilsTest, CurvatureCircle) {
    // Points on a circle of radius 1
    double r = 1.0;
    Point2D p1(r, 0);
    Point2D p2(0, r);
    Point2D p3(-r, 0);
    
    // Curvature should be 1/r = 1
    EXPECT_NEAR(curvature(p1, p2, p3), 1.0, 0.01);
}

// ===================
// Pure Pursuit Steering Tests
// ===================

TEST(MathUtilsTest, PurePursuitStraight) {
    // Lookahead directly ahead (no lateral error)
    double steering = purePursuitSteering(2.0, 0.0, 0.324);
    EXPECT_NEAR(steering, 0.0, 1e-9);
}

TEST(MathUtilsTest, PurePursuitLeft) {
    // Lookahead point to the left (positive y in robot frame)
    double steering = purePursuitSteering(2.0, 0.5, 0.324);
    EXPECT_GT(steering, 0.0);  // Should steer left (positive)
}

TEST(MathUtilsTest, PurePursuitRight) {
    // Lookahead point to the right (negative y in robot frame)
    double steering = purePursuitSteering(2.0, -0.5, 0.324);
    EXPECT_LT(steering, 0.0);  // Should steer right (negative)
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
