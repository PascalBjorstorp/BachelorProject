/**
 * @file test_pure_pursuit.c
 * @brief Testbench for Pure Pursuit FPGA Implementation
 *
 * Tests:
 * 1. Fixed-point math functions (sin, cos, atan, atan2)
 * 2. Pure Pursuit steering on synthetic waypoints
 * 3. Edge cases and error conditions
 *
 * Run on x86 before HLS synthesis to validate correctness.
 * Compile: gcc -I./include -o build/test_pure_pursuit src/pure_pursuit_fpga.c src/fp_math_hls.c test/test_pure_pursuit.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "fpga_interface.h"
#include "fp_math_hls.h"

/*===========================================================================
 * Test Utilities
 *===========================================================================*/

#define TOLERANCE_PERCENT 1.0   // Allow 1% error for fixed-point
#define TOLERANCE_ABS     0.01  // Or 0.01 absolute error

static int tests_passed = 0;
static int tests_failed = 0;

static void check_fp(const char* name, fixed_point_t got, double expected)
{
    double got_f = FP_TO_DOUBLE(got);
    double error = fabs(got_f - expected);
    double pct_error = (expected != 0) ? (error / fabs(expected)) * 100.0 : error * 100.0;
    
    int pass = (pct_error < TOLERANCE_PERCENT) || (error < TOLERANCE_ABS);
    
    if (pass) {
        printf("[PASS] %s: got %.6f, expected %.6f (err: %.3f%%)\n", 
               name, got_f, expected, pct_error);
        tests_passed++;
    } else {
        printf("[FAIL] %s: got %.6f, expected %.6f (err: %.3f%%)\n", 
               name, got_f, expected, pct_error);
        tests_failed++;
    }
}

static void check_condition(const char* name, int condition)
{
    if (condition) {
        printf("[PASS] %s\n", name);
        tests_passed++;
    } else {
        printf("[FAIL] %s\n", name);
        tests_failed++;
    }
}

/*===========================================================================
 * Test 1: Fixed-Point Math Functions
 *===========================================================================*/

static void test_fp_math(void)
{
    printf("\n========== Test Fixed-Point Math ==========\n");
    
    // Basic operations
    fixed_point_t a = FP_CONST(3.5);
    fixed_point_t b = FP_CONST(2.0);
    
    check_fp("fp_add(3.5, 2.0)", fp_add(a, b), 5.5);
    check_fp("fp_sub(3.5, 2.0)", fp_sub(a, b), 1.5);
    check_fp("fp_mul(3.5, 2.0)", fp_mul(a, b), 7.0);
    check_fp("fp_div(3.5, 2.0)", fp_div(a, b), 1.75);
    
    // Trigonometric functions
    check_fp("fp_sin(0)", fp_sin(0), 0.0);
    check_fp("fp_sin(π/2)", fp_sin(FP_PI_HALF), 1.0);
    check_fp("fp_sin(π)", fp_sin(FP_PI), 0.0);
    check_fp("fp_sin(-π/2)", fp_sin(-FP_PI_HALF), -1.0);
    check_fp("fp_sin(π/6)", fp_sin(FP_CONST(M_PI/6)), 0.5);
    
    check_fp("fp_cos(0)", fp_cos(0), 1.0);
    check_fp("fp_cos(π/2)", fp_cos(FP_PI_HALF), 0.0);
    check_fp("fp_cos(π)", fp_cos(FP_PI), -1.0);
    check_fp("fp_cos(π/3)", fp_cos(FP_CONST(M_PI/3)), 0.5);
    
    // Arctangent
    check_fp("fp_atan(0)", fp_atan(0), 0.0);
    check_fp("fp_atan(1)", fp_atan(FP_ONE), M_PI / 4.0);
    check_fp("fp_atan(-1)", fp_atan(-FP_ONE), -M_PI / 4.0);
    check_fp("fp_atan(0.5)", fp_atan(FP_CONST(0.5)), atan(0.5));
    check_fp("fp_atan(2.0)", fp_atan(FP_CONST(2.0)), atan(2.0));
    
    // atan2 quadrants
    check_fp("fp_atan2(1, 1)", fp_atan2(FP_ONE, FP_ONE), M_PI / 4.0);         // Q1
    check_fp("fp_atan2(1, -1)", fp_atan2(FP_ONE, -FP_ONE), 3.0 * M_PI / 4.0); // Q2
    check_fp("fp_atan2(-1, -1)", fp_atan2(-FP_ONE, -FP_ONE), -3.0 * M_PI / 4.0); // Q3
    check_fp("fp_atan2(-1, 1)", fp_atan2(-FP_ONE, FP_ONE), -M_PI / 4.0);      // Q4
    
    // Square root
    check_fp("fp_sqrt(4.0)", fp_sqrt(FP_CONST(4.0)), 2.0);
    check_fp("fp_sqrt(9.0)", fp_sqrt(FP_CONST(9.0)), 3.0);
    check_fp("fp_sqrt(2.0)", fp_sqrt(FP_CONST(2.0)), sqrt(2.0));
    
    // Normalize angle
    check_fp("normalize(0)", fp_normalize_angle(0), 0.0);
    check_fp("normalize(2π)", fp_normalize_angle(FP_TWO_PI), 0.0);
    check_fp("normalize(-2π)", fp_normalize_angle(-FP_TWO_PI), 0.0);
    check_fp("normalize(3π)", fp_normalize_angle(FP_CONST(3.0 * M_PI)), M_PI);
}

/*===========================================================================
 * Test 2: Pure Pursuit on Straight Path
 *===========================================================================*/

// Declared in pure_pursuit_fpga.c
extern void load_trajectory(const FpgaWaypoint_t* waypoints, uint32_t num_waypoints);
extern void pure_pursuit_compute(const FpgaStateInput_t* state, 
                                  const FpgaParams_t* params,
                                  FpgaOutput_t* output);
extern void pure_pursuit_fpga(uint32_t mode,
                               const FpgaWaypoint_t* waypoints_in,
                               uint32_t num_waypoints,
                               const FpgaStateInput_t* state,
                               const FpgaParams_t* params,
                               FpgaOutput_t* output,
                               uint32_t* traj_loaded,
                               uint32_t* traj_size_out);

static void test_straight_path(void)
{
    printf("\n========== Test Straight Path ==========\n");
    
    // Create straight path along X-axis
    FpgaWaypoint_t waypoints[20];
    for (int i = 0; i < 20; i++) {
        waypoints[i].x_fp = FP_CONST(i * 1.0);  // 0, 1, 2, ... meters
        waypoints[i].y_fp = 0;
        waypoints[i].theta_fp = 0;  // Heading = 0 (along X)
        waypoints[i].velocity_fp = FP_CONST(3.0);  // 3 m/s
        waypoints[i].kappa_fp = 0;
        memset(waypoints[i].reserved, 0, sizeof(waypoints[i].reserved));
    }
    
    // Load trajectory
    uint32_t traj_loaded, traj_size;
    FpgaOutput_t output;
    
    pure_pursuit_fpga(1, waypoints, 20, NULL, NULL, NULL, &traj_loaded, &traj_size);
    check_condition("Trajectory loaded", traj_loaded == 1);
    check_condition("Trajectory size = 20", traj_size == 20);
    
    // Setup parameters
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(1.5),
        .max_lookahead_fp = FP_CONST(3.0),
        .lookahead_gain_fp = FP_CONST(0.3),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),  // 24 degrees
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 20,
        .lookahead_points = 10
    };
    
    // Test 1: Vehicle on path, pointing along path
    FpgaStateInput_t state1 = {
        .x_fp = FP_CONST(2.0),
        .y_fp = 0,
        .theta_fp = 0,
        .velocity_fp = FP_CONST(2.0),
        .waypoint_index = 2,
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state1, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("Status OK", output.status == STATUS_OK);
    check_fp("Steering ~0 on straight path", output.steering_angle_fp, 0.0);
    check_fp("Velocity = 3.0", output.velocity_fp, 3.0);
    check_fp("CTE ~0", output.cross_track_error_fp, 0.0);
    
    // Test 2: Vehicle slightly off path (should steer towards path)
    FpgaStateInput_t state2 = {
        .x_fp = FP_CONST(2.0),
        .y_fp = FP_CONST(0.5),  // 0.5m to the left
        .theta_fp = 0,
        .velocity_fp = FP_CONST(2.0),
        .waypoint_index = 2,
        .timestamp_ms = 0,
        .sequence_number = 2
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state2, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("Status OK (off-path)", output.status == STATUS_OK);
    // Should steer right (negative) to get back to path
    double steering_deg = FP_TO_DOUBLE(output.steering_angle_fp) * 180.0 / M_PI;
    check_condition("Steers right (negative steering)", output.steering_angle_fp < 0);
    printf("  Steering angle: %.2f degrees (should be negative)\n", steering_deg);
}

/*===========================================================================
 * Test 3: Curved Path
 *===========================================================================*/

static void test_curved_path(void)
{
    printf("\n========== Test Curved Path ==========\n");
    
    // Create a quarter circle path (90 degree turn)
    FpgaWaypoint_t waypoints[25];
    double radius = 5.0;  // 5 meter radius
    
    for (int i = 0; i < 25; i++) {
        double angle = (double)i / 24.0 * (M_PI / 2.0);  // 0 to π/2
        waypoints[i].x_fp = FP_CONST(radius * sin(angle));
        waypoints[i].y_fp = FP_CONST(radius * (1.0 - cos(angle)));
        waypoints[i].theta_fp = FP_CONST(angle);  // Tangent direction
        waypoints[i].velocity_fp = FP_CONST(2.0);
        waypoints[i].kappa_fp = FP_CONST(1.0 / radius);
        memset(waypoints[i].reserved, 0, sizeof(waypoints[i].reserved));
    }
    
    uint32_t traj_loaded, traj_size;
    FpgaOutput_t output;
    
    pure_pursuit_fpga(1, waypoints, 25, NULL, NULL, NULL, &traj_loaded, &traj_size);
    
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(1.0),
        .max_lookahead_fp = FP_CONST(2.5),
        .lookahead_gain_fp = FP_CONST(0.3),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 25,
        .lookahead_points = 10
    };
    
    // Test at start of curve
    FpgaStateInput_t state = {
        .x_fp = waypoints[0].x_fp,
        .y_fp = waypoints[0].y_fp,
        .theta_fp = waypoints[0].theta_fp,
        .velocity_fp = FP_CONST(2.0),
        .waypoint_index = 0,
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("Status OK (curve)", output.status == STATUS_OK);
    // Should steer left (positive) for left turn
    check_condition("Steers left (positive)", output.steering_angle_fp > 0);
    
    double steering_deg = FP_TO_DOUBLE(output.steering_angle_fp) * 180.0 / M_PI;
    printf("  Steering angle: %.2f degrees (for R=5m curve)\n", steering_deg);
    printf("  Target waypoint: %u\n", output.target_waypoint_idx);
}

/*===========================================================================
 * Test 4: Error Conditions
 *===========================================================================*/

static void test_error_conditions(void)
{
    printf("\n========== Test Error Conditions ==========\n");
    
    uint32_t traj_loaded = 0, traj_size = 0;
    FpgaOutput_t output;
    
    // Clear trajectory first (simulating fresh start)
    // Note: We can't actually clear static vars, but we can test with invalid index
    
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(1.5),
        .max_lookahead_fp = FP_CONST(3.0),
        .lookahead_gain_fp = FP_CONST(0.3),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 0,  // No trajectory
        .lookahead_points = 10
    };
    
    FpgaStateInput_t state = {
        .x_fp = 0, .y_fp = 0, .theta_fp = 0, .velocity_fp = 0,
        .waypoint_index = 99999,  // Invalid index
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    // Load minimal trajectory
    FpgaWaypoint_t wp = {0};
    wp.velocity_fp = FP_CONST(1.0);
    pure_pursuit_fpga(1, &wp, 1, NULL, NULL, NULL, &traj_loaded, &traj_size);
    check_condition("Single waypoint loaded", traj_size == 1);
    
    // Test with invalid waypoint index (should wrap around)
    params.trajectory_size = 1;
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    check_condition("Status OK with wrapped index", output.status == STATUS_OK);
}

/*===========================================================================
 * Test 5: Trajectory Wrap-Around
 *===========================================================================*/

static void test_trajectory_wraparound(void)
{
    printf("\n========== Test Trajectory Wrap-Around ==========\n");
    
    // Create 100-waypoint circular path (closed loop)
    FpgaWaypoint_t waypoints[100];
    double radius = 10.0;
    
    for (int i = 0; i < 100; i++) {
        double angle = (double)i / 100.0 * 2.0 * M_PI;
        waypoints[i].x_fp = FP_CONST(radius * cos(angle));
        waypoints[i].y_fp = FP_CONST(radius * sin(angle));
        waypoints[i].theta_fp = FP_CONST(angle + M_PI / 2.0);  // Tangent
        waypoints[i].velocity_fp = FP_CONST(3.0);
        waypoints[i].kappa_fp = FP_CONST(1.0 / radius);
        memset(waypoints[i].reserved, 0, sizeof(waypoints[i].reserved));
    }
    
    uint32_t traj_loaded, traj_size;
    FpgaOutput_t output;
    
    pure_pursuit_fpga(1, waypoints, 100, NULL, NULL, NULL, &traj_loaded, &traj_size);
    check_condition("100 waypoints loaded", traj_size == 100);
    
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(2.0),
        .max_lookahead_fp = FP_CONST(4.0),
        .lookahead_gain_fp = FP_CONST(0.3),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 100,
        .lookahead_points = 20
    };
    
    // Test at waypoint 98 (near end, should search forward to 99, 0, 1, 2...)
    FpgaStateInput_t state = {
        .x_fp = waypoints[98].x_fp,
        .y_fp = waypoints[98].y_fp,
        .theta_fp = waypoints[98].theta_fp,
        .velocity_fp = FP_CONST(3.0),
        .waypoint_index = 98,
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("Wraparound: Status OK", output.status == STATUS_OK);
    // Target should be a few waypoints ahead, wrapping around 0
    printf("  Current waypoint: 98, Target waypoint: %u\n", output.target_waypoint_idx);
    // Target could be 99, 0, 1, 2, etc depending on lookahead
    int wrapped = (output.target_waypoint_idx < 5) || (output.target_waypoint_idx >= 98);
    check_condition("Target wrapped or near end", wrapped);
}

/*===========================================================================
 * Test 6: Zero Velocity Edge Case
 *===========================================================================*/

static void test_zero_velocity(void)
{
    printf("\n========== Test Zero Velocity ==========\n");
    
    // Use previous trajectory (still loaded)
    uint32_t traj_loaded, traj_size;
    FpgaOutput_t output;
    
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(1.5),
        .max_lookahead_fp = FP_CONST(3.0),
        .lookahead_gain_fp = FP_CONST(0.3),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 100,
        .lookahead_points = 10
    };
    
    // Zero velocity - lookahead should be min_lookahead
    FpgaStateInput_t state = {
        .x_fp = FP_CONST(10.0),  // On trajectory
        .y_fp = FP_CONST(0.0),
        .theta_fp = FP_CONST(M_PI / 2.0),
        .velocity_fp = 0,  // ZERO velocity
        .waypoint_index = 0,
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("Zero velocity: Status OK", output.status == STATUS_OK);
    // Lookahead should equal min_lookahead (since v=0)
    check_fp("Lookahead = min (1.5)", output.lookahead_dist_fp, 1.5);
    
    // Negative velocity (reversing)
    state.velocity_fp = FP_CONST(-2.0);
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("Negative velocity: Status OK", output.status == STATUS_OK);
    // Should use absolute value of velocity for lookahead
    double lookahead_expected = 1.5 + 0.3 * 2.0;  // min + gain * |v|
    check_fp("Lookahead with negative v", output.lookahead_dist_fp, lookahead_expected);
}

/*===========================================================================
 * Test 7: Division Safety (L_sq near zero)
 *===========================================================================*/

static void test_division_safety(void)
{
    printf("\n========== Test Division Safety ==========\n");
    
    // Create simple path
    FpgaWaypoint_t waypoints[5];
    for (int i = 0; i < 5; i++) {
        waypoints[i].x_fp = FP_CONST(i * 1.0);
        waypoints[i].y_fp = 0;
        waypoints[i].theta_fp = 0;
        waypoints[i].velocity_fp = FP_CONST(2.0);
        waypoints[i].kappa_fp = 0;
        memset(waypoints[i].reserved, 0, sizeof(waypoints[i].reserved));
    }
    
    uint32_t traj_loaded, traj_size;
    FpgaOutput_t output;
    
    pure_pursuit_fpga(1, waypoints, 5, NULL, NULL, NULL, &traj_loaded, &traj_size);
    
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(0.1),  // Very small lookahead
        .max_lookahead_fp = FP_CONST(0.5),
        .lookahead_gain_fp = FP_CONST(0.1),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 5,
        .lookahead_points = 5
    };
    
    // Car exactly on top of waypoint 2
    FpgaStateInput_t state = {
        .x_fp = FP_CONST(2.0),
        .y_fp = 0,
        .theta_fp = 0,
        .velocity_fp = FP_CONST(0.1),  // Very slow
        .waypoint_index = 2,
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    
    check_condition("On waypoint: Status OK", output.status == STATUS_OK);
    // Should not crash or produce NaN-like values
    double steering_deg = FP_TO_DOUBLE(output.steering_angle_fp) * 180.0 / M_PI;
    check_condition("Steering is finite", steering_deg >= -30.0 && steering_deg <= 30.0);
    printf("  Steering when on waypoint: %.2f degrees\n", steering_deg);
}

/*===========================================================================
 * Test 8: Boundary Waypoints (First and Last)
 *===========================================================================*/

static void test_boundary_waypoints(void)
{
    printf("\n========== Test Boundary Waypoints ==========\n");
    
    // Create 10-waypoint path
    FpgaWaypoint_t waypoints[10];
    for (int i = 0; i < 10; i++) {
        waypoints[i].x_fp = FP_CONST(i * 2.0);
        waypoints[i].y_fp = 0;
        waypoints[i].theta_fp = 0;
        waypoints[i].velocity_fp = FP_CONST(3.0);
        waypoints[i].kappa_fp = 0;
        memset(waypoints[i].reserved, 0, sizeof(waypoints[i].reserved));
    }
    
    uint32_t traj_loaded, traj_size;
    FpgaOutput_t output;
    
    pure_pursuit_fpga(1, waypoints, 10, NULL, NULL, NULL, &traj_loaded, &traj_size);
    
    FpgaParams_t params = {
        .min_lookahead_fp = FP_CONST(2.0),
        .max_lookahead_fp = FP_CONST(5.0),
        .lookahead_gain_fp = FP_CONST(0.3),
        .wheelbase_fp = FP_CONST(0.324),
        .max_steering_fp = FP_CONST(0.4189),
        .max_velocity_fp = FP_CONST(5.0),
        .trajectory_size = 10,
        .lookahead_points = 5
    };
    
    // Test at first waypoint (index 0)
    FpgaStateInput_t state = {
        .x_fp = FP_CONST(0.0),
        .y_fp = 0,
        .theta_fp = 0,
        .velocity_fp = FP_CONST(2.0),
        .waypoint_index = 0,
        .timestamp_ms = 0,
        .sequence_number = 1
    };
    
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    check_condition("First waypoint: Status OK", output.status == STATUS_OK);
    check_condition("First waypoint: target > 0", output.target_waypoint_idx > 0);
    printf("  At waypoint 0, target: %u\n", output.target_waypoint_idx);
    
    // Test at last waypoint (index 9)
    state.x_fp = FP_CONST(18.0);
    state.waypoint_index = 9;
    
    pure_pursuit_fpga(0, NULL, 0, &state, &params, &output, &traj_loaded, &traj_size);
    check_condition("Last waypoint: Status OK", output.status == STATUS_OK);
    printf("  At waypoint 9, target: %u\n", output.target_waypoint_idx);
}

/*===========================================================================
 * Test 9: Additional Fixed-Point Math Edge Cases
 *===========================================================================*/

static void test_fp_math_edge_cases(void)
{
    printf("\n========== Test FP Math Edge Cases ==========\n");
    
    // Division by very small number (not zero)
    fixed_point_t small = FP_CONST(0.001);
    fixed_point_t result = fp_div(FP_ONE, small);
    check_fp("1.0 / 0.001 = 1000", result, 1000.0);
    
    // Division by zero (should not crash)
    result = fp_div(FP_ONE, 0);
    check_condition("div by zero returns 0", result == 0);
    
    // Sin/cos at negative angles
    check_fp("sin(-π/2)", fp_sin(-FP_PI_HALF), -1.0);
    check_fp("cos(-π)", fp_cos(-FP_PI), -1.0);
    
    // Large angle normalization
    fixed_point_t large_angle = FP_CONST(10.0);  // ~3π
    fixed_point_t normalized = fp_normalize_angle(large_angle);
    double norm_deg = FP_TO_DOUBLE(normalized) * 180.0 / M_PI;
    check_condition("Large angle normalized to [-180, 180]", 
                    norm_deg >= -180.0 && norm_deg <= 180.0);
    printf("  10.0 rad normalized to: %.2f rad (%.1f deg)\n", 
           FP_TO_DOUBLE(normalized), norm_deg);
    
    // Sqrt of very small number
    result = fp_sqrt(FP_CONST(0.0001));
    check_fp("sqrt(0.0001) = 0.01", result, 0.01);
    
    // Sqrt of zero
    result = fp_sqrt(0);
    check_condition("sqrt(0) = 0", result == 0);
    
    // atan2 with zero args
    check_fp("atan2(0, 1) = 0", fp_atan2(0, FP_ONE), 0.0);
    check_fp("atan2(1, 0) = π/2", fp_atan2(FP_ONE, 0), M_PI / 2.0);
    check_fp("atan2(0, -1) = π", fp_atan2(0, -FP_ONE), M_PI);
    check_fp("atan2(-1, 0) = -π/2", fp_atan2(-FP_ONE, 0), -M_PI / 2.0);
}

/*===========================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("========================================\n");
    printf("Pure Pursuit FPGA Testbench\n");
    printf("========================================\n");
    
    test_fp_math();
    test_straight_path();
    test_curved_path();
    test_error_conditions();    test_trajectory_wraparound();
    test_zero_velocity();
    test_division_safety();
    test_boundary_waypoints();
    test_fp_math_edge_cases();    
    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");
    
    return (tests_failed > 0) ? 1 : 0;
}
