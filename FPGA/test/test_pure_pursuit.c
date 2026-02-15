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
 * Top-level function (scalar interface)
 *===========================================================================*/

extern void pure_pursuit_fpga(
    uint32_t mode,
    uint32_t wp_index,
    int32_t wp_x, int32_t wp_y, int32_t wp_theta,
    int32_t wp_vel, int32_t wp_kappa,
    uint32_t wp_total,
    int32_t st_x, int32_t st_y, int32_t st_theta, int32_t st_vel,
    uint32_t st_wp_idx,
    int32_t p_min_la, int32_t p_max_la, int32_t p_la_gain,
    int32_t p_wheelbase, int32_t p_max_steer, int32_t p_max_vel,
    uint32_t p_la_points,
    int32_t* out_steering, int32_t* out_velocity,
    int32_t* out_cte, int32_t* out_heading_err,
    int32_t* out_lookahead, uint32_t* out_target_wp,
    uint32_t* out_status,
    uint32_t* out_traj_loaded, uint32_t* out_traj_size
);

/*===========================================================================
 * Helper: load an array of waypoints one at a time, then finalize
 *===========================================================================*/
typedef struct {
    int32_t x_fp, y_fp, theta_fp, velocity_fp, kappa_fp;
} TestWaypoint;

static void load_waypoints(const TestWaypoint* wps, int count,
                           uint32_t* out_traj_loaded, uint32_t* out_traj_size)
{
    int32_t  dummy_s = 0, dummy_v = 0, dummy_c = 0, dummy_h = 0, dummy_l = 0;
    uint32_t dummy_tw = 0, dummy_st = 0;

    for (int i = 0; i < count; i++) {
        pure_pursuit_fpga(1,  /* mode = LOAD_WAYPOINT */
            (uint32_t)i, wps[i].x_fp, wps[i].y_fp, wps[i].theta_fp,
            wps[i].velocity_fp, wps[i].kappa_fp, (uint32_t)count,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            &dummy_s, &dummy_v, &dummy_c, &dummy_h,
            &dummy_l, &dummy_tw, &dummy_st,
            out_traj_loaded, out_traj_size);
    }

    /* Finalize (mode=2) */
    pure_pursuit_fpga(2,
        0, 0, 0, 0, 0, 0, (uint32_t)count,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        &dummy_s, &dummy_v, &dummy_c, &dummy_h,
        &dummy_l, &dummy_tw, &dummy_st,
        out_traj_loaded, out_traj_size);
}

/*===========================================================================
 * Helper: call compute (mode=0) and collect outputs
 *===========================================================================*/
typedef struct {
    int32_t  steering_fp, velocity_fp, cte_fp, heading_err_fp, lookahead_fp;
    uint32_t target_wp, status;
    uint32_t traj_loaded, traj_size;
} ComputeResult;

static ComputeResult compute(
    int32_t st_x, int32_t st_y, int32_t st_theta, int32_t st_vel,
    uint32_t st_wp_idx,
    int32_t p_min_la, int32_t p_max_la, int32_t p_la_gain,
    int32_t p_wheelbase, int32_t p_max_steer, int32_t p_max_vel,
    uint32_t p_la_points)
{
    ComputeResult r;
    memset(&r, 0, sizeof(r));

    pure_pursuit_fpga(0,
        0, 0, 0, 0, 0, 0, 0,   /* wp fields unused in mode 0 */
        st_x, st_y, st_theta, st_vel, st_wp_idx,
        p_min_la, p_max_la, p_la_gain,
        p_wheelbase, p_max_steer, p_max_vel, p_la_points,
        &r.steering_fp, &r.velocity_fp,
        &r.cte_fp, &r.heading_err_fp,
        &r.lookahead_fp, &r.target_wp,
        &r.status,
        &r.traj_loaded, &r.traj_size);

    return r;
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

static void test_straight_path(void)
{
    printf("\n========== Test Straight Path ==========\n");
    
    // Create straight path along X-axis
    TestWaypoint waypoints[20];
    for (int i = 0; i < 20; i++) {
        waypoints[i].x_fp        = FP_CONST(i * 1.0);
        waypoints[i].y_fp        = 0;
        waypoints[i].theta_fp    = 0;
        waypoints[i].velocity_fp = FP_CONST(3.0);
        waypoints[i].kappa_fp    = 0;
    }
    
    // Load trajectory
    uint32_t traj_loaded = 0, traj_size = 0;
    load_waypoints(waypoints, 20, &traj_loaded, &traj_size);
    check_condition("Trajectory loaded", traj_loaded == 1);
    check_condition("Trajectory size = 20", traj_size == 20);
    
    // Parameters
    int32_t  p_min_la    = FP_CONST(1.5);
    int32_t  p_max_la    = FP_CONST(3.0);
    int32_t  p_la_gain   = FP_CONST(0.3);
    int32_t  p_wheelbase = FP_CONST(0.324);
    int32_t  p_max_steer = FP_CONST(0.4189);
    int32_t  p_max_vel   = FP_CONST(5.0);
    uint32_t p_la_points = 10;
    
    // Test 1: Vehicle on path, pointing along path
    ComputeResult r = compute(
        FP_CONST(2.0), 0, 0, FP_CONST(2.0), 2,
        p_min_la, p_max_la, p_la_gain,
        p_wheelbase, p_max_steer, p_max_vel, p_la_points);
    
    check_condition("Status OK", r.status == STATUS_OK);
    check_fp("Steering ~0 on straight path", r.steering_fp, 0.0);
    check_fp("Velocity = 3.0", r.velocity_fp, 3.0);
    check_fp("CTE ~0", r.cte_fp, 0.0);
    
    // Test 2: Vehicle slightly off path (should steer towards path)
    r = compute(
        FP_CONST(2.0), FP_CONST(0.5), 0, FP_CONST(2.0), 2,
        p_min_la, p_max_la, p_la_gain,
        p_wheelbase, p_max_steer, p_max_vel, p_la_points);
    
    check_condition("Status OK (off-path)", r.status == STATUS_OK);
    double steering_deg = FP_TO_DOUBLE(r.steering_fp) * 180.0 / M_PI;
    check_condition("Steers right (negative steering)", r.steering_fp < 0);
    printf("  Steering angle: %.2f degrees (should be negative)\n", steering_deg);
}

/*===========================================================================
 * Test 3: Curved Path
 *===========================================================================*/

static void test_curved_path(void)
{
    printf("\n========== Test Curved Path ==========\n");
    
    // Create a quarter circle path (90 degree turn)
    TestWaypoint waypoints[25];
    double radius = 5.0;
    
    for (int i = 0; i < 25; i++) {
        double angle = (double)i / 24.0 * (M_PI / 2.0);
        waypoints[i].x_fp        = FP_CONST(radius * sin(angle));
        waypoints[i].y_fp        = FP_CONST(radius * (1.0 - cos(angle)));
        waypoints[i].theta_fp    = FP_CONST(angle);
        waypoints[i].velocity_fp = FP_CONST(2.0);
        waypoints[i].kappa_fp    = FP_CONST(1.0 / radius);
    }
    
    uint32_t traj_loaded = 0, traj_size = 0;
    load_waypoints(waypoints, 25, &traj_loaded, &traj_size);
    
    // Test at start of curve
    ComputeResult r = compute(
        waypoints[0].x_fp, waypoints[0].y_fp, waypoints[0].theta_fp,
        FP_CONST(2.0), 0,
        FP_CONST(1.0), FP_CONST(2.5), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 10);
    
    check_condition("Status OK (curve)", r.status == STATUS_OK);
    check_condition("Steers left (positive)", r.steering_fp > 0);
    
    double steering_deg = FP_TO_DOUBLE(r.steering_fp) * 180.0 / M_PI;
    printf("  Steering angle: %.2f degrees (for R=5m curve)\n", steering_deg);
    printf("  Target waypoint: %u\n", r.target_wp);
}

/*===========================================================================
 * Test 4: Error Conditions
 *===========================================================================*/

static void test_error_conditions(void)
{
    printf("\n========== Test Error Conditions ==========\n");
    
    // Load minimal trajectory (1 waypoint)
    TestWaypoint wp = { .x_fp = 0, .y_fp = 0, .theta_fp = 0,
                        .velocity_fp = FP_CONST(1.0), .kappa_fp = 0 };
    uint32_t traj_loaded = 0, traj_size = 0;
    load_waypoints(&wp, 1, &traj_loaded, &traj_size);
    check_condition("Single waypoint loaded", traj_size == 1);
    
    // Test with invalid waypoint index (should wrap around)
    ComputeResult r = compute(
        0, 0, 0, 0, 99999,
        FP_CONST(1.5), FP_CONST(3.0), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 10);
    check_condition("Status OK with wrapped index", r.status == STATUS_OK);
}

/*===========================================================================
 * Test 5: Trajectory Wrap-Around
 *===========================================================================*/

static void test_trajectory_wraparound(void)
{
    printf("\n========== Test Trajectory Wrap-Around ==========\n");
    
    // Create 100-waypoint circular path (closed loop)
    TestWaypoint waypoints[100];
    double radius = 10.0;
    
    for (int i = 0; i < 100; i++) {
        double angle = (double)i / 100.0 * 2.0 * M_PI;
        waypoints[i].x_fp        = FP_CONST(radius * cos(angle));
        waypoints[i].y_fp        = FP_CONST(radius * sin(angle));
        waypoints[i].theta_fp    = FP_CONST(angle + M_PI / 2.0);
        waypoints[i].velocity_fp = FP_CONST(3.0);
        waypoints[i].kappa_fp    = FP_CONST(1.0 / radius);
    }
    
    uint32_t traj_loaded = 0, traj_size = 0;
    load_waypoints(waypoints, 100, &traj_loaded, &traj_size);
    check_condition("100 waypoints loaded", traj_size == 100);
    
    // Test at waypoint 98 (near end, should search forward to 99, 0, 1, 2...)
    ComputeResult r = compute(
        waypoints[98].x_fp, waypoints[98].y_fp, waypoints[98].theta_fp,
        FP_CONST(3.0), 98,
        FP_CONST(2.0), FP_CONST(4.0), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 20);
    
    check_condition("Wraparound: Status OK", r.status == STATUS_OK);
    printf("  Current waypoint: 98, Target waypoint: %u\n", r.target_wp);
    int wrapped = (r.target_wp < 5) || (r.target_wp >= 98);
    check_condition("Target wrapped or near end", wrapped);
}

/*===========================================================================
 * Test 6: Zero Velocity Edge Case
 *===========================================================================*/

static void test_zero_velocity(void)
{
    printf("\n========== Test Zero Velocity ==========\n");
    
    // Use previous 100-waypoint circular trajectory (still loaded)
    
    // Zero velocity - lookahead should be min_lookahead
    ComputeResult r = compute(
        FP_CONST(10.0), FP_CONST(0.0), FP_CONST(M_PI / 2.0),
        0,  /* ZERO velocity */
        0,
        FP_CONST(1.5), FP_CONST(3.0), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 10);
    
    check_condition("Zero velocity: Status OK", r.status == STATUS_OK);
    check_fp("Lookahead = min (1.5)", r.lookahead_fp, 1.5);
    
    // Negative velocity (reversing)
    r = compute(
        FP_CONST(10.0), FP_CONST(0.0), FP_CONST(M_PI / 2.0),
        FP_CONST(-2.0),
        0,
        FP_CONST(1.5), FP_CONST(3.0), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 10);
    
    check_condition("Negative velocity: Status OK", r.status == STATUS_OK);
    double lookahead_expected = 1.5 + 0.3 * 2.0;
    check_fp("Lookahead with negative v", r.lookahead_fp, lookahead_expected);
}

/*===========================================================================
 * Test 7: Division Safety (L_sq near zero)
 *===========================================================================*/

static void test_division_safety(void)
{
    printf("\n========== Test Division Safety ==========\n");
    
    // Create simple 5-waypoint path
    TestWaypoint waypoints[5];
    for (int i = 0; i < 5; i++) {
        waypoints[i].x_fp        = FP_CONST(i * 1.0);
        waypoints[i].y_fp        = 0;
        waypoints[i].theta_fp    = 0;
        waypoints[i].velocity_fp = FP_CONST(2.0);
        waypoints[i].kappa_fp    = 0;
    }
    
    uint32_t traj_loaded = 0, traj_size = 0;
    load_waypoints(waypoints, 5, &traj_loaded, &traj_size);
    
    // Car exactly on top of waypoint 2
    ComputeResult r = compute(
        FP_CONST(2.0), 0, 0, FP_CONST(0.1), 2,
        FP_CONST(0.1), FP_CONST(0.5), FP_CONST(0.1),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 5);
    
    check_condition("On waypoint: Status OK", r.status == STATUS_OK);
    double steering_deg = FP_TO_DOUBLE(r.steering_fp) * 180.0 / M_PI;
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
    TestWaypoint waypoints[10];
    for (int i = 0; i < 10; i++) {
        waypoints[i].x_fp        = FP_CONST(i * 2.0);
        waypoints[i].y_fp        = 0;
        waypoints[i].theta_fp    = 0;
        waypoints[i].velocity_fp = FP_CONST(3.0);
        waypoints[i].kappa_fp    = 0;
    }
    
    uint32_t traj_loaded = 0, traj_size = 0;
    load_waypoints(waypoints, 10, &traj_loaded, &traj_size);
    
    // Test at first waypoint (index 0)
    ComputeResult r = compute(
        FP_CONST(0.0), 0, 0, FP_CONST(2.0), 0,
        FP_CONST(2.0), FP_CONST(5.0), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 5);
    
    check_condition("First waypoint: Status OK", r.status == STATUS_OK);
    check_condition("First waypoint: target > 0", r.target_wp > 0);
    printf("  At waypoint 0, target: %u\n", r.target_wp);
    
    // Test at last waypoint (index 9)
    r = compute(
        FP_CONST(18.0), 0, 0, FP_CONST(2.0), 9,
        FP_CONST(2.0), FP_CONST(5.0), FP_CONST(0.3),
        FP_CONST(0.324), FP_CONST(0.4189), FP_CONST(5.0), 5);
    
    check_condition("Last waypoint: Status OK", r.status == STATUS_OK);
    printf("  At waypoint 9, target: %u\n", r.target_wp);
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
    test_error_conditions();
    test_trajectory_wraparound();
    test_zero_velocity();
    test_division_safety();
    test_boundary_waypoints();
    test_fp_math_edge_cases();
    
    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");
    
    return (tests_failed > 0) ? 1 : 0;
}
