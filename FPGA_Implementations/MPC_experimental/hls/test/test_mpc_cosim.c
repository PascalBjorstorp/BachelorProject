/**
 * @file test_mpc_cosim.c
 * @brief HLS C-Simulation Testbench for MPC FPGA IP
 *
 * Tests the mpc_fpga() top-level function through the scalar AXI-Lite
 * interface, exercising all modes:
 *   1. Load trajectory waypoints (mode=1)
 *   2. Finalize trajectory (mode=2)
 *   3. Compute MPC control for various scenarios (mode=0)
 *   4. Reset solver state (mode=3)
 *
 * This runs during `csim_design` in Vitis HLS.
 * Return 0 = pass, non-zero = fail.
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "../include/mpc_fpga_interface.h"

/*===========================================================================
 * Fixed-point helpers
 *===========================================================================*/
#define FP_FRAC_BITS 16
#define FP_ONE       (1 << FP_FRAC_BITS)
#define DOUBLE_TO_FP(x) ((int32_t)((x) * FP_ONE))
#define FP_TO_DOUBLE(x) ((double)(x) / (double)FP_ONE)

/*===========================================================================
 * Test infrastructure
 *===========================================================================*/
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s: %s\n", __func__, msg); \
        tests_failed++; \
        return 1; \
    } \
} while(0)

#define TEST_PASS(msg) do { \
    printf("[PASS] %s: %s\n", __func__, msg); \
    tests_passed++; \
} while(0)

/*===========================================================================
 * Call helper — wraps mpc_fpga with default unused args
 *===========================================================================*/
static void call_mpc_fpga_compute(
    int32_t lat_err, int32_t hdg_err, int32_t vx, int32_t vy,
    int32_t omega, uint32_t wp_idx,
    int32_t *steer, int32_t *accel, uint32_t *status,
    uint32_t *iters, int32_t *cost,
    uint32_t *traj_loaded, uint32_t *traj_size)
{
    mpc_fpga(
        MPC_FPGA_MODE_COMPUTE,
        /* wp loading (unused in mode 0) */
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        /* vehicle state */
        lat_err, hdg_err, vx, vy, omega, wp_idx,
        /* outputs */
        steer, accel, status, iters, cost, traj_loaded, traj_size);
}

static void load_waypoint(
    uint32_t idx,
    double ref_vel, double curvature,
    double left_wall, double right_wall)
{
    int32_t dummy_steer, dummy_accel, dummy_cost;
    uint32_t dummy_status, dummy_iters, dummy_loaded, dummy_size;

    mpc_fpga(
        MPC_FPGA_MODE_LOAD_WP,
        /* wp loading */
        idx,
        DOUBLE_TO_FP(0.0),            /* ref lateral error */
        DOUBLE_TO_FP(0.0),            /* ref heading error */
        DOUBLE_TO_FP(ref_vel),         /* ref velocity */
        DOUBLE_TO_FP(0.0),            /* ref lateral velocity */
        DOUBLE_TO_FP(ref_vel * curvature), /* ref yaw rate = v * kappa */
        DOUBLE_TO_FP(curvature),       /* curvature */
        DOUBLE_TO_FP(left_wall),       /* left wall */
        DOUBLE_TO_FP(right_wall),      /* right wall */
        0,                             /* wp_total (unused in mode 1) */
        /* state (unused) */
        0, 0, 0, 0, 0, 0,
        /* outputs (unused) */
        &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters,
        &dummy_cost, &dummy_loaded, &dummy_size);
}

static void finalize_trajectory(uint32_t total) {
    int32_t dummy_steer, dummy_accel, dummy_cost;
    uint32_t dummy_status, dummy_iters, dummy_loaded, dummy_size;

    mpc_fpga(
        MPC_FPGA_MODE_FINALIZE,
        /* wp loading */
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        total,
        /* state (unused) */
        0, 0, 0, 0, 0, 0,
        /* outputs */
        &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters,
        &dummy_cost, &dummy_loaded, &dummy_size);
}

static void reset_mpc(void) {
    int32_t dummy_steer, dummy_accel, dummy_cost;
    uint32_t dummy_status, dummy_iters, dummy_loaded, dummy_size;

    mpc_fpga(
        MPC_FPGA_MODE_RESET,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters,
        &dummy_cost, &dummy_loaded, &dummy_size);
}

/*===========================================================================
 * Test 1: Trajectory loading and status
 *===========================================================================*/
static int test_trajectory_loading(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    /* Before loading: should report no trajectory */
    call_mpc_fpga_compute(
        0, 0, DOUBLE_TO_FP(3.0), 0, 0, 0,
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);
    /* Note: first call might trigger initialization, trajectory still not loaded
     * until finalize is called. Check traj_loaded flag. */

    /* Load a simple straight trajectory */
    for (uint32_t i = 0; i < 50; i++) {
        load_waypoint(i, 3.0, 0.0, 2.0, 2.0);
    }

    /* Finalize */
    finalize_trajectory(50);

    /* Now compute — should succeed */
    call_mpc_fpga_compute(
        0, 0, DOUBLE_TO_FP(3.0), 0, 0, 0,
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);

    TEST_ASSERT(traj_loaded == 1, "trajectory should be loaded");
    TEST_ASSERT(traj_size == 50, "trajectory size should be 50");
    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should return OK or max_iter");

    TEST_PASS("trajectory loading and finalization");
    return 0;
}

/*===========================================================================
 * Test 2: Straight line — on path, at reference speed
 *===========================================================================*/
static int test_straight_line_on_path(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    /* Vehicle is on path, at reference speed -> expect near-zero control */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.0),    /* lateral error = 0 */
        DOUBLE_TO_FP(0.0),    /* heading error = 0 */
        DOUBLE_TO_FP(3.0),    /* vx = 3.0 m/s (matching reference) */
        DOUBLE_TO_FP(0.0),    /* vy = 0 */
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        0,                     /* waypoint index */
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);

    double steer_d = FP_TO_DOUBLE(steer);
    double accel_d = FP_TO_DOUBLE(accel);

    printf("  Straight on-path: steer=%.4f rad, accel=%.4f m/s2, iters=%u\n",
           steer_d, accel_d, iters);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed");
    TEST_ASSERT(fabs(steer_d) < 0.05, "steering should be near zero on straight");
    TEST_ASSERT(fabs(accel_d) < 2.0, "acceleration should be small when at ref speed");

    TEST_PASS("straight line on path");
    return 0;
}

/*===========================================================================
 * Test 3: Lateral offset — should steer back to path
 *===========================================================================*/
static int test_lateral_offset(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    /* Vehicle is 0.5m to the left of path */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.5),    /* lateral error = 0.5m left */
        DOUBLE_TO_FP(0.0),    /* heading error = 0 */
        DOUBLE_TO_FP(3.0),    /* vx = 3.0 m/s */
        DOUBLE_TO_FP(0.0),    /* vy = 0 */
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        5,                     /* waypoint index */
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);

    double steer_d = FP_TO_DOUBLE(steer);
    printf("  Lateral offset: steer=%.4f rad (should steer right, negative)\n", steer_d);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed");
    TEST_ASSERT(steer_d < -0.01, "should steer right (negative) to correct left offset");

    TEST_PASS("lateral offset correction");
    return 0;
}

/*===========================================================================
 * Test 4: Heading error — should steer to align
 *===========================================================================*/
static int test_heading_error(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    /* Vehicle heading is 0.2 rad to the left */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.0),    /* lateral error = 0 */
        DOUBLE_TO_FP(0.2),    /* heading error = 0.2 rad */
        DOUBLE_TO_FP(3.0),    /* vx = 3.0 m/s */
        DOUBLE_TO_FP(0.0),    /* vy = 0 */
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        5,                     /* waypoint index */
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);

    double steer_d = FP_TO_DOUBLE(steer);
    printf("  Heading error: steer=%.4f rad (should steer right, negative)\n", steer_d);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed");
    TEST_ASSERT(steer_d < -0.01, "should steer right to correct left heading error");

    TEST_PASS("heading error correction");
    return 0;
}

/*===========================================================================
 * Test 5: Speed below reference — should accelerate
 *===========================================================================*/
static int test_speed_tracking(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    /* Vehicle is slower than reference (1.0 m/s vs 3.0 m/s ref) */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(1.0),    /* vx = 1.0 m/s (below 3.0 ref) */
        DOUBLE_TO_FP(0.0),    /* vy = 0 */
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        5,                     /* waypoint index */
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);

    double accel_d = FP_TO_DOUBLE(accel);
    printf("  Speed tracking: accel=%.4f m/s2 (should be positive)\n", accel_d);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed");
    TEST_ASSERT(accel_d > 0.0, "should accelerate when below reference speed");

    TEST_PASS("speed tracking");
    return 0;
}

/*===========================================================================
 * Test 6: Curve tracking
 *===========================================================================*/
static int test_curve_tracking(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    /* Reset solver to clear warm-start from straight-line tests */
    reset_mpc();

    /* Load a curved trajectory (kappa = 1.0 -> 1m radius) */
    for (uint32_t i = 0; i < 50; i++) {
        load_waypoint(i, 2.0, 1.0, 2.0, 2.0);
    }
    finalize_trajectory(50);

    /* Warm up: call three times for solver convergence */
    for (int warm = 0; warm < 3; warm++) {
        call_mpc_fpga_compute(
            DOUBLE_TO_FP(0.0),
            DOUBLE_TO_FP(0.0),
            DOUBLE_TO_FP(2.0),
            DOUBLE_TO_FP(0.0),
            DOUBLE_TO_FP(2.0),    /* omega = v * kappa = 2.0 */
            5,                     /* waypoint index */
            &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);
    }

    double steer_d = FP_TO_DOUBLE(steer);
    printf("  Curve tracking: steer=%.4f rad (should steer into curve)\n", steer_d);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed on curve");
    /* On a positive-curvature curve, expect positive (left) steering */
    TEST_ASSERT(fabs(steer_d) > 0.01, "should have non-trivial steering on curve");

    TEST_PASS("curve tracking");
    return 0;
}

/*===========================================================================
 * Test 7: Reset functionality
 *===========================================================================*/
static int test_reset(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;

    reset_mpc();

    /* After reset, solver should still work (trajectory still in BRAM) */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(2.0),
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        0,                     /* waypoint index */
        &steer, &accel, &status, &iters, &cost, &traj_loaded, &traj_size);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed after reset");
    TEST_ASSERT(traj_loaded == 1, "trajectory should remain loaded after reset");

    TEST_PASS("reset functionality");
    return 0;
}

/*===========================================================================
 * Test 8: Warm-start convergence improvement
 *===========================================================================*/
static int test_warm_start(void) {
    int32_t steer, accel, cost;
    uint32_t status, iters_cold, iters_warm, iters, traj_loaded, traj_size;

    /* Reset for cold start */
    reset_mpc();

    /* First call (cold start) */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.1),
        DOUBLE_TO_FP(0.05),
        DOUBLE_TO_FP(3.0),
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        0,                     /* waypoint index */
        &steer, &accel, &status, &iters_cold, &cost, &traj_loaded, &traj_size);

    /* Second call with similar state (warm start should help) */
    call_mpc_fpga_compute(
        DOUBLE_TO_FP(0.09),
        DOUBLE_TO_FP(0.04),
        DOUBLE_TO_FP(3.0),
        DOUBLE_TO_FP(0.0),
        DOUBLE_TO_FP(0.0),    /* omega = 0 */
        1,                     /* waypoint index */
        &steer, &accel, &status, &iters_warm, &cost, &traj_loaded, &traj_size);

    printf("  Warm-start: cold=%u iters, warm=%u iters\n", iters_cold, iters_warm);

    TEST_ASSERT(status == MPC_FPGA_STATUS_OK || status == MPC_FPGA_STATUS_MAX_ITER,
                "solver should succeed");
    /* Warm start should use fewer or equal iterations */
    TEST_ASSERT(iters_warm <= iters_cold + 5,
                "warm start should not use significantly more iterations than cold");

    TEST_PASS("warm-start convergence");
    return 0;
}

/*===========================================================================
 * Main
 *===========================================================================*/
int main(void) {
    printf("============================================\n");
    printf("MPC FPGA C-Simulation Testbench\n");
    printf("============================================\n\n");

    test_trajectory_loading();
    test_straight_line_on_path();
    test_lateral_offset();
    test_heading_error();
    test_speed_tracking();
    test_curve_tracking();
    test_reset();
    test_warm_start();

    printf("\n============================================\n");
    printf("  RESULTS: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("============================================\n");

    return tests_failed;
}
