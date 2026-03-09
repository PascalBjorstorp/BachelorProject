/**
 * @file test_mpc_cpu.c
 * @brief CPU MPC Testbench — verifies Riccati-ADMM solver correctness
 *
 * Tests the mpc_compute_optimal_control() API with:
 *   1. Straight-line tracking at 5 m/s
 *   2. Lateral offset recovery
 *   3. Heading error correction
 *   4. Curved path following
 *   5. Stability: repeated calls produce consistent output
 *
 * Build:
 *   gcc -O3 -march=native -I ../include -std=c99 -D_POSIX_C_SOURCE=200809L \
 *       -o test_mpc_cpu test_mpc_cpu.c \
 *       ../src/fp_math.c ../src/vehicle_model.c \
 *       ../src/riccati_solver.c ../src/mpc_riccati.c -lm
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "mpc.h"
#include "mpc_types.h"
#include "fp_math.h"

#define N_HORIZON 20

static int total_checks = 0;
static int passed_checks = 0;

#define CHECK(cond, msg) do { \
    total_checks++; \
    if (cond) { passed_checks++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

#define CHECK_RANGE(val, lo, hi, name) do { \
    total_checks++; \
    if ((val) >= (lo) && (val) <= (hi)) { passed_checks++; } \
    else { printf("  FAIL: %s = %.6f not in [%.3f, %.3f]\n", \
                  name, (double)(val), (double)(lo), (double)(hi)); } \
} while(0)

static void build_straight_reference(
    TrajectoryReferencePoint_t *ref,
    double velocity, double curvature)
{
    for (int k = 0; k < N_HORIZON; k++) {
        ref[k].reference_lateral_error_meters = 0;
        ref[k].reference_heading_error_radians = 0;
        ref[k].reference_velocity_meters_per_second = DOUBLE_TO_FP(velocity);
        ref[k].reference_lateral_velocity_meters_per_second = 0;
        ref[k].reference_yaw_rate_radians_per_second =
            DOUBLE_TO_FP(curvature * velocity);
        ref[k].path_curvature_radians_per_meter = DOUBLE_TO_FP(curvature);
        ref[k].left_wall_bound_meters = DOUBLE_TO_FP(2.0);
        ref[k].right_wall_bound_meters = DOUBLE_TO_FP(2.0);
        ref[k].reference_acceleration_meters_per_second_squared = 0;
    }
}

int main(void)
{
    printf("=== CPU MPC Testbench (Riccati-ADMM) ===\n\n");

    /* Initialize MPC */
    mpc_initialize();

    FrenetState_t state;
    TrajectoryReferencePoint_t ref[N_HORIZON];
    MpcSolverResult_t result;

    /* ---------------------------------------------------------------
     * Phase 1: Straight-line tracking (zero error, v=5 m/s)
     * --------------------------------------------------------------- */
    printf("Phase 1: Straight-line tracking test...\n");
    mpc_reset();

    state.lateral_error_meters = 0;
    state.heading_error_radians = 0;
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    build_straight_reference(ref, 5.0, 0.0);
    mpc_compute_optimal_control(&state, ref, &result);

    double steer1 = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    double accel1 = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer1, steer1 * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel1);
    printf("    Status:   %d, Iters: %d\n", result.solver_status, result.iterations_used);

    CHECK_RANGE(steer1, -0.05, 0.05, "straight_steer");
    CHECK_RANGE(accel1, -2.0, 2.0, "straight_accel");
    CHECK(result.solver_status == MPC_STATUS_SUCCESS ||
          result.solver_status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED,
          "straight_status");

    /* ---------------------------------------------------------------
     * Phase 2: Lateral offset recovery (0.5m left of path)
     * --------------------------------------------------------------- */
    printf("\nPhase 2: Lateral offset recovery test...\n");
    mpc_reset();

    state.lateral_error_meters = DOUBLE_TO_FP(0.5);
    state.heading_error_radians = 0;
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    build_straight_reference(ref, 5.0, 0.0);
    mpc_compute_optimal_control(&state, ref, &result);

    double steer2 = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    double accel2 = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer2, steer2 * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel2);
    printf("    Status:   %d, Iters: %d\n", result.solver_status, result.iterations_used);

    /* Should steer RIGHT (negative) to correct left offset */
    CHECK(steer2 < -0.001, "lateral_offset_steers_right");
    CHECK_RANGE(steer2, -0.5, 0.0, "lateral_offset_range");

    /* ---------------------------------------------------------------
     * Phase 3: Heading error correction (10 deg right)
     * --------------------------------------------------------------- */
    printf("\nPhase 3: Heading error correction test...\n");
    mpc_reset();

    state.lateral_error_meters = 0;
    state.heading_error_radians = DOUBLE_TO_FP(10.0 * M_PI / 180.0);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    build_straight_reference(ref, 5.0, 0.0);
    mpc_compute_optimal_control(&state, ref, &result);

    double steer3 = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    double accel3 = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer3, steer3 * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel3);
    printf("    Status:   %d, Iters: %d\n", result.solver_status, result.iterations_used);

    /* Should steer LEFT (negative) to correct right heading error */
    CHECK(steer3 < -0.001, "heading_error_steers_left");
    CHECK_RANGE(steer3, -0.5, 0.0, "heading_correction_range");

    /* ---------------------------------------------------------------
     * Phase 4: Curved path (kappa=0.2, turn left)
     * --------------------------------------------------------------- */
    printf("\nPhase 4: Curved section test...\n");
    mpc_reset();

    state.lateral_error_meters = 0;
    state.heading_error_radians = 0;
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    build_straight_reference(ref, 5.0, 0.2);  /* kappa=0.2 (5m radius) */
    mpc_compute_optimal_control(&state, ref, &result);

    double steer4 = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    double accel4 = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer4, steer4 * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel4);
    printf("    Status:   %d, Iters: %d\n", result.solver_status, result.iterations_used);

    /* Should steer LEFT (positive) for positive curvature */
    CHECK(steer4 > 0.01, "curve_steers_left");
    CHECK_RANGE(steer4, 0.0, 0.5, "curve_steering_range");

    /* ---------------------------------------------------------------
     * Phase 5: Stability test (5 consecutive calls)
     * --------------------------------------------------------------- */
    printf("\nPhase 5: Stability test (5 consecutive calls)...\n");
    mpc_reset();

    state.lateral_error_meters = DOUBLE_TO_FP(0.1);
    state.heading_error_radians = DOUBLE_TO_FP(0.05);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    build_straight_reference(ref, 5.0, 0.0);

    double prev_steer = 0.0;
    int stable = 1;
    for (int call = 0; call < 5; call++) {
        mpc_compute_optimal_control(&state, ref, &result);
        double s = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
        double a = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);
        printf("    Call %d: steer=%.4f, accel=%.4f, status=%d, iters=%d\n",
               call, s, a, result.solver_status, result.iterations_used);

        if (call > 0 && fabs(s - prev_steer) > 0.3) {
            stable = 0;
        }
        prev_steer = s;

        /* Feed actual control back */
        ControlInput_t actual;
        actual.steering_angle_radians = result.optimal_control.steering_angle_radians;
        actual.acceleration_meters_per_second_squared =
            result.optimal_control.acceleration_meters_per_second_squared;
        mpc_set_actual_previous_control(&actual);
    }
    CHECK(stable, "stability_consecutive_calls");

    /* ---------------------------------------------------------------
     * Summary
     * --------------------------------------------------------------- */
    printf("\n=== %d / %d checks passed ===\n", passed_checks, total_checks);
    if (passed_checks == total_checks) {
        printf("=== Test PASSED ===\n");
        return 0;
    } else {
        printf("=== Test FAILED ===\n");
        return 1;
    }
}
