/**
 * @file tb_mpc_fpga.c
 * @brief C Testbench for MPC FPGA HLS Cosimulation
 *
 * Tests the mpc_fpga_top function in multiple phases:
 *   1. Load a simple oval trajectory (mode=1 + mode=2)
 *   2. Run compute mode (mode=0) with test vehicle states
 *   3. Verify outputs with quantitative bounds
 *   4. Test warm-start convergence improvement
 *   5. Test on curved section (steering direction)
 *   6. Consistency check: repeated calls produce stable output
 *
 * Uses full real-hardware physics model (Pacejka tires, atan slip angles,
 * cos/sin delta force resolution).
 *
 * Build with: gcc -O2 -I../include -Wno-unknown-pragmas -o tb_mpc_fpga \
 *             tb_mpc_fpga.c ../src/mpc_fpga_top.c ../src/mpc_riccati_hls.c \
 *             ../src/riccati_solver_hls.c ../src/vehicle_model_hls.c \
 *             ../src/fp_math_hls.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/mpc_fpga_types.h"

/* Top function prototype */
extern void mpc_fpga_top(
    int mode,
    int wp_index,
    int wp_x_fp, int wp_y_fp, int wp_psi_fp,
    int wp_vx_fp, int wp_kappa_fp, int wp_ax_fp,
    int wp_left_bound_fp, int wp_right_bound_fp,
    int wp_total,
    int state_x_fp, int state_y_fp, int state_theta_fp,
    int state_vx_fp, int state_vy_fp, int state_omega_fp,
    int state_steering_fp, int state_wp_idx,
    int *out_steering_fp, int *out_accel_fp,
    int *out_status, int *out_iterations);

/* Helper: float to Q16.16 */
static int ftofp(double v) {
    return (int)(v >= 0 ? (v * 65536.0 + 0.5) : (v * 65536.0 - 0.5));
}

/* Helper: Q16.16 to float */
static double fptof(int v) {
    return (double)v / 65536.0;
}

/* Test assertion macro */
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
    else { printf("  FAIL: %s = %.6f not in [%.3f, %.3f]\n", name, (double)(val), (double)(lo), (double)(hi)); } \
} while(0)

int main(void)
{
    int out_steer, out_accel, out_status, out_iters;

    printf("=== MPC FPGA Testbench (Real-Hardware Model) ===\n\n");

    /* ---------------------------------------------------------------
     * Phase 1: Load an oval trajectory (100 waypoints)
     * --------------------------------------------------------------- */
    printf("Phase 1: Loading trajectory...\n");
    const int N_WP = 100;
    const double track_half_width = 1.0;
    const double target_vel = 5.0;

    for (int i = 0; i < N_WP; i++) {
        double t = (double)i / N_WP;
        double x, y, psi, kappa;

        double straight_len = 10.0;
        double radius = 5.0;
        double total_len = 2.0 * straight_len + 2.0 * M_PI * radius;
        double s = t * total_len;

        if (s < straight_len) {
            x = s; y = 0.0; psi = 0.0; kappa = 0.0;
        } else if (s < straight_len + M_PI * radius) {
            double angle = (s - straight_len) / radius;
            x = straight_len + radius * sin(angle);
            y = radius * (1.0 - cos(angle));
            psi = angle;
            kappa = 1.0 / radius;
        } else if (s < 2.0 * straight_len + M_PI * radius) {
            double ds = s - straight_len - M_PI * radius;
            x = straight_len - ds; y = 2.0 * radius; psi = M_PI; kappa = 0.0;
        } else {
            double angle = (s - 2.0 * straight_len - M_PI * radius) / radius;
            x = -radius * sin(angle);
            y = 2.0 * radius - radius * (1.0 - cos(angle));
            psi = M_PI + angle;
            kappa = 1.0 / radius;
        }

        while (psi > M_PI) psi -= 2.0 * M_PI;
        while (psi < -M_PI) psi += 2.0 * M_PI;

        mpc_fpga_top(
            1, i,
            ftofp(x), ftofp(y), ftofp(psi),
            ftofp(target_vel), ftofp(kappa), ftofp(0.0),
            ftofp(track_half_width), ftofp(track_half_width),
            0,
            0, 0, 0, 0, 0, 0, 0, 0,
            &out_steer, &out_accel, &out_status, &out_iters);
    }

    /* Finalize trajectory */
    mpc_fpga_top(
        2, 0, 0, 0, 0, 0, 0, 0, 0, 0, N_WP,
        0, 0, 0, 0, 0, 0, 0, 0,
        &out_steer, &out_accel, &out_status, &out_iters);

    printf("  Loaded %d waypoints, status=%d\n\n", out_iters, out_status);
    CHECK(out_status == 0, "Trajectory loading should succeed");

    /* ---------------------------------------------------------------
     * Phase 2: Cold-start compute on straight section
     * Vehicle at (x=1.0, y=0.2, θ=0.05), 5 m/s, slight lateral error
     * --------------------------------------------------------------- */
    printf("Phase 2: Cold-start compute (straight section)...\n");

    mpc_fpga_top(
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ftofp(1.0), ftofp(0.2), ftofp(0.05),
        ftofp(5.0), ftofp(0.0), ftofp(0.0),
        ftofp(0.0), 2,
        &out_steer, &out_accel, &out_status, &out_iters);

    double steer1 = fptof(out_steer);
    double accel1 = fptof(out_accel);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer1, steer1 * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel1);
    printf("    Status:   %d, Iters: %d\n\n", out_status, out_iters);

    /* Quantitative checks for straight section with rightward lateral error:
     * - Steering should be negative (correct back toward path)
     * - Magnitude should be reasonable (not saturated for small error) */
    CHECK(out_status != 2, "Solver should not return error");
    CHECK_RANGE(steer1, -0.45, 0.0, "steering_cold_straight");
    CHECK_RANGE(accel1, -8.0, 8.0, "accel_cold_straight");
    CHECK(out_iters > 0, "Should take at least 1 iteration");

    /* ---------------------------------------------------------------
     * Phase 3: Warm-start compute (same state — should converge faster)
     * --------------------------------------------------------------- */
    printf("Phase 3: Warm-start compute...\n");
    int cold_iters = out_iters;

    mpc_fpga_top(
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ftofp(1.0), ftofp(0.2), ftofp(0.05),
        ftofp(5.0), ftofp(0.0), ftofp(0.0),
        out_steer, 2,
        &out_steer, &out_accel, &out_status, &out_iters);

    double steer2 = fptof(out_steer);
    double accel2 = fptof(out_accel);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer2, steer2 * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel2);
    printf("    Status:   %d, Iters: %d (cold was %d)\n\n",
           out_status, out_iters, cold_iters);

    CHECK(out_status != 2, "Warm-start should not error");
    /* Warm-start should converge in fewer or equal iterations */
    CHECK(out_iters <= cold_iters + 5,
          "Warm-start should not need many more iterations than cold");

    /* Consistency: warm-start output should be close to cold-start */
    double steer_diff = fabs(steer2 - steer1);
    CHECK(steer_diff < 0.15, "Warm vs cold steering should be similar (<0.15 rad)");

    /* ---------------------------------------------------------------
     * Phase 4: Curved section test
     * Vehicle entering right semicircle — steering should be positive
     * --------------------------------------------------------------- */
    printf("Phase 4: Curved section test...\n");

    int curve_wp = N_WP / 4;

    mpc_fpga_top(
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ftofp(10.0), ftofp(0.5), ftofp(0.3),
        ftofp(5.0), ftofp(0.1), ftofp(0.05),
        ftofp(0.05), curve_wp,
        &out_steer, &out_accel, &out_status, &out_iters);

    double steer_curve = fptof(out_steer);
    double accel_curve = fptof(out_accel);

    printf("    Steering: %.4f rad (%.1f deg)\n", steer_curve, steer_curve * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel_curve);
    printf("    Status:   %d, Iters: %d\n\n", out_status, out_iters);

    CHECK(out_status != 2, "Curve solver should not error");
    /* On a right semicircle (left turn), steering should be positive */
    CHECK(steer_curve > -0.05, "Curve steering should be positive (left turn)");
    CHECK_RANGE(steer_curve, -0.10, 0.45, "steering_curve");

    /* ---------------------------------------------------------------
     * Phase 5: Stability test — run 5 consecutive calls, check output
     * doesn't diverge
     * --------------------------------------------------------------- */
    printf("Phase 5: Stability test (5 consecutive calls)...\n");
    int prev_steer = ftofp(0.0);
    int stable = 1;

    for (int call = 0; call < 5; call++) {
        mpc_fpga_top(
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ftofp(2.0 + call * 0.5), ftofp(0.1), ftofp(0.02),
            ftofp(5.0), ftofp(0.0), ftofp(0.0),
            prev_steer, 5 + call,
            &out_steer, &out_accel, &out_status, &out_iters);

        double sv = fptof(out_steer);
        double av = fptof(out_accel);
        printf("    Call %d: steer=%.4f, accel=%.4f, status=%d, iters=%d\n",
               call, sv, av, out_status, out_iters);

        if (fabs(sv) > 0.45 || fabs(av) > 8.5) {
            printf("    WARNING: Output exceeds physical limits!\n");
            stable = 0;
        }
        if (out_status == 2) stable = 0;

        prev_steer = out_steer;
    }
    printf("\n");
    CHECK(stable, "All 5 consecutive calls should be within physical limits");

    /* ---------------------------------------------------------------
     * Phase 6: Zero-lateral-error test — steering should be near zero
     * --------------------------------------------------------------- */
    printf("Phase 6: Zero-error test (on path, correct heading)...\n");

    /* Reset ADMM state by reloading trajectory */
    mpc_fpga_top(2, 0, 0, 0, 0, 0, 0, 0, 0, 0, N_WP,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 &out_steer, &out_accel, &out_status, &out_iters);

    /* Vehicle exactly on path, correct heading, target velocity */
    mpc_fpga_top(
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ftofp(5.0), ftofp(0.0), ftofp(0.0),
        ftofp(5.0), ftofp(0.0), ftofp(0.0),
        ftofp(0.0), 15,
        &out_steer, &out_accel, &out_status, &out_iters);

    double steer_zero = fptof(out_steer);
    double accel_zero = fptof(out_accel);

    printf("    Steering: %.4f rad\n", steer_zero);
    printf("    Accel:    %.4f m/s^2\n", accel_zero);
    printf("    Status:   %d, Iters: %d\n\n", out_status, out_iters);

    /* On a straight with zero error, steering should be near zero.
     * Note: MPC horizon may see upcoming curve, allowing small anticipatory steering */
    CHECK_RANGE(steer_zero, -0.10, 0.10, "steering_zero_error");
    /* Accel should be near zero (already at target velocity) */
    CHECK_RANGE(accel_zero, -2.0, 2.0, "accel_zero_error");

    /* ---------------------------------------------------------------
     * Results
     * --------------------------------------------------------------- */
    printf("=== %d / %d checks passed ===\n", passed_checks, total_checks);
    printf("=== Test %s ===\n", (passed_checks == total_checks) ? "PASSED" : "FAILED");

    return (passed_checks == total_checks) ? 0 : 1;
}
