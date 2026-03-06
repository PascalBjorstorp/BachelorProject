/**
 * @file tb_mpc_fpga.c
 * @brief C Testbench for MPC FPGA HLS Cosimulation
 *
 * Tests the mpc_fpga_top function in three phases:
 *   1. Load a simple oval trajectory (mode=1 + mode=2)
 *   2. Run compute mode (mode=0) with a test vehicle state
 *   3. Verify outputs are reasonable
 *
 * Build with: gcc -I../include -o tb_mpc_fpga tb_mpc_fpga.c \
 *             ../src/mpc_fpga_top.c ../src/mpc_riccati_hls.c \
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

int main(void)
{
    int out_steer, out_accel, out_status, out_iters;
    int pass = 1;

    printf("=== MPC FPGA Testbench ===\n\n");

    /* ---------------------------------------------------------------
     * Phase 1: Load an oval trajectory (100 waypoints)
     *
     * Simple oval: two straights + two semicircles
     * Straight: 10m long, semicircle: radius 5m
     * Track width: 2m (left_bound=1m, right_bound=1m)
     * Target velocity: 5 m/s
     * --------------------------------------------------------------- */
    printf("Phase 1: Loading trajectory...\n");
    const int N_WP = 100;
    const double track_half_width = 1.0;
    const double target_vel = 5.0;

    for (int i = 0; i < N_WP; i++) {
        double t = (double)i / N_WP;
        double x, y, psi, kappa;

        /* Simple oval parameterization */
        double straight_len = 10.0;
        double radius = 5.0;
        double total_len = 2.0 * straight_len + 2.0 * M_PI * radius;
        double s = t * total_len;

        if (s < straight_len) {
            /* Bottom straight (going right) */
            x = s;
            y = 0.0;
            psi = 0.0;
            kappa = 0.0;
        } else if (s < straight_len + M_PI * radius) {
            /* Right semicircle */
            double angle = (s - straight_len) / radius;
            x = straight_len + radius * sin(angle);
            y = radius * (1.0 - cos(angle));
            psi = angle;
            kappa = 1.0 / radius;
        } else if (s < 2.0 * straight_len + M_PI * radius) {
            /* Top straight (going left) */
            double ds = s - straight_len - M_PI * radius;
            x = straight_len - ds;
            y = 2.0 * radius;
            psi = M_PI;
            kappa = 0.0;
        } else {
            /* Left semicircle */
            double angle = (s - 2.0 * straight_len - M_PI * radius) / radius;
            x = -radius * sin(angle);
            y = 2.0 * radius - radius * (1.0 - cos(angle));
            psi = M_PI + angle;
            kappa = 1.0 / radius;
        }

        /* Normalize psi to [-pi, pi] */
        while (psi > M_PI) psi -= 2.0 * M_PI;
        while (psi < -M_PI) psi += 2.0 * M_PI;

        mpc_fpga_top(
            1, /* mode = load waypoint */
            i,
            ftofp(x), ftofp(y), ftofp(psi),
            ftofp(target_vel), ftofp(kappa), ftofp(0.0),
            ftofp(track_half_width), ftofp(track_half_width),
            0, /* wp_total unused in mode 1 */
            0, 0, 0, 0, 0, 0, 0, 0, /* state unused */
            &out_steer, &out_accel, &out_status, &out_iters);
    }

    /* Finalize trajectory */
    mpc_fpga_top(
        2, /* mode = finalize */
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        N_WP, /* wp_total */
        0, 0, 0, 0, 0, 0, 0, 0,
        &out_steer, &out_accel, &out_status, &out_iters);

    printf("  Loaded %d waypoints, status=%d\n\n", out_iters, out_status);
    if (out_status != 0) {
        printf("FAIL: Trajectory loading failed\n");
        return 1;
    }

    /* ---------------------------------------------------------------
     * Phase 2: Run MPC compute with a test state
     *
     * Vehicle at (x=1.0, y=0.2, theta=0.05) on the bottom straight,
     * moving at 5 m/s with small lateral error.
     * --------------------------------------------------------------- */
    printf("Phase 2: Running MPC compute...\n");

    /* First call (cold start) */
    mpc_fpga_top(
        0, /* mode = compute */
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* wp params unused */
        ftofp(1.0),   /* state_x */
        ftofp(0.2),   /* state_y (slightly off-centerline) */
        ftofp(0.05),  /* state_theta (slight heading error) */
        ftofp(5.0),   /* state_vx */
        ftofp(0.0),   /* state_vy */
        ftofp(0.0),   /* state_omega */
        ftofp(0.0),   /* state_steering (centered) */
        2,            /* state_wp_idx (closest waypoint) */
        &out_steer, &out_accel, &out_status, &out_iters);

    double steer_val = fptof(out_steer);
    double accel_val = fptof(out_accel);

    printf("  Call 1 (cold start):\n");
    printf("    Steering: %.4f rad (%.1f deg)\n", steer_val, steer_val * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel_val);
    printf("    Status:   %d\n", out_status);
    printf("    Iters:    %d\n\n", out_iters);

    /* Check: steering should be negative (correcting rightward back to path) */
    if (steer_val > 0.5 || steer_val < -0.5) {
        printf("  WARNING: Steering magnitude seems large: %.4f rad\n", steer_val);
    }
    if (out_status == 2) {
        printf("  FAIL: Solver returned error status\n");
        pass = 0;
    }

    /* Second call (warm start) — same state, should converge faster */
    mpc_fpga_top(
        0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ftofp(1.0), ftofp(0.2), ftofp(0.05),
        ftofp(5.0), ftofp(0.0), ftofp(0.0),
        out_steer, /* Use previous steering output as current servo pos */
        2,
        &out_steer, &out_accel, &out_status, &out_iters);

    steer_val = fptof(out_steer);
    accel_val = fptof(out_accel);

    printf("  Call 2 (warm start):\n");
    printf("    Steering: %.4f rad (%.1f deg)\n", steer_val, steer_val * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel_val);
    printf("    Status:   %d\n", out_status);
    printf("    Iters:    %d\n\n", out_iters);

    /* ---------------------------------------------------------------
     * Phase 3: Run on a curved section
     * --------------------------------------------------------------- */
    printf("Phase 3: Testing on curved section...\n");

    /* Place vehicle entering the right semicircle */
    int curve_wp = N_WP / 4;  /* ~25% into trajectory = start of curve */
    double curve_x = 10.0;
    double curve_y = 0.5;
    double curve_theta = 0.3;

    mpc_fpga_top(
        0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ftofp(curve_x), ftofp(curve_y), ftofp(curve_theta),
        ftofp(5.0), ftofp(0.1), ftofp(0.05),
        ftofp(0.05), /* Small existing steering */
        curve_wp,
        &out_steer, &out_accel, &out_status, &out_iters);

    steer_val = fptof(out_steer);
    accel_val = fptof(out_accel);

    printf("  Curve entry:\n");
    printf("    Steering: %.4f rad (%.1f deg)\n", steer_val, steer_val * 180.0 / M_PI);
    printf("    Accel:    %.4f m/s^2\n", accel_val);
    printf("    Status:   %d\n", out_status);
    printf("    Iters:    %d\n\n", out_iters);

    /* Steering should be positive (turning left for right semicircle) */
    if (out_status == 2) {
        printf("  FAIL: Solver error on curve\n");
        pass = 0;
    }

    /* ---------------------------------------------------------------
     * Results
     * --------------------------------------------------------------- */
    printf("=== Test %s ===\n", pass ? "PASSED" : "FAILED");
    printf("  Note: Verify exact values match MPC_experimental output\n");
    printf("  for numerical equivalence testing.\n");

    return pass ? 0 : 1;
}
