/**
 * @file test_cosim_minimal.c
 * @brief Minimal RTL Co-Simulation Testbench
 *
 * A lightweight testbench for RTL co-simulation.
 * Exercises the basic load→finalize→compute flow with minimal calls,
 * since RTL co-sim is much slower than C simulation.
 *
 * Return 0 = pass, non-zero = fail.
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "../include/mpc_fpga_interface.h"

#define FP_FRAC_BITS 16
#define FP_ONE       (1 << FP_FRAC_BITS)
#define DOUBLE_TO_FP(x) ((int32_t)((x) * FP_ONE))
#define FP_TO_DOUBLE(x) ((double)(x) / (double)FP_ONE)

/* Helper: call mpc_fpga for waypoint loading */
static void load_wp(uint32_t idx, double vel, double kappa,
                    double lwall, double rwall)
{
    int32_t ds, da, dc;
    uint32_t du1, du2, du3, du4;
    double ws = vel / 0.051;

    mpc_fpga(1, idx,
             DOUBLE_TO_FP(0.0), DOUBLE_TO_FP(0.0),
             DOUBLE_TO_FP(vel), DOUBLE_TO_FP(0.0),
             DOUBLE_TO_FP(vel * kappa), DOUBLE_TO_FP(ws),
             DOUBLE_TO_FP(kappa), DOUBLE_TO_FP(lwall), DOUBLE_TO_FP(rwall),
             0,
             0, 0, 0, 0, 0, 0, 0,
             &ds, &da, &du1, &du2, &dc, &du3, &du4);
}

int main(void)
{
    int32_t steer, accel, cost;
    uint32_t status, iters, traj_loaded, traj_size;
    int errors = 0;

    printf("=== MPC FPGA Minimal Co-Sim Testbench ===\n");

    /*--- Test 1: Load straight trajectory ---*/
    printf("Loading 30 straight waypoints (v=3.0 m/s)...\n");
    for (uint32_t i = 0; i < 30; i++) {
        load_wp(i, 3.0, 0.0, 2.0, 2.0);
    }

    /* Finalize */
    mpc_fpga(2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30,
             0, 0, 0, 0, 0, 0, 0,
             &steer, &accel, &status, &iters, &cost,
             &traj_loaded, &traj_size);

    if (traj_loaded != 1 || traj_size != 30) {
        printf("[FAIL] Trajectory not loaded correctly: loaded=%u size=%u\n",
               traj_loaded, traj_size);
        errors++;
    } else {
        printf("[PASS] Trajectory loaded: %u waypoints\n", traj_size);
    }

    /*--- Test 2: Compute on-path ---*/
    printf("\nComputing MPC control (on path, v=3.0 m/s)...\n");
    mpc_fpga(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             DOUBLE_TO_FP(0.0),    /* lat error */
             DOUBLE_TO_FP(0.0),    /* heading error */
             DOUBLE_TO_FP(3.0),    /* vx */
             DOUBLE_TO_FP(0.0),    /* vy */
             DOUBLE_TO_FP(0.0),    /* omega */
             DOUBLE_TO_FP(3.0/0.051), /* wheel speed */
             0,                     /* wp index */
             &steer, &accel, &status, &iters, &cost,
             &traj_loaded, &traj_size);

    printf("  Status=%u, Iterations=%u\n", status, iters);
    printf("  Steering=%.4f rad, Acceleration=%.4f m/s²\n",
           FP_TO_DOUBLE(steer), FP_TO_DOUBLE(accel));

    if (status > 1) {
        printf("[FAIL] Solver failed with status %u\n", status);
        errors++;
    } else {
        printf("[PASS] Solver converged\n");
    }

    if (fabs(FP_TO_DOUBLE(steer)) > 0.1) {
        printf("[FAIL] Steering too large on straight: %.4f\n",
               FP_TO_DOUBLE(steer));
        errors++;
    } else {
        printf("[PASS] Steering near zero on straight\n");
    }

    /*--- Test 3: Compute with lateral offset ---*/
    printf("\nComputing MPC control (0.3m lateral offset)...\n");
    mpc_fpga(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             DOUBLE_TO_FP(0.3),    /* lat error = 0.3m left */
             DOUBLE_TO_FP(0.0),
             DOUBLE_TO_FP(3.0),
             DOUBLE_TO_FP(0.0),
             DOUBLE_TO_FP(0.0),
             DOUBLE_TO_FP(3.0/0.051),
             5,
             &steer, &accel, &status, &iters, &cost,
             &traj_loaded, &traj_size);

    printf("  Steering=%.4f rad (should be negative to correct)\n",
           FP_TO_DOUBLE(steer));

    if (FP_TO_DOUBLE(steer) >= 0) {
        printf("[FAIL] Should steer right (negative) to correct left offset\n");
        errors++;
    } else {
        printf("[PASS] Correct steering direction\n");
    }

    /*--- Summary ---*/
    printf("\n=== Results: %d errors ===\n", errors);
    return errors;
}
