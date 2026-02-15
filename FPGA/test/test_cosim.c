/**
 * @file test_cosim.c
 * @brief RTL Co-Simulation testbench for Pure Pursuit
 *
 * All calls use only scalar arguments for co-sim compatibility.
 *
 * Compile: gcc -I./include -o build/test_cosim \
 *              src/pure_pursuit_fpga.c src/fp_math_hls.c \
 *              test/test_cosim.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "fpga_interface.h"
#include "fp_math_hls.h"

/* Top-level function */
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

/* Helper: call compute mode with common params */
static void call_compute(
    int32_t x, int32_t y, int32_t theta, int32_t vel, uint32_t wp_idx,
    int32_t* steering, int32_t* velocity, int32_t* cte,
    int32_t* heading_err, int32_t* lookahead, uint32_t* target_wp,
    uint32_t* status, uint32_t* traj_loaded, uint32_t* traj_size
)
{
    pure_pursuit_fpga(
        0,  /* mode=compute */
        0, 0, 0, 0, 0, 0, 0,   /* waypoint args (unused) */
        x, y, theta, vel, wp_idx,
        DOUBLE_TO_FP(1.5),     /* min_lookahead */
        DOUBLE_TO_FP(3.0),     /* max_lookahead */
        DOUBLE_TO_FP(0.3),     /* lookahead_gain */
        DOUBLE_TO_FP(0.324),   /* wheelbase */
        DOUBLE_TO_FP(0.4189),  /* max_steering */
        DOUBLE_TO_FP(5.0),     /* max_velocity */
        10,                    /* lookahead_points */
        steering, velocity, cte, heading_err,
        lookahead, target_wp, status,
        traj_loaded, traj_size
    );
}

int main(void)
{
    int errors = 0;
    int32_t steering, velocity, cte, heading_err, lookahead;
    uint32_t target_wp, status, traj_loaded, traj_size;

    printf("========== RTL Co-Sim Test (v2) ==========\n\n");

    /* ---- Test 1: Load 20 waypoints (straight line along X) ---- */
    printf("Loading 20 waypoints...\n");
    for (int i = 0; i < 20; i++) {
        pure_pursuit_fpga(
            1,  /* mode=load_waypoint */
            i,  /* index */
            DOUBLE_TO_FP(i * 1.0),   /* x */
            0,                        /* y */
            0,                        /* theta */
            DOUBLE_TO_FP(3.0),       /* velocity */
            0,                        /* kappa */
            20,                       /* total (unused in mode 1) */
            0, 0, 0, 0, 0,           /* state (unused) */
            0, 0, 0, 0, 0, 0, 0,     /* params (unused) */
            &steering, &velocity, &cte, &heading_err,
            &lookahead, &target_wp, &status,
            &traj_loaded, &traj_size
        );
    }

    /* ---- Finalize trajectory ---- */
    pure_pursuit_fpga(
        2,  /* mode=finalize */
        0, 0, 0, 0, 0, 0,
        20, /* total waypoints */
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        &steering, &velocity, &cte, &heading_err,
        &lookahead, &target_wp, &status,
        &traj_loaded, &traj_size
    );

    printf("  traj_loaded=%u, traj_size=%u\n", traj_loaded, traj_size);
    if (traj_loaded != 1) { printf("FAIL: trajectory not loaded\n"); errors++; }
    if (traj_size != 20)  { printf("FAIL: wrong trajectory size\n"); errors++; }

    /* ---- Test 2: Compute on path (expect ~0 steering) ---- */
    printf("\nCompute: vehicle on path (x=2.0, y=0.0, theta=0)...\n");
    call_compute(
        DOUBLE_TO_FP(2.0), 0, 0, DOUBLE_TO_FP(2.0), 2,
        &steering, &velocity, &cte, &heading_err,
        &lookahead, &target_wp, &status, &traj_loaded, &traj_size
    );
    printf("  steering=%.4f rad, velocity=%.2f m/s, cte=%.4f, status=%u\n",
           FP_TO_DOUBLE(steering), FP_TO_DOUBLE(velocity),
           FP_TO_DOUBLE(cte), status);

    if (status != 0) { printf("FAIL: status not OK\n"); errors++; }
    if (fabs(FP_TO_DOUBLE(steering)) > 0.05) {
        printf("FAIL: steering should be ~0 on straight path (got %.4f)\n",
               FP_TO_DOUBLE(steering));
        errors++;
    }

    /* ---- Test 3: Compute off path (expect negative steering) ---- */
    printf("\nCompute: vehicle off path (x=2.0, y=0.5, theta=0)...\n");
    call_compute(
        DOUBLE_TO_FP(2.0), DOUBLE_TO_FP(0.5), 0, DOUBLE_TO_FP(2.0), 2,
        &steering, &velocity, &cte, &heading_err,
        &lookahead, &target_wp, &status, &traj_loaded, &traj_size
    );
    printf("  steering=%.4f rad, velocity=%.2f m/s, cte=%.4f\n",
           FP_TO_DOUBLE(steering), FP_TO_DOUBLE(velocity),
           FP_TO_DOUBLE(cte));

    if (steering >= 0) {
        printf("FAIL: steering should be negative to correct back to path\n");
        errors++;
    }

    /* ---- Test 4: Compute with higher speed (larger lookahead) ---- */
    printf("\nCompute: high speed (x=2.0, y=0.0, vel=5.0)...\n");
    call_compute(
        DOUBLE_TO_FP(2.0), 0, 0, DOUBLE_TO_FP(5.0), 2,
        &steering, &velocity, &cte, &heading_err,
        &lookahead, &target_wp, &status, &traj_loaded, &traj_size
    );
    printf("  steering=%.4f rad, lookahead=%.2f m, target_wp=%u\n",
           FP_TO_DOUBLE(steering), FP_TO_DOUBLE(lookahead), target_wp);

    /* Higher speed should produce larger lookahead */
    if (FP_TO_DOUBLE(lookahead) < 2.0) {
        printf("FAIL: lookahead should be > 2.0m at high speed\n");
        errors++;
    }

    /* ---- Test 5: No trajectory loaded ---- */
    /* (Can't easily test this since trajectory is already loaded
       and we can't unload in the refactored version) */

    /* ---- Summary ---- */
    printf("\n========================================\n");
    if (errors == 0) {
        printf("  All tests PASSED\n");
    } else {
        printf("  %d test(s) FAILED\n", errors);
    }
    printf("========================================\n");

    return errors;
}
