/**
 * @file bench_mpc_timing.c
 * @brief MPC timing benchmark — measures end-to-end compute time
 *
 * Simulates realistic driving scenarios (straight, curve, off-path)
 * and measures wall-clock time per mpc_compute_optimal_control() call.
 */

#include <stdio.h>
#include <time.h>
#include <string.h>
#include "mpc.h"
#include "fp_math.h"

#define BENCH_ITERATIONS 1000

static double get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

static void build_straight_trajectory(TrajectoryReferencePoint_t *ref, int n,
                                       fixed_point_t velocity)
{
    for (int i = 0; i < n; i++) {
        memset(&ref[i], 0, sizeof(ref[i]));
        ref[i].reference_velocity_meters_per_second = velocity;
            fp_div(velocity, DOUBLE_TO_FP(0.05));
        ref[i].left_wall_bound_meters  = DOUBLE_TO_FP(1.5);
        ref[i].right_wall_bound_meters = DOUBLE_TO_FP(1.5);
    }
}

static void build_curve_trajectory(TrajectoryReferencePoint_t *ref, int n,
                                    fixed_point_t velocity, fixed_point_t kappa)
{
    build_straight_trajectory(ref, n, velocity);
    for (int i = 0; i < n; i++) {
        ref[i].path_curvature_radians_per_meter = kappa;
    }
}

int main(void)
{
    printf("========================================\n");
    printf("MPC Timing Benchmark\n");
    printf("========================================\n\n");

    mpc_initialize();

    TrajectoryReferencePoint_t ref[50];
    MpcSolverResult_t result;

    /* Test 1: On-path straight (trivial, warm-started) */
    {
        FrenetState_t state = {0};
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(3.0);
        build_straight_trajectory(ref, 10, DOUBLE_TO_FP(3.0));

        /* Warm up */
        for (int i = 0; i < 10; i++)
            mpc_compute_optimal_control(&state, ref, &result);

        double t0 = get_time_us();
        for (int i = 0; i < BENCH_ITERATIONS; i++)
            mpc_compute_optimal_control(&state, ref, &result);
        double t1 = get_time_us();

        printf("Straight on-path (warm):  %.2f us/call  (iters=%d)\n",
               (t1 - t0) / BENCH_ITERATIONS, result.iterations_used);
    }

    /* Test 2: Off-path (needs correction) */
    {
        mpc_reset();
        FrenetState_t state = {0};
        state.lateral_error_meters = DOUBLE_TO_FP(0.3);
        state.heading_error_radians = DOUBLE_TO_FP(0.1);
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(3.0);
        build_straight_trajectory(ref, 10, DOUBLE_TO_FP(3.0));

        /* Warm up */
        for (int i = 0; i < 10; i++)
            mpc_compute_optimal_control(&state, ref, &result);

        double t0 = get_time_us();
        for (int i = 0; i < BENCH_ITERATIONS; i++)
            mpc_compute_optimal_control(&state, ref, &result);
        double t1 = get_time_us();

        printf("Straight off-path (warm): %.2f us/call  (iters=%d)\n",
               (t1 - t0) / BENCH_ITERATIONS, result.iterations_used);
    }

    /* Test 3: Curve (aggressive cornering) */
    {
        mpc_reset();
        FrenetState_t state = {0};
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(4.0);
        build_curve_trajectory(ref, 10, DOUBLE_TO_FP(4.0), DOUBLE_TO_FP(2.0));

        /* Warm up */
        for (int i = 0; i < 10; i++)
            mpc_compute_optimal_control(&state, ref, &result);

        double t0 = get_time_us();
        for (int i = 0; i < BENCH_ITERATIONS; i++)
            mpc_compute_optimal_control(&state, ref, &result);
        double t1 = get_time_us();

        printf("Curve kappa=2.0 (warm):   %.2f us/call  (iters=%d)\n",
               (t1 - t0) / BENCH_ITERATIONS, result.iterations_used);
    }

    /* Test 4: Cold start (no warm-start, needs most iterations) */
    {
        FrenetState_t state = {0};
        state.lateral_error_meters = DOUBLE_TO_FP(0.5);
        state.heading_error_radians = DOUBLE_TO_FP(0.2);
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
        build_curve_trajectory(ref, 10, DOUBLE_TO_FP(5.0), DOUBLE_TO_FP(1.5));

        double total = 0;
        int max_iters = 0;
        for (int i = 0; i < BENCH_ITERATIONS; i++) {
            mpc_reset();  /* Force cold start each time */
            double t0 = get_time_us();
            mpc_compute_optimal_control(&state, ref, &result);
            double t1 = get_time_us();
            total += (t1 - t0);
            if (result.iterations_used > max_iters)
                max_iters = result.iterations_used;
        }

        printf("Cold start (curve+off):   %.2f us/call  (max iters=%d)\n",
               total / BENCH_ITERATIONS, max_iters);
    }

    printf("\n========================================\n");
    return 0;
}
