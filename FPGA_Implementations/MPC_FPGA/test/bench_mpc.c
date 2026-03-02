/**
 * @file bench_mpc.c
 * @brief Benchmark for full MPC solve timing (linearization + QP construction + solve)
 */
#include <stdio.h>
#include <time.h>
#include "mpc.h"
#include "vehicle_model.h"
#include "fp_math.h"
#include "mpc_types.h"

static double timespec_diff_us(struct timespec *start, struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1e6 +
           (end->tv_nsec - start->tv_nsec) / 1e3;
}

int main(void)
{
    vehicle_model_initialize();
    mpc_initialize();

    /* Configure MPC */
    MpcConfiguration_t config = mpc_get_configuration();
    config.prediction_horizon_steps = 20;
    config.time_step_seconds = FP_CONST(0.05);
    mpc_set_configuration(&config);

    /* Straight-line reference trajectory */
    TrajectoryReferencePoint_t ref[50];
    for (int i = 0; i < 50; i++)
    {
        ref[i].reference_lateral_error_meters = 0;
        ref[i].reference_heading_error_radians = 0;
        ref[i].reference_velocity_meters_per_second = FP_CONST(5.0);
        ref[i].reference_lateral_velocity_meters_per_second = 0;
        ref[i].reference_yaw_rate_radians_per_second = 0;
        ref[i].path_curvature_radians_per_meter = 0;
        ref[i].left_wall_bound_meters = FP_CONST(1.5);
        ref[i].right_wall_bound_meters = FP_CONST(1.5);
    }

    /* Initial state */
    FrenetState_t state;
    state.lateral_error_meters = FP_CONST(0.1);
    state.heading_error_radians = FP_CONST(0.02);
    state.longitudinal_velocity_meters_per_second = FP_CONST(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    MpcSolverResult_t result;

    /* Warm-up */
    for (int i = 0; i < 10; i++)
    {
        mpc_compute_optimal_control(&state, ref, &result);
    }

    /* Benchmark: 1000 solves */
    const int N = 1000;
    struct timespec t0, t1;
    double total_us = 0.0;
    double min_us = 1e9, max_us = 0.0;

    for (int i = 0; i < N; i++)
    {
        /* Vary state slightly to avoid caching artefacts */
        state.lateral_error_meters = FP_CONST(0.05) + (i & 0xFF);
        state.heading_error_radians = FP_CONST(0.01) + ((i >> 2) & 0x7F);

        clock_gettime(CLOCK_MONOTONIC, &t0);
        mpc_compute_optimal_control(&state, ref, &result);
        clock_gettime(CLOCK_MONOTONIC, &t1);

        double dt = timespec_diff_us(&t0, &t1);
        total_us += dt;
        if (dt < min_us) min_us = dt;
        if (dt > max_us) max_us = dt;
    }

    printf("=== Full MPC Solve Benchmark ===\n\n");
    printf("Straight line (%d iterations):\n", N);
    printf("  Average: %.1f us\n", total_us / N);
    printf("  Min:     %.1f us\n", min_us);
    printf("  Max:     %.1f us\n", max_us);

    /* Corner scenario */
    for (int i = 0; i < 50; i++)
    {
        ref[i].path_curvature_radians_per_meter = FP_CONST(0.5);
    }
    state.lateral_error_meters = FP_CONST(0.3);
    state.heading_error_radians = FP_CONST(0.15);
    state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);

    /* Cold start (reset warm start) */
    mpc_reset();

    total_us = 0.0;
    min_us = 1e9;
    max_us = 0.0;

    for (int i = 0; i < N; i++)
    {
        clock_gettime(CLOCK_MONOTONIC, &t0);
        mpc_compute_optimal_control(&state, ref, &result);
        clock_gettime(CLOCK_MONOTONIC, &t1);

        double dt = timespec_diff_us(&t0, &t1);
        total_us += dt;
        if (dt < min_us) min_us = dt;
        if (dt > max_us) max_us = dt;
    }

    printf("\nCorner scenario (%d iterations):\n", N);
    printf("  Average: %.1f us\n", total_us / N);
    printf("  Min:     %.1f us\n", min_us);
    printf("  Max:     %.1f us\n", max_us);

    return 0;
}
