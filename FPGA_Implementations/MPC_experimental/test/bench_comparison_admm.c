/**
 * @file bench_comparison_admm.c
 * @brief ADMM-specific MPC benchmark (linked against mpc_admm_core)
 *
 * This binary is compiled with -DMPC_SOLVER_ADMM, so
 * mpc_compute_optimal_control() uses the ADMM QP solver internally.
 * Same scenarios as bench_comparison.c for apples-to-apples comparison.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "mpc.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"

#define BENCH_ITERATIONS 1000

static double get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

static void build_straight_ref(TrajectoryReferencePoint_t *ref, int n, fixed_point_t vel)
{
    memset(ref, 0, n * sizeof(ref[0]));
    for (int i = 0; i < n; i++) {
        ref[i].reference_velocity_meters_per_second = vel;
        ref[i].left_wall_bound_meters  = FP_CONST(1.5);
        ref[i].right_wall_bound_meters = FP_CONST(1.5);
    }
}

static void build_curve_ref(TrajectoryReferencePoint_t *ref, int n,
                             fixed_point_t vel, fixed_point_t kappa)
{
    build_straight_ref(ref, n, vel);
    for (int i = 0; i < n; i++) {
        ref[i].path_curvature_radians_per_meter = kappa;
    }
}

typedef struct {
    double avg_us, min_us, max_us;
    int    avg_iters;
    double steer_rad, accel_mps2;
} Result_t;

static Result_t bench_admm(const FrenetState_t *state,
                            const TrajectoryReferencePoint_t *ref,
                            int cold_start)
{
    Result_t r = {0};
    r.min_us = 1e9;
    MpcSolverResult_t result;
    int total_iters = 0;

    if (cold_start) {
        for (int i = 0; i < BENCH_ITERATIONS; i++) {
            mpc_reset();
            double t0 = get_time_us();
            mpc_compute_optimal_control(state, ref, &result);
            double t1 = get_time_us();
            double dt = t1 - t0;
            r.avg_us += dt;
            if (dt < r.min_us) r.min_us = dt;
            if (dt > r.max_us) r.max_us = dt;
            total_iters += result.iterations_used;
        }
    } else {
        for (int i = 0; i < 10; i++)
            mpc_compute_optimal_control(state, ref, &result);

        for (int i = 0; i < BENCH_ITERATIONS; i++) {
            double t0 = get_time_us();
            mpc_compute_optimal_control(state, ref, &result);
            double t1 = get_time_us();
            double dt = t1 - t0;
            r.avg_us += dt;
            if (dt < r.min_us) r.min_us = dt;
            if (dt > r.max_us) r.max_us = dt;
            total_iters += result.iterations_used;
        }
    }

    r.avg_us /= BENCH_ITERATIONS;
    r.avg_iters = total_iters / BENCH_ITERATIONS;
    r.steer_rad = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    r.accel_mps2 = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);
    return r;
}

static void print_result(const char *name, const Result_t *r)
{
    printf("  %-28s  %10.1f  %10.1f  %10.1f  %6d  %8.4f  %8.4f\n",
           name, r->avg_us, r->min_us, r->max_us,
           r->avg_iters, r->steer_rad, r->accel_mps2);
}

int main(void)
{
    printf("================================================================\n");
    printf("  Dense ADMM MPC Benchmark\n");
    printf("  (Drop-in ADMM QP solver via MPC_SOLVER_ADMM)\n");
    printf("  %d iterations per scenario\n", BENCH_ITERATIONS);
    printf("================================================================\n");

    vehicle_model_initialize();
    mpc_initialize();

    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 20;
    cfg.time_step_seconds = FP_CONST(0.05);
    mpc_set_configuration(&cfg);

    TrajectoryReferencePoint_t ref[50];
    FrenetState_t state;

    printf("\n%-30s  %10s  %10s  %10s  %6s  %8s  %8s\n",
           "Scenario", "Avg(us)", "Min(us)", "Max(us)", "Iters", "Steer", "Accel");
    printf("%-30s  %10s  %10s  %10s  %6s  %8s  %8s\n",
           "------------------------------", "----------", "----------",
           "----------", "------", "--------", "--------");

    /* Scenario 1: Straight warm */
    {
        memset(&state, 0, sizeof(state));
        state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);
        build_straight_ref(ref, 50, FP_CONST(3.0));
        Result_t r = bench_admm(&state, ref, 0);
        print_result("Straight (warm)", &r);
    }

    /* Scenario 2: Off-path warm */
    {
        memset(&state, 0, sizeof(state));
        state.lateral_error_meters = FP_CONST(0.3);
        state.heading_error_radians = FP_CONST(0.1);
        state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);
        build_straight_ref(ref, 50, FP_CONST(3.0));
        mpc_reset();
        Result_t r = bench_admm(&state, ref, 0);
        print_result("Off-path (warm)", &r);
    }

    /* Scenario 3: Curve κ=2.0 warm */
    {
        memset(&state, 0, sizeof(state));
        state.longitudinal_velocity_meters_per_second = FP_CONST(4.0);
        build_curve_ref(ref, 50, FP_CONST(4.0), FP_CONST(2.0));
        mpc_reset();
        Result_t r = bench_admm(&state, ref, 0);
        print_result("Curve k=2.0 (warm)", &r);
    }

    /* Scenario 4: Cold start curve + off-path */
    {
        memset(&state, 0, sizeof(state));
        state.lateral_error_meters = FP_CONST(0.5);
        state.heading_error_radians = FP_CONST(0.2);
        state.longitudinal_velocity_meters_per_second = FP_CONST(5.0);
        build_curve_ref(ref, 50, FP_CONST(5.0), FP_CONST(1.5));
        Result_t r = bench_admm(&state, ref, 1);
        print_result("Cold start (curve+off)", &r);
    }

    /* Scenario 5: Tight walls warm */
    {
        memset(&state, 0, sizeof(state));
        state.lateral_error_meters = FP_CONST(0.15);
        state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);
        build_straight_ref(ref, 50, FP_CONST(3.0));
        for (int i = 0; i < 50; i++) {
            ref[i].left_wall_bound_meters = FP_CONST(0.3);
            ref[i].right_wall_bound_meters = FP_CONST(0.3);
        }
        mpc_reset();
        Result_t r = bench_admm(&state, ref, 0);
        print_result("Tight walls (warm)", &r);
    }

    printf("\n================================================================\n");
    return 0;
}
