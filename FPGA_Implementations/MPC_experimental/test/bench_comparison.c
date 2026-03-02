/**
 * @file bench_comparison.c
 * @brief Side-by-side benchmark of all three MPC solvers
 *
 * Compares:
 *   1. Projected Gradient (original, via mpc_compute_optimal_control)
 *   2. Dense ADMM (via mpc_compute_optimal_control with MPC_SOLVER_ADMM)
 *   3. Riccati-ADMM (via mpc_riccati_compute_optimal_control)
 *
 * Since we can't link all three into one binary (ADMM needs a compile flag),
 * this binary tests the ORIGINAL projected gradient and Riccati side-by-side.
 * A separate bench_comparison_admm binary tests ADMM.
 *
 * Both binaries print results in the same format for easy comparison.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "mpc.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"

/* Riccati MPC API (from mpc_riccati.c) */
extern void mpc_riccati_initialize(void);
extern void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg);
extern void mpc_riccati_reset(void);
extern MpcSolverStatus_t mpc_riccati_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);

#define BENCH_ITERATIONS 1000

static double get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

/*===========================================================================*/

typedef struct {
    const char *name;
    double avg_us;
    double min_us;
    double max_us;
    int    avg_iters;
    double steer_rad;
    double accel_mps2;
} BenchResult_t;

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

/*===========================================================================
 * Run benchmark with projected gradient solver
 *===========================================================================*/

static BenchResult_t bench_projected_gradient(
    const FrenetState_t *state,
    const TrajectoryReferencePoint_t *ref,
    const char *scenario_name,
    int cold_start)
{
    BenchResult_t r = {0};
    r.name = scenario_name;
    r.min_us = 1e9;

    MpcSolverResult_t result;
    int total_iters = 0;

    if (cold_start) {
        /* Each iteration resets warm-start */
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
        /* Warm-start: run 10 warm-up calls first */
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

/*===========================================================================
 * Run benchmark with Riccati-ADMM solver
 *===========================================================================*/

static BenchResult_t bench_riccati(
    const FrenetState_t *state,
    const TrajectoryReferencePoint_t *ref,
    const char *scenario_name,
    int cold_start)
{
    BenchResult_t r = {0};
    r.name = scenario_name;
    r.min_us = 1e9;

    MpcSolverResult_t result;
    int total_iters = 0;

    if (cold_start) {
        for (int i = 0; i < BENCH_ITERATIONS; i++) {
            mpc_riccati_reset();
            double t0 = get_time_us();
            mpc_riccati_compute_optimal_control(state, ref, &result);
            double t1 = get_time_us();
            double dt = t1 - t0;
            r.avg_us += dt;
            if (dt < r.min_us) r.min_us = dt;
            if (dt > r.max_us) r.max_us = dt;
            total_iters += result.iterations_used;
        }
    } else {
        /* Warm-start: run 10 warm-up calls first */
        for (int i = 0; i < 10; i++)
            mpc_riccati_compute_optimal_control(state, ref, &result);

        for (int i = 0; i < BENCH_ITERATIONS; i++) {
            double t0 = get_time_us();
            mpc_riccati_compute_optimal_control(state, ref, &result);
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

/*===========================================================================*/

static void print_header(const char *title)
{
    printf("\n%-30s  %10s  %10s  %10s  %6s  %8s  %8s\n",
           title, "Avg(us)", "Min(us)", "Max(us)", "Iters", "Steer", "Accel");
    printf("%-30s  %10s  %10s  %10s  %6s  %8s  %8s\n",
           "------------------------------", "----------", "----------",
           "----------", "------", "--------", "--------");
}

static void print_result(const char *solver, const BenchResult_t *r)
{
    printf("  %-28s  %10.1f  %10.1f  %10.1f  %6d  %8.4f  %8.4f\n",
           solver, r->avg_us, r->min_us, r->max_us,
           r->avg_iters, r->steer_rad, r->accel_mps2);
}

int main(void)
{
    printf("================================================================\n");
    printf("  MPC Solver Comparison Benchmark\n");
    printf("  Projected Gradient vs Riccati-ADMM\n");
    printf("  %d iterations per scenario\n", BENCH_ITERATIONS);
    printf("================================================================\n");

    vehicle_model_initialize();

    /* Initialize both solvers with same config */
    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 20;
    cfg.time_step_seconds = FP_CONST(0.05);

    mpc_initialize();
    mpc_set_configuration(&cfg);
    mpc_riccati_initialize_with_configuration(&cfg);

    TrajectoryReferencePoint_t ref[50];
    FrenetState_t state;

    /* ---- Scenario 1: Straight, warm ---- */
    {
        print_header("Scenario 1: Straight (warm)");
        memset(&state, 0, sizeof(state));
        state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);
        build_straight_ref(ref, 50, FP_CONST(3.0));

        BenchResult_t pg = bench_projected_gradient(&state, ref, "Straight warm", 0);
        mpc_riccati_reset();
        BenchResult_t ri = bench_riccati(&state, ref, "Straight warm", 0);

        print_result("Projected Gradient", &pg);
        print_result("Riccati-ADMM", &ri);
    }

    /* ---- Scenario 2: Off-path correction, warm ---- */
    {
        print_header("Scenario 2: Off-path (warm)");
        memset(&state, 0, sizeof(state));
        state.lateral_error_meters = FP_CONST(0.3);
        state.heading_error_radians = FP_CONST(0.1);
        state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);
        build_straight_ref(ref, 50, FP_CONST(3.0));

        mpc_reset();
        BenchResult_t pg = bench_projected_gradient(&state, ref, "Off-path warm", 0);
        mpc_riccati_reset();
        BenchResult_t ri = bench_riccati(&state, ref, "Off-path warm", 0);

        print_result("Projected Gradient", &pg);
        print_result("Riccati-ADMM", &ri);
    }

    /* ---- Scenario 3: Curve κ=2.0, warm ---- */
    {
        print_header("Scenario 3: Curve k=2.0 (warm)");
        memset(&state, 0, sizeof(state));
        state.longitudinal_velocity_meters_per_second = FP_CONST(4.0);
        build_curve_ref(ref, 50, FP_CONST(4.0), FP_CONST(2.0));

        mpc_reset();
        BenchResult_t pg = bench_projected_gradient(&state, ref, "Curve warm", 0);
        mpc_riccati_reset();
        BenchResult_t ri = bench_riccati(&state, ref, "Curve warm", 0);

        print_result("Projected Gradient", &pg);
        print_result("Riccati-ADMM", &ri);
    }

    /* ---- Scenario 4: Cold start curve + off-path ---- */
    {
        print_header("Scenario 4: Cold start (curve+off)");
        memset(&state, 0, sizeof(state));
        state.lateral_error_meters = FP_CONST(0.5);
        state.heading_error_radians = FP_CONST(0.2);
        state.longitudinal_velocity_meters_per_second = FP_CONST(5.0);
        build_curve_ref(ref, 50, FP_CONST(5.0), FP_CONST(1.5));

        BenchResult_t pg = bench_projected_gradient(&state, ref, "Cold curve+off", 1);
        BenchResult_t ri = bench_riccati(&state, ref, "Cold curve+off", 1);

        print_result("Projected Gradient", &pg);
        print_result("Riccati-ADMM", &ri);
    }

    /* ---- Scenario 5: Tight walls ---- */
    {
        print_header("Scenario 5: Tight walls (warm)");
        memset(&state, 0, sizeof(state));
        state.lateral_error_meters = FP_CONST(0.15);
        state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);
        build_straight_ref(ref, 50, FP_CONST(3.0));
        for (int i = 0; i < 50; i++) {
            ref[i].left_wall_bound_meters  = FP_CONST(0.3);
            ref[i].right_wall_bound_meters = FP_CONST(0.3);
        }

        mpc_reset();
        BenchResult_t pg = bench_projected_gradient(&state, ref, "Tight walls warm", 0);
        mpc_riccati_reset();
        BenchResult_t ri = bench_riccati(&state, ref, "Tight walls warm", 0);

        print_result("Projected Gradient", &pg);
        print_result("Riccati-ADMM", &ri);
    }

    printf("\n================================================================\n");
    return 0;
}
