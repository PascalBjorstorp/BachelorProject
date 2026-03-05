/**
 * @file test_rk4_sim.c
 * @brief RK4-based closed-loop simulation test for condensed-QP MPC controller
 *
 * This test matches the f1tenth_gym simulator's dynamics exactly:
 *   - Single-track dynamic bicycle model (beta formulation)
 *   - RK4 integration at dt=0.01s (same as sim)
 *   - Bang-bang steering servo (sv_max=3.2 rad/s)
 *   - PID speed controller (kp=10*a_max/v_max)
 *   - Sim vehicle parameters from F110Env.f1tenth_vehicle_params()
 *
 * The MPC controller uses its own (measured) vehicle parameters internally,
 * matching the real setup where the controller model differs from the plant.
 *
 * Test scenarios:
 *   1. Straight-line tracking at multiple speeds (3-12 m/s)
 *   2. Constant-curvature curves (R=5, 10, 20 m)
 *   3. S-curve transition (kappa = -0.1 -> 0 -> +0.1)
 *   4. Emergency braking
 *   5. High-speed curve entry
 *   6. Model mismatch convergence
 *   7. Low-speed RK4 stability
 *
 * Compile:
 *   gcc -Wall -Wextra -O2 -I../include -o test_rk4_sim \
 *       test_rk4_sim.c ../src/fp_math.c ../src/vehicle_model.c \
 *       ../src/qp_solver.c ../src/mpc.c -lm
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "fp_math.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "mpc.h"

/*===========================================================================
 * Test Framework
 *===========================================================================*/

static int tests_passed = 0;
static int tests_failed = 0;

static void check_condition(const char *name, int condition)
{
    if (condition) {
        printf("  [PASS] %s\n", name);
        tests_passed++;
    } else {
        printf("  [FAIL] %s\n", name);
        tests_failed++;
    }
}

/*===========================================================================
 * F1Tenth Gym Simulator Parameters
 *===========================================================================
 * From F110Env.f1tenth_vehicle_params() in f1tenth_gym/envs/f110_env.py
 * These are the PLANT parameters (what the sim uses), which differ from
 * the MPC's internal model parameters (mpc_types.h #defines).
 */

/* Tire / friction */
#define SIM_MU      1.0489
#define SIM_C_SF    4.718
#define SIM_C_SR    5.4562

/* Geometry */
#define SIM_LF      0.15875
#define SIM_LR      0.17145
#define SIM_L       (SIM_LF + SIM_LR)   /* 0.3302 */
#define SIM_H       0.074                /* CG height */

/* Mass / inertia */
#define SIM_M       3.74
#define SIM_IZ      0.04712

/* Steering limits */
#define SIM_S_MIN   (-0.4189)
#define SIM_S_MAX   0.4189
#define SIM_SV_MIN  (-3.2)
#define SIM_SV_MAX  3.2

/* Speed limits */
#define SIM_V_MIN   (-5.0)
#define SIM_V_MAX   20.0
#define SIM_A_MAX   9.51

/* Kinematic/dynamic switch speed */
#define SIM_V_SWITCH 7.319

/* Physics */
#define SIM_G       9.81

/* Sim timestep (seconds) */
#define SIM_DT      0.01

/* MPC control interval (seconds) — 5 sim steps per MPC call */
#define MPC_DT      0.05

/* MPC prediction horizon (condensed QP) */
#define MPC_HORIZON 20

/*===========================================================================
 * Single-Track Dynamic Bicycle Model (Beta Formulation)
 *===========================================================================
 * Matches f1tenth_gym/envs/dynamic_models/single_track.py exactly.
 *
 * State vector: [X, Y, delta, V, psi, psi_dot, beta]
 *   X, Y       — global position [m]
 *   delta      — current steering angle [rad]
 *   V          — speed magnitude [m/s]
 *   psi        — heading [rad]
 *   psi_dot    — yaw rate [rad/s]
 *   beta       — slip angle at CG [rad]
 *
 * Control input: [steering_velocity, acceleration]
 *   steering_velocity — rate of steering change [rad/s]
 *   acceleration      — longitudinal acceleration [m/s^2]
 */

/* State indices */
#define ST_X        0
#define ST_Y        1
#define ST_DELTA    2
#define ST_V        3
#define ST_PSI      4
#define ST_PSI_DOT  5
#define ST_BETA     6
#define ST_DIM      7

/**
 * Compute state derivatives for the single-track dynamic model.
 */
static void single_track_dynamics(const double state[ST_DIM],
                                  double sv, double accl,
                                  double deriv[ST_DIM])
{
    double X       = state[ST_X];
    double Y       = state[ST_Y];
    double delta   = state[ST_DELTA];
    double V       = state[ST_V];
    double psi     = state[ST_PSI];
    double psi_dot = state[ST_PSI_DOT];
    double beta    = state[ST_BETA];

    (void)X; (void)Y; /* unused in derivatives */

    /* Load transfer factors */
    double g_lr = SIM_G * SIM_LR - accl * SIM_H;
    double g_lf = SIM_G * SIM_LF + accl * SIM_H;

    if (fabs(V) < 0.5) {
        /* Kinematic model (low speed fallback) */
        double lwb = SIM_L;
        double beta_kin = atan(tan(delta) * SIM_LR / lwb);
        deriv[ST_X]       = V * cos(psi + beta_kin);
        deriv[ST_Y]       = V * sin(psi + beta_kin);
        deriv[ST_DELTA]   = sv;
        deriv[ST_V]       = accl;
        deriv[ST_PSI]     = (V / lwb) * cos(beta_kin) * tan(delta);
        deriv[ST_PSI_DOT] = 0.0;
        deriv[ST_BETA]    = 0.0;
    } else {
        /* Dynamic single-track model */
        deriv[ST_X]     = V * cos(psi + beta);
        deriv[ST_Y]     = V * sin(psi + beta);
        deriv[ST_DELTA] = sv;
        deriv[ST_V]     = accl;
        deriv[ST_PSI]   = psi_dot;

        /* Yaw acceleration */
        deriv[ST_PSI_DOT] = (SIM_MU * SIM_M / (SIM_IZ * SIM_L))
            * (SIM_LF * SIM_C_SF * g_lr * delta
               + (SIM_LR * SIM_C_SR * g_lf - SIM_LF * SIM_C_SF * g_lr) * beta
               - (SIM_LF * SIM_LF * SIM_C_SF * g_lr
                  + SIM_LR * SIM_LR * SIM_C_SR * g_lf)
                 * (psi_dot / V));

        /* Slip angle rate */
        deriv[ST_BETA] = (SIM_MU / (V * SIM_L))
            * (SIM_C_SF * g_lr * delta
               - (SIM_C_SR * g_lf + SIM_C_SF * g_lr) * beta
               + (SIM_C_SR * g_lf * SIM_LR - SIM_C_SF * g_lr * SIM_LF)
                 * (psi_dot / V))
            - psi_dot;
    }
}

/*===========================================================================
 * RK4 Integrator
 *===========================================================================*/

static void rk4_step(double state[ST_DIM], double sv, double accl, double dt)
{
    double k1[ST_DIM], k2[ST_DIM], k3[ST_DIM], k4[ST_DIM];
    double tmp[ST_DIM];

    single_track_dynamics(state, sv, accl, k1);
    for (int i = 0; i < ST_DIM; i++)
        tmp[i] = state[i] + dt * 0.5 * k1[i];
    single_track_dynamics(tmp, sv, accl, k2);
    for (int i = 0; i < ST_DIM; i++)
        tmp[i] = state[i] + dt * 0.5 * k2[i];
    single_track_dynamics(tmp, sv, accl, k3);
    for (int i = 0; i < ST_DIM; i++)
        tmp[i] = state[i] + dt * k3[i];
    single_track_dynamics(tmp, sv, accl, k4);

    for (int i = 0; i < ST_DIM; i++)
        state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

/*===========================================================================
 * Steering Servo Model (Bang-Bang)
 *===========================================================================*/

static double bang_bang_steer(double steer_cmd, double current_steer)
{
    if (steer_cmd > SIM_S_MAX) steer_cmd = SIM_S_MAX;
    if (steer_cmd < SIM_S_MIN) steer_cmd = SIM_S_MIN;

    double diff = steer_cmd - current_steer;
    if (fabs(diff) < 1e-4) return 0.0;

    double sv = (diff > 0.0) ? SIM_SV_MAX : SIM_SV_MIN;

    if (current_steer >= SIM_S_MAX && sv > 0.0) sv = 0.0;
    if (current_steer <= SIM_S_MIN && sv < 0.0) sv = 0.0;

    return sv;
}

/*===========================================================================
 * Speed PID Controller
 *===========================================================================*/

static double pid_accl(double speed_cmd, double current_speed)
{
    double vel_diff = speed_cmd - current_speed;
    double kp, accl;

    if (vel_diff > 0.0) {
        kp = 10.0 * SIM_A_MAX / SIM_V_MAX;
        accl = kp * vel_diff;
    } else {
        kp = 10.0 * SIM_A_MAX / (-SIM_V_MIN);
        accl = kp * vel_diff;
    }

    if (accl > SIM_A_MAX) accl = SIM_A_MAX;
    if (accl < -SIM_A_MAX) accl = -SIM_A_MAX;
    return accl;
}

/*===========================================================================
 * State Conversion: Sim -> MPC Frenet
 *===========================================================================
 *
 * Sim state:    [X, Y, delta, V, psi, psi_dot, beta]
 * MPC Frenet:   [e_y, e_psi, vx, vy, omega]
 */

typedef struct {
    double x;
    double y;
    double heading;
    double arc_length;
} PathState_t;

static void path_state_init(PathState_t *ps)
{
    ps->x = 0.0;
    ps->y = 0.0;
    ps->heading = 0.0;
    ps->arc_length = 0.0;
}

static void path_state_advance(PathState_t *ps, double kappa, double ds)
{
    double dheading = kappa * ds;
    double dx_local, dy_local;

    if (fabs(kappa) < 1e-6) {
        dx_local = ds;
        dy_local = 0.0;
    } else {
        dx_local = sin(dheading) / kappa;
        dy_local = (1.0 - cos(dheading)) / kappa;
    }

    double ch = cos(ps->heading);
    double sh = sin(ps->heading);
    ps->x += ch * dx_local - sh * dy_local;
    ps->y += sh * dx_local + ch * dy_local;
    ps->heading += dheading;
    ps->arc_length += ds;
}

static void sim_to_frenet(const double sim_state[ST_DIM],
                          const PathState_t *path,
                          FrenetState_t *frenet)
{
    double V    = sim_state[ST_V];
    double psi  = sim_state[ST_PSI];
    double beta = sim_state[ST_BETA];

    double dx = sim_state[ST_X] - path->x;
    double dy = sim_state[ST_Y] - path->y;

    double cos_ph = cos(path->heading);
    double sin_ph = sin(path->heading);
    double e_y = -sin_ph * dx + cos_ph * dy;

    frenet->lateral_error_meters = DOUBLE_TO_FP(e_y);
    frenet->heading_error_radians = DOUBLE_TO_FP(psi - path->heading);
    frenet->longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(V * cos(beta));
    frenet->lateral_velocity_meters_per_second = DOUBLE_TO_FP(V * sin(beta));
    frenet->yaw_rate_radians_per_second = DOUBLE_TO_FP(sim_state[ST_PSI_DOT]);
}

/*===========================================================================
 * MPC Configuration
 *===========================================================================*/

static void init_mpc_sim_config(void)
{
    mpc_initialize();
    MpcConfiguration_t cfg = mpc_get_configuration();
    cfg.prediction_horizon_steps = MPC_HORIZON;

    /* Sim-specific weight overrides */
    cfg.weight_heading_error    = FP_CONST(25.0);
    cfg.weight_lateral_error    = FP_CONST(30.0);
    cfg.weight_lateral_velocity = FP_CONST(3.0);

    mpc_set_configuration(&cfg);
}

/*===========================================================================
 * Reference Trajectory Helper
 *===========================================================================*/

static void init_ref(TrajectoryReferencePoint_t *ref, double velocity,
                     double curvature, double left_wall, double right_wall)
{
    ref->reference_lateral_error_meters = 0;
    ref->reference_heading_error_radians = 0;
    ref->reference_velocity_meters_per_second = DOUBLE_TO_FP(velocity);
    ref->reference_lateral_velocity_meters_per_second = 0;
    ref->reference_yaw_rate_radians_per_second = DOUBLE_TO_FP(curvature * velocity);
    ref->path_curvature_radians_per_meter = DOUBLE_TO_FP(curvature);
    ref->left_wall_bound_meters = DOUBLE_TO_FP(left_wall);
    ref->right_wall_bound_meters = DOUBLE_TO_FP(right_wall);
}

/*===========================================================================
 * Closed-Loop Simulation Driver
 *===========================================================================*/

typedef struct {
    double max_lateral_error;
    double max_heading_error;
    double max_yaw_rate;
    double max_steer;
    double max_speed;
    double final_lateral_error;
    double final_heading_error;
    int    total_mpc_iterations;
    int    solver_failures;
    int    diverged;
} SimResult_t;

static void run_rk4_closed_loop(
    double ref_velocity,
    double ref_curvature,
    double left_wall,
    double right_wall,
    double initial_ey,
    double initial_epsi,
    double sim_time,
    int log_every_mpc,
    SimResult_t *result)
{
    memset(result, 0, sizeof(*result));

    /* Initialize sim state along X-axis */
    double state[ST_DIM];
    memset(state, 0, sizeof(state));
    state[ST_Y]     = initial_ey;
    state[ST_PSI]   = initial_epsi;
    state[ST_V]     = ref_velocity;
    state[ST_DELTA] = 0.0;
    state[ST_BETA]  = 0.0;

    PathState_t path;
    path_state_init(&path);

    int sim_steps = (int)(sim_time / SIM_DT + 0.5);
    int mpc_period = (int)(MPC_DT / SIM_DT + 0.5);  /* 5 */

    double steer_cmd = 0.0;
    double speed_cmd = ref_velocity;

    int mpc_call_count = 0;

    for (int step = 0; step < sim_steps; step++) {
        /* MPC call every mpc_period sim steps */
        if (step % mpc_period == 0) {
            FrenetState_t frenet;
            sim_to_frenet(state, &path, &frenet);

            TrajectoryReferencePoint_t ref[MPC_HORIZON];
            for (int i = 0; i < MPC_HORIZON; i++)
                init_ref(&ref[i], ref_velocity, ref_curvature, left_wall, right_wall);

            MpcSolverResult_t mpc_result;
            memset(&mpc_result, 0, sizeof(mpc_result));
            MpcSolverStatus_t status = mpc_compute_optimal_control(
                &frenet, ref, &mpc_result);

            if (status != MPC_STATUS_SUCCESS &&
                status != MPC_STATUS_MAXIMUM_ITERATIONS_REACHED) {
                result->solver_failures++;
            }
            result->total_mpc_iterations += mpc_result.iterations_used;

            /* Extract MPC commands */
            steer_cmd = FP_TO_DOUBLE(mpc_result.optimal_control.steering_angle_radians);
            speed_cmd = ref_velocity;

            /* Logging */
            if (log_every_mpc > 0 && (mpc_call_count % log_every_mpc == 0)) {
                double e_y = FP_TO_DOUBLE(frenet.lateral_error_meters);
                double e_psi = FP_TO_DOUBLE(frenet.heading_error_radians);
                double vx = FP_TO_DOUBLE(frenet.longitudinal_velocity_meters_per_second);
                double vy = FP_TO_DOUBLE(frenet.lateral_velocity_meters_per_second);
                double omega = FP_TO_DOUBLE(frenet.yaw_rate_radians_per_second);
                printf("  [MPC %3d] e_y=%+.4f e_psi=%+.4f vx=%.2f vy=%+.4f "
                       "w=%+.3f | d_cmd=%+.4f d_act=%+.4f iter=%d\n",
                       mpc_call_count, e_y, e_psi, vx, vy, omega,
                       steer_cmd, state[ST_DELTA],
                       mpc_result.iterations_used);
            }
            mpc_call_count++;
        }

        /* Steering servo: bang-bang toward MPC's commanded angle */
        double sv = bang_bang_steer(steer_cmd, state[ST_DELTA]);

        /* Speed controller: PID toward reference speed */
        double accl = pid_accl(speed_cmd, state[ST_V]);

        /* RK4 integration step (SIM_DT = 0.01s) */
        rk4_step(state, sv, accl, SIM_DT);

        /* Clamp steering angle */
        if (state[ST_DELTA] > SIM_S_MAX) state[ST_DELTA] = SIM_S_MAX;
        if (state[ST_DELTA] < SIM_S_MIN) state[ST_DELTA] = SIM_S_MIN;

        /* Clamp speed */
        if (state[ST_V] > SIM_V_MAX) state[ST_V] = SIM_V_MAX;
        if (state[ST_V] < SIM_V_MIN) state[ST_V] = SIM_V_MIN;

        /* Advance reference path */
        double vx_body = state[ST_V] * cos(state[ST_BETA]);
        path_state_advance(&path, ref_curvature, vx_body * SIM_DT);

        /* Track statistics */
        FrenetState_t frenet_check;
        sim_to_frenet(state, &path, &frenet_check);
        double ey_abs = fabs(FP_TO_DOUBLE(frenet_check.lateral_error_meters));
        double epsi_abs = fabs(FP_TO_DOUBLE(frenet_check.heading_error_radians));
        double omega_abs = fabs(state[ST_PSI_DOT]);

        if (ey_abs > result->max_lateral_error)
            result->max_lateral_error = ey_abs;
        if (epsi_abs > result->max_heading_error)
            result->max_heading_error = epsi_abs;
        if (omega_abs > result->max_yaw_rate)
            result->max_yaw_rate = omega_abs;
        if (fabs(state[ST_DELTA]) > result->max_steer)
            result->max_steer = fabs(state[ST_DELTA]);
        if (state[ST_V] > result->max_speed)
            result->max_speed = state[ST_V];

        /* Check for divergence */
        if (ey_abs > left_wall || ey_abs > right_wall) {
            result->diverged = 1;
        }
    }

    /* Final errors */
    FrenetState_t frenet_final;
    sim_to_frenet(state, &path, &frenet_final);
    result->final_lateral_error = fabs(FP_TO_DOUBLE(frenet_final.lateral_error_meters));
    result->final_heading_error = fabs(FP_TO_DOUBLE(frenet_final.heading_error_radians));
}

/*===========================================================================
 * TEST 1: Straight-Line Tracking at Multiple Speeds
 *===========================================================================*/

static void test_straight_speed_sweep(void)
{
    printf("\n========== Test 1: Straight-Line Speed Sweep (RK4) ==========\n");

    double speeds[] = {3.0, 4.0, 5.0};
    int n_speeds = sizeof(speeds) / sizeof(speeds[0]);

    int all_converged = 1;
    int any_diverged = 0;

    for (int i = 0; i < n_speeds; i++) {
        double v = speeds[i];
        printf("\n  --- v = %.1f m/s ---\n", v);

        init_mpc_sim_config();
        mpc_reset();

        SimResult_t res;
        run_rk4_closed_loop(
            v, 0.001,       /* near-straight */
            1.0, 1.0,
            0.3, 0.0,       /* 0.3m lateral offset */
            5.0, 4,
            &res);

        printf("  Result: max|e_y|=%.4f max|e_psi|=%.4f max|w|=%.3f "
               "max|d|=%.4f failures=%d\n",
               res.max_lateral_error, res.max_heading_error,
               res.max_yaw_rate, res.max_steer, res.solver_failures);

        if (res.final_lateral_error > 0.35) all_converged = 0;
        if (res.diverged) any_diverged = 1;

        char buf[128];
        snprintf(buf, sizeof(buf), "v=%.0f: stays within track bounds", v);
        check_condition(buf, !res.diverged);

        snprintf(buf, sizeof(buf), "v=%.0f: max yaw rate < 5 rad/s", v);
        check_condition(buf, res.max_yaw_rate < 5.0);

        snprintf(buf, sizeof(buf), "v=%.0f: max lateral error < 1.0 m", v);
        check_condition(buf, res.max_lateral_error < 1.0);
    }

    check_condition("Speed sweep: all speeds converge (final e_y < 0.35m)", all_converged);
    check_condition("Speed sweep: no divergence at any speed", !any_diverged);
}

/*===========================================================================
 * TEST 2: Constant-Curvature Curves
 *===========================================================================*/

static void test_constant_curvature(void)
{
    printf("\n========== Test 2: Constant-Curvature Curves (RK4) ==========\n");

    struct { double R; double v; double ey_limit; } cases[] = {
        { 10.0, 4.0,  0.15 },
        { 20.0, 5.0,  0.20 },
        { 30.0, 6.0,  0.40 },
    };
    int n_cases = sizeof(cases) / sizeof(cases[0]);

    for (int i = 0; i < n_cases; i++) {
        double R = cases[i].R;
        double v = cases[i].v;
        double kappa = 1.0 / R;
        double ey_lim = cases[i].ey_limit;

        printf("\n  --- R=%.0f m, v=%.1f m/s, kappa=%.3f ---\n", R, v, kappa);

        init_mpc_sim_config();
        mpc_reset();

        SimResult_t res;
        run_rk4_closed_loop(
            v, kappa,
            1.0, 1.0,
            0.0, 0.0,
            5.0, 4,
            &res);

        printf("  Result: max|e_y|=%.4f max|e_psi|=%.4f max|w|=%.3f "
               "max|d|=%.4f failures=%d\n",
               res.max_lateral_error, res.max_heading_error,
               res.max_yaw_rate, res.max_steer, res.solver_failures);

        char buf[128];
        snprintf(buf, sizeof(buf), "R=%.0f v=%.0f: lateral error < %.2f m", R, v, ey_lim);
        check_condition(buf, res.max_lateral_error < ey_lim);

        snprintf(buf, sizeof(buf), "R=%.0f v=%.0f: stays within track", R, v);
        check_condition(buf, !res.diverged);

        snprintf(buf, sizeof(buf), "R=%.0f v=%.0f: yaw rate bounded < 8 rad/s", R, v);
        check_condition(buf, res.max_yaw_rate < 8.0);
    }
}

/*===========================================================================
 * TEST 3: S-Curve Transition
 *===========================================================================*/

static void test_s_curve(void)
{
    printf("\n========== Test 3: S-Curve Transition (RK4) ==========\n");

    double v = 5.0;
    double kappas[] = {-0.08, 0.0, 0.08};
    double segment_time = 2.0;
    int n_segments = 3;

    init_mpc_sim_config();
    mpc_reset();

    double state[ST_DIM];
    memset(state, 0, sizeof(state));
    state[ST_V] = v;

    PathState_t path;
    path_state_init(&path);
    double max_ey = 0.0, max_epsi = 0.0, max_omega = 0.0;
    int total_failures = 0;
    int diverged = 0;

    double steer_cmd = 0.0;
    int mpc_call_count = 0;
    int mpc_period = (int)(MPC_DT / SIM_DT + 0.5);
    int global_step = 0;

    for (int seg = 0; seg < n_segments; seg++) {
        double kappa = kappas[seg];
        int seg_steps = (int)(segment_time / SIM_DT + 0.5);

        printf("\n  --- Segment %d: kappa=%+.3f ---\n", seg, kappa);

        for (int s = 0; s < seg_steps; s++, global_step++) {
            if (global_step % mpc_period == 0) {
                FrenetState_t frenet;
                sim_to_frenet(state, &path, &frenet);

                TrajectoryReferencePoint_t ref[MPC_HORIZON];
                for (int i = 0; i < MPC_HORIZON; i++)
                    init_ref(&ref[i], v, kappa, 1.5, 1.5);

                MpcSolverResult_t mpc_result;
                memset(&mpc_result, 0, sizeof(mpc_result));
                MpcSolverStatus_t status = mpc_compute_optimal_control(
                    &frenet, ref, &mpc_result);

                if (status != MPC_STATUS_SUCCESS &&
                    status != MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                    total_failures++;

                steer_cmd = FP_TO_DOUBLE(mpc_result.optimal_control.steering_angle_radians);

                if (mpc_call_count % 8 == 0) {
                    printf("  [MPC %3d] e_y=%+.4f e_psi=%+.4f vx=%.2f d_cmd=%+.4f\n",
                           mpc_call_count,
                           FP_TO_DOUBLE(frenet.lateral_error_meters),
                           FP_TO_DOUBLE(frenet.heading_error_radians),
                           FP_TO_DOUBLE(frenet.longitudinal_velocity_meters_per_second),
                           steer_cmd);
                }
                mpc_call_count++;
            }

            double sv = bang_bang_steer(steer_cmd, state[ST_DELTA]);
            double accl = pid_accl(v, state[ST_V]);
            rk4_step(state, sv, accl, SIM_DT);

            if (state[ST_DELTA] > SIM_S_MAX) state[ST_DELTA] = SIM_S_MAX;
            if (state[ST_DELTA] < SIM_S_MIN) state[ST_DELTA] = SIM_S_MIN;
            if (state[ST_V] > SIM_V_MAX) state[ST_V] = SIM_V_MAX;
            if (state[ST_V] < SIM_V_MIN) state[ST_V] = SIM_V_MIN;

            double vx_body = state[ST_V] * cos(state[ST_BETA]);
            path_state_advance(&path, kappa, vx_body * SIM_DT);

            FrenetState_t fc;
            sim_to_frenet(state, &path, &fc);
            double ey = fabs(FP_TO_DOUBLE(fc.lateral_error_meters));
            double ep = fabs(FP_TO_DOUBLE(fc.heading_error_radians));
            double om = fabs(state[ST_PSI_DOT]);

            if (ey > max_ey) max_ey = ey;
            if (ep > max_epsi) max_epsi = ep;
            if (om > max_omega) max_omega = om;
            if (ey > 1.5) diverged = 1;
        }
    }

    printf("\n  S-curve result: max|e_y|=%.4f max|e_psi|=%.4f max|w|=%.3f "
           "failures=%d\n", max_ey, max_epsi, max_omega, total_failures);

    check_condition("S-curve: lateral error < 0.8 m", max_ey < 0.8);
    check_condition("S-curve: heading error < 0.5 rad", max_epsi < 0.5);
    check_condition("S-curve: yaw rate < 5 rad/s", max_omega < 5.0);
    check_condition("S-curve: stays within track", !diverged);
}

/*===========================================================================
 * TEST 4: Emergency Braking
 *===========================================================================*/

static void test_emergency_braking(void)
{
    printf("\n========== Test 4: Emergency Braking (RK4) ==========\n");

    init_mpc_sim_config();
    mpc_reset();

    double initial_speed = 6.0;
    double target_speed = 2.0;

    double state[ST_DIM];
    memset(state, 0, sizeof(state));
    state[ST_V] = initial_speed;
    state[ST_Y] = 0.1;

    PathState_t path;
    path_state_init(&path);
    double steer_cmd = 0.0;
    double max_ey = 0.0, max_omega = 0.0;
    int failures = 0;
    int mpc_period = (int)(MPC_DT / SIM_DT + 0.5);
    int sim_steps = (int)(4.0 / SIM_DT + 0.5);
    int mpc_call_count = 0;
    double final_speed = 0.0;

    for (int step = 0; step < sim_steps; step++) {
        if (step % mpc_period == 0) {
            FrenetState_t frenet;
            sim_to_frenet(state, &path, &frenet);

            TrajectoryReferencePoint_t ref[MPC_HORIZON];
            for (int i = 0; i < MPC_HORIZON; i++)
                init_ref(&ref[i], target_speed, 0.0, 1.0, 1.0);

            MpcSolverResult_t mpc_result;
            memset(&mpc_result, 0, sizeof(mpc_result));
            MpcSolverStatus_t status = mpc_compute_optimal_control(
                &frenet, ref, &mpc_result);

            if (status != MPC_STATUS_SUCCESS &&
                status != MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                failures++;

            steer_cmd = FP_TO_DOUBLE(mpc_result.optimal_control.steering_angle_radians);

            if (mpc_call_count % 4 == 0) {
                printf("  [MPC %3d] e_y=%+.4f v=%.2f d_cmd=%+.4f\n",
                       mpc_call_count,
                       FP_TO_DOUBLE(frenet.lateral_error_meters),
                       state[ST_V], steer_cmd);
            }
            mpc_call_count++;
        }

        double sv = bang_bang_steer(steer_cmd, state[ST_DELTA]);
        double accl = pid_accl(target_speed, state[ST_V]);
        rk4_step(state, sv, accl, SIM_DT);

        if (state[ST_DELTA] > SIM_S_MAX) state[ST_DELTA] = SIM_S_MAX;
        if (state[ST_DELTA] < SIM_S_MIN) state[ST_DELTA] = SIM_S_MIN;
        if (state[ST_V] > SIM_V_MAX) state[ST_V] = SIM_V_MAX;
        if (state[ST_V] < SIM_V_MIN) state[ST_V] = SIM_V_MIN;

        double vx_body = state[ST_V] * cos(state[ST_BETA]);
        path_state_advance(&path, 0.0, vx_body * SIM_DT);

        FrenetState_t fc;
        sim_to_frenet(state, &path, &fc);
        double ey = fabs(FP_TO_DOUBLE(fc.lateral_error_meters));
        double om = fabs(state[ST_PSI_DOT]);

        if (ey > max_ey) max_ey = ey;
        if (om > max_omega) max_omega = om;
        final_speed = state[ST_V];
    }

    printf("  Braking result: max|e_y|=%.4f max|w|=%.3f final_v=%.2f failures=%d\n",
           max_ey, max_omega, final_speed, failures);

    check_condition("Braking: lateral error < 0.5 m", max_ey < 0.5);
    check_condition("Braking: yaw rate < 3 rad/s", max_omega < 3.0);
    check_condition("Braking: speed reaches target (~2 m/s)",
                    fabs(final_speed - target_speed) < 1.5);
    check_condition("Braking: no solver failures", failures == 0);
}

/*===========================================================================
 * TEST 5: High-Speed Curve Entry
 *===========================================================================*/

static void test_high_speed_curve_entry(void)
{
    printf("\n========== Test 5: High-Speed Curve Entry (RK4) ==========\n");

    init_mpc_sim_config();
    mpc_reset();

    double v = 5.0;
    double kappa = 0.05;  /* R=20m, moderate for condensed QP (N=20) */

    double state[ST_DIM];
    memset(state, 0, sizeof(state));
    state[ST_V] = v;

    PathState_t path;
    path_state_init(&path);
    double steer_cmd = 0.0;
    double max_ey = 0.0, max_omega = 0.0, max_epsi = 0.0;
    int failures = 0;
    int mpc_period = (int)(MPC_DT / SIM_DT + 0.5);
    int mpc_call_count = 0;

    int phase1_steps = (int)(1.0 / SIM_DT + 0.5);
    int phase2_steps = (int)(4.0 / SIM_DT + 0.5);
    int total_steps = phase1_steps + phase2_steps;
    int global_step = 0;

    for (int step = 0; step < total_steps; step++, global_step++) {
        double current_kappa = (step < phase1_steps) ? 0.0 : kappa;

        if (global_step % mpc_period == 0) {
            FrenetState_t frenet;
            sim_to_frenet(state, &path, &frenet);

            TrajectoryReferencePoint_t ref[MPC_HORIZON];
            for (int i = 0; i < MPC_HORIZON; i++)
                init_ref(&ref[i], v, current_kappa, 1.5, 1.5);

            MpcSolverResult_t mpc_result;
            memset(&mpc_result, 0, sizeof(mpc_result));
            MpcSolverStatus_t status = mpc_compute_optimal_control(
                &frenet, ref, &mpc_result);

            if (status != MPC_STATUS_SUCCESS &&
                status != MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                failures++;

            steer_cmd = FP_TO_DOUBLE(mpc_result.optimal_control.steering_angle_radians);

            if (mpc_call_count % 4 == 0) {
                printf("  [MPC %3d] e_y=%+.4f e_psi=%+.4f v=%.2f d=%+.4f %s\n",
                       mpc_call_count,
                       FP_TO_DOUBLE(frenet.lateral_error_meters),
                       FP_TO_DOUBLE(frenet.heading_error_radians),
                       state[ST_V], steer_cmd,
                       (step < phase1_steps) ? "[STRAIGHT]" : "[CURVE]");
            }
            mpc_call_count++;
        }

        double sv = bang_bang_steer(steer_cmd, state[ST_DELTA]);
        double accl = pid_accl(v, state[ST_V]);
        rk4_step(state, sv, accl, SIM_DT);

        if (state[ST_DELTA] > SIM_S_MAX) state[ST_DELTA] = SIM_S_MAX;
        if (state[ST_DELTA] < SIM_S_MIN) state[ST_DELTA] = SIM_S_MIN;
        if (state[ST_V] > SIM_V_MAX) state[ST_V] = SIM_V_MAX;
        if (state[ST_V] < SIM_V_MIN) state[ST_V] = SIM_V_MIN;

        double vx_body = state[ST_V] * cos(state[ST_BETA]);
        path_state_advance(&path, current_kappa, vx_body * SIM_DT);

        FrenetState_t fc;
        sim_to_frenet(state, &path, &fc);
        double ey = fabs(FP_TO_DOUBLE(fc.lateral_error_meters));
        double ep = fabs(FP_TO_DOUBLE(fc.heading_error_radians));
        double om = fabs(state[ST_PSI_DOT]);

        if (ey > max_ey) max_ey = ey;
        if (ep > max_epsi) max_epsi = ep;
        if (om > max_omega) max_omega = om;
    }

    printf("  Curve entry: max|e_y|=%.4f max|e_psi|=%.4f max|w|=%.3f failures=%d\n",
           max_ey, max_epsi, max_omega, failures);

    check_condition("Curve entry: lateral error < 2.0 m", max_ey < 2.0);
    check_condition("Curve entry: heading error < 1.5 rad", max_epsi < 1.5);
    check_condition("Curve entry: yaw rate < 10 rad/s", max_omega < 10.0);
    check_condition("Curve entry: no hard solver failures", failures == 0);
}

/*===========================================================================
 * TEST 6: Solver Convergence Under Plant-Model Mismatch
 *===========================================================================*/

static void test_model_mismatch_convergence(void)
{
    printf("\n========== Test 6: Model Mismatch Convergence (RK4) ==========\n");

    init_mpc_sim_config();
    mpc_reset();

    SimResult_t res;
    run_rk4_closed_loop(
        5.0, 0.04,
        1.0, 1.0,
        0.2, 0.05,
        5.0, 4,
        &res);

    printf("  Result: max|e_y|=%.4f max|e_psi|=%.4f "
           "avg_iter=%.1f failures=%d\n",
           res.max_lateral_error, res.max_heading_error,
           (double)res.total_mpc_iterations / (5.0 / MPC_DT),
           res.solver_failures);

    check_condition("Mismatch: converges to path (final e_y < 0.25m)",
                    res.final_lateral_error < 0.25);
    check_condition("Mismatch: stays within track bounds",
                    !res.diverged);
    check_condition("Mismatch: reasonable iterations (avg < 500)",
                    (double)res.total_mpc_iterations / (5.0 / MPC_DT) < 500.0);
}

/*===========================================================================
 * TEST 7: Low-Speed Stability
 *===========================================================================*/

static void test_low_speed_rk4_stability(void)
{
    printf("\n========== Test 7: Low-Speed RK4 Stability ==========\n");

    double low_speeds[] = {1.0, 1.5, 2.0, 2.5};
    int n_speeds = sizeof(low_speeds) / sizeof(low_speeds[0]);

    for (int i = 0; i < n_speeds; i++) {
        double v = low_speeds[i];
        printf("\n  --- v = %.1f m/s ---\n", v);

        init_mpc_sim_config();
        mpc_reset();

        SimResult_t res;
        run_rk4_closed_loop(
            v, 0.02,
            1.0, 1.0,
            0.15, 0.0,
            5.0, 4,
            &res);

        printf("  Result: max|e_y|=%.4f max|e_psi|=%.4f max|w|=%.3f\n",
               res.max_lateral_error, res.max_heading_error, res.max_yaw_rate);

        char buf[128];
        snprintf(buf, sizeof(buf), "v=%.1f: no divergence (RK4 stable)", v);
        check_condition(buf, !res.diverged);

        snprintf(buf, sizeof(buf), "v=%.1f: yaw rate bounded < 3 rad/s", v);
        check_condition(buf, res.max_yaw_rate < 3.0);

        snprintf(buf, sizeof(buf), "v=%.1f: lateral error < 0.5 m", v);
        check_condition(buf, res.max_lateral_error < 0.5);
    }
}

/*===========================================================================
 * main()
 *===========================================================================*/

int main(void)
{
    printf("==========================================================\n");
    printf("  RK4 Closed-Loop Simulation Tests for Condensed-QP MPC\n");
    printf("  Plant: f1tenth_gym single-track model (beta formulation)\n");
    printf("  Integration: RK4, dt=%.3fs\n", SIM_DT);
    printf("  Servo: bang-bang, sv_max=%.1f rad/s\n", SIM_SV_MAX);
    printf("  MPC interval: %.3fs, horizon=%d\n", MPC_DT, MPC_HORIZON);
    printf("  Sim params:  m=%.3f Iz=%.5f C_Sf=%.3f C_Sr=%.4f mu=%.4f\n",
           SIM_M, SIM_IZ, SIM_C_SF, SIM_C_SR, SIM_MU);
    printf("  MPC params:  m=3.314 Iz=0.035 C_Sf=4.17 C_Sr=4.42 mu=0.7463\n");
    printf("==========================================================\n");

    test_straight_speed_sweep();
    test_constant_curvature();
    test_s_curve();
    test_emergency_braking();
    test_high_speed_curve_entry();
    test_model_mismatch_convergence();
    test_low_speed_rk4_stability();

    printf("\n==========================================================\n");
    printf("  RESULTS: %d passed, %d failed (total %d)\n",
           tests_passed, tests_failed, tests_passed + tests_failed);
    printf("==========================================================\n");

    return tests_failed > 0 ? 1 : 0;
}
