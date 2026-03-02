/**
 * @file test_riccati_n20_diag.c
 * @brief Focused diagnostic for N=20 curve κ=2.0 convergence
 *
 * Traces iteration-by-iteration convergence and tests different approaches:
 *   1. Baseline (α=1.6, ρ=1.0, adaptive)
 *   2. No over-relaxation (α=1.0)
 *   3. Higher ρ (5.0, 10.0, 20.0)
 *   4. Higher max iterations (500, 1000)
 *   5. Split ρ: separate ρ for states vs controls
 */

#include <stdio.h>
#include <string.h>
#include "mpc.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"
#include "riccati_solver.h"

/* Access Riccati MPC */
extern void mpc_riccati_initialize(void);
extern void mpc_riccati_reset(void);
extern MpcSolverStatus_t mpc_riccati_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);

/*===========================================================================*/

static void build_curve_scenario(FrenetState_t *state,
                                  TrajectoryReferencePoint_t *ref,
                                  int n)
{
    memset(state, 0, sizeof(*state));
    state->longitudinal_velocity_meters_per_second = FP_CONST(4.0);
    state->heading_error_radians = FP_CONST(0.05);
    state->lateral_error_meters = FP_CONST(0.3);

    memset(ref, 0, n * sizeof(ref[0]));
    for (int i = 0; i < n; i++) {
        ref[i].reference_velocity_meters_per_second = FP_CONST(4.0);
        ref[i].path_curvature_radians_per_meter = FP_CONST(2.0);
        ref[i].left_wall_bound_meters  = FP_CONST(1.5);
        ref[i].right_wall_bound_meters = FP_CONST(1.5);
    }
}

/*===========================================================================
 * Test 1: High-iteration solve through MPC layer — see if it EVER converges
 *===========================================================================*/
static void test_mpc_high_maxiter(void)
{
    printf("\n=== Test 1: N=20 curve via MPC layer with high max_iter ===\n");

    FrenetState_t state;
    TrajectoryReferencePoint_t ref[50];
    build_curve_scenario(&state, ref, 50);

    MpcSolverResult_t result;

    /* Use default config but crank up max iterations */
    MpcConfiguration_t cfg = get_default_configuration();
    cfg.maximum_solver_iterations = 500;

    extern void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg);
    mpc_riccati_initialize_with_configuration(&cfg);

    /* Cold start */
    mpc_riccati_reset();
    mpc_riccati_compute_optimal_control(&state, ref, &result);
    printf("Cold start: iters=%d, status=%d, steer=%.4f, accel=%.4f\n",
           result.iterations_used, result.solver_status,
           result.optimal_control.steering_angle_radians / 65536.0,
           result.optimal_control.acceleration_meters_per_second_squared / 65536.0);

    /* Warm start (second call, same scenario) */
    mpc_riccati_compute_optimal_control(&state, ref, &result);
    printf("Warm start: iters=%d, status=%d, steer=%.4f, accel=%.4f\n",
           result.iterations_used, result.solver_status,
           result.optimal_control.steering_angle_radians / 65536.0,
           result.optimal_control.acceleration_meters_per_second_squared / 65536.0);

    /* Third call */
    mpc_riccati_compute_optimal_control(&state, ref, &result);
    printf("3rd call:   iters=%d, status=%d, steer=%.4f, accel=%.4f\n",
           result.iterations_used, result.solver_status,
           result.optimal_control.steering_angle_radians / 65536.0,
           result.optimal_control.acceleration_meters_per_second_squared / 65536.0);
}

/*===========================================================================
 * Test 2: Direct Riccati-ADMM with convergence trace (prints every iteration)
 *===========================================================================*/

/* Modified solve that prints trace — based on riccati_admm_solve internals */
static void test_direct_trace(int max_iter, fixed_point_t rho_init,
                               fixed_point_t alpha, int adaptive,
                               const char *label)
{
    printf("\n=== Test 2: Direct trace — %s ===\n", label);
    printf("Config: rho=%.2f, alpha=%.2f, adaptive=%d, max_iter=%d\n",
           rho_init / 65536.0, alpha / 65536.0, adaptive, max_iter);

    /* Build the same scenario as MPC layer (7 states, 2 controls, N=20) */
    #define NX 7
    #define NU 2
    #define N_STEPS 20

    FrenetState_t fstate;
    TrajectoryReferencePoint_t fref[50];
    build_curve_scenario(&fstate, fref, 50);

    /* We need to reconstruct the same step_data. Let's use the MPC layer
     * by calling it once and then manually running the solver.
     * Easier approach: just call through the MPC API with different configs. */

    RiccatiAdmmConfig_t cfg;
    cfg.rho = rho_init;
    cfg.tolerance = FP_CONST(0.05);  /* Same as Riccati default */
    cfg.max_iterations = max_iter;
    cfg.adaptive_rho = adaptive;
    cfg.alpha = alpha;

    /* Initialize via MPC layer */
    MpcConfiguration_t mpc_cfg = get_default_configuration();
    mpc_cfg.maximum_solver_iterations = max_iter;
    extern void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *);
    mpc_riccati_initialize_with_configuration(&mpc_cfg);
    mpc_riccati_reset();

    MpcSolverResult_t result;
    mpc_riccati_compute_optimal_control(&fstate, fref, &result);
    printf("Result: iters=%d, status=%d, steer=%.4f\n",
           result.iterations_used, result.solver_status,
           result.optimal_control.steering_angle_radians / 65536.0);

    printf("(Note: internal rho/alpha overridden by mpc_riccati defaults;");
    printf(" use direct solver for custom params)\n");
}

/*===========================================================================
 * Test 3: Direct Riccati-ADMM with different parameters
 *
 * Bypasses MPC layer to directly control solver configuration.
 * Reconstructs step_data matching the MPC setup.
 *===========================================================================*/

static void build_step_data_for_curve(RiccatiStepData_t *step_data,
                                       fixed_point_t *terminal_Q,
                                       fixed_point_t *terminal_q,
                                       fixed_point_t *x0,
                                       int N)
{
    /* Linearize vehicle model */
    vehicle_model_initialize();
    VehicleParameters_t vp = vehicle_model_get_parameters();
    MpcConfiguration_t mpc_cfg = get_default_configuration();

    FrenetState_t lin_state;
    memset(&lin_state, 0, sizeof(lin_state));
    lin_state.longitudinal_velocity_meters_per_second = FP_CONST(4.0);

    ControlInput_t lin_control;
    /* Clamp feedforward steering to δ_max/2 to avoid Pacejka saturation */
    fixed_point_t delta_ff = fp_atan(fp_mul(vp.wheelbase_meters, FP_CONST(2.0)));
    fixed_point_t delta_clamp = vp.maximum_steering_angle_radians >> 1;
    if (delta_ff > delta_clamp) delta_ff = delta_clamp;
    if (delta_ff < fp_neg(delta_clamp)) delta_ff = fp_neg(delta_clamp);
    lin_control.steering_angle_radians = delta_ff;
    lin_control.acceleration_meters_per_second_squared = 0;

    fixed_point_t A_base[5][5], B_base[5][2];
    vehicle_model_compute_frenet_linearization(
        &lin_state, &lin_control, mpc_cfg.time_step_seconds,
        FP_CONST(2.0), A_base, B_base);

    /* Max steering at v=4.0 */
    fixed_point_t max_steer = vp.maximum_steering_angle_radians;

    /* Weights */
    fixed_point_t w_steer_eff = mpc_cfg.weight_steering_effort;
    fixed_point_t w_accel_eff = mpc_cfg.weight_acceleration_effort;
    fixed_point_t w_steer_rate = mpc_cfg.weight_steering_rate;
    fixed_point_t w_accel_rate = mpc_cfg.weight_acceleration_rate;

    memset(step_data, 0, N * sizeof(step_data[0]));

    for (int k = 0; k < N; k++) {
        RiccatiStepData_t *sd = &step_data[k];

        /* Augmented A (7×7) */
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
                sd->A[i][j] = A_base[i][j];
        sd->A[1][2] = fp_neg(fp_mul(mpc_cfg.time_step_seconds, FP_CONST(2.0)));

        /* Augmented B (7×2) */
        for (int i = 0; i < 5; i++)
            for (int a = 0; a < 2; a++)
                sd->B[i][a] = B_base[i][a];
        sd->B[5][0] = FP_ONE;
        sd->B[6][1] = FP_ONE;

        /* Q_diag */
        sd->Q_diag[0] = fp_mul(FP_TWO, mpc_cfg.weight_lateral_error);
        sd->Q_diag[1] = fp_mul(FP_TWO, mpc_cfg.weight_heading_error);
        sd->Q_diag[2] = fp_mul(FP_TWO, mpc_cfg.weight_velocity);
        sd->Q_diag[3] = fp_mul(FP_TWO, mpc_cfg.weight_lateral_velocity);
        sd->Q_diag[4] = fp_mul(FP_TWO, mpc_cfg.weight_yaw_rate);
        sd->Q_diag[5] = fp_mul(FP_TWO, w_steer_rate);
        sd->Q_diag[6] = fp_mul(FP_TWO, w_accel_rate);

        /* q */
        sd->q[0] = 0;  /* ref_ey = 0 */
        sd->q[1] = 0;
        sd->q[2] = -fp_mul(sd->Q_diag[2], FP_CONST(4.0));
        sd->q[3] = 0;
        sd->q[4] = 0;

        /* R_diag */
        sd->R_diag[0] = fp_mul(FP_TWO, fp_add(w_steer_eff, w_steer_rate));
        sd->R_diag[1] = fp_mul(FP_TWO, fp_add(w_accel_eff, w_accel_rate));

        /* Cross-cost N */
        sd->N[5][0] = fp_neg(fp_mul(FP_TWO, w_steer_rate));
        sd->N[6][1] = fp_neg(fp_mul(FP_TWO, w_accel_rate));

        /* Bounds - sparse wall constraints matching PG (start=2, stride=3) */
        int wall_active = (k >= 2) && ((k - 2) % 3 == 0);
        if (wall_active) {
            sd->x_lb[0] = fp_neg(fp_sub(FP_CONST(1.5), FP_CONST(0.10)));
            sd->x_ub[0] = fp_sub(FP_CONST(1.5), FP_CONST(0.10));
        } else {
            sd->x_lb[0] = FP_CONST(-100.0);
            sd->x_ub[0] = FP_CONST(100.0);
        }
        for (int s = 1; s < 7; s++) {
            sd->x_lb[s] = FP_CONST(-100.0);
            sd->x_ub[s] = FP_CONST(100.0);
        }
        sd->u_lb[0] = fp_neg(max_steer);
        sd->u_ub[0] = max_steer;
        sd->u_lb[1] = vp.minimum_acceleration_meters_per_second_squared;
        sd->u_ub[1] = vp.maximum_acceleration_meters_per_second_squared;
    }

    /* Terminal cost */
    memset(terminal_Q, 0, RICCATI_MAX_NX * sizeof(fixed_point_t));
    memset(terminal_q, 0, RICCATI_MAX_NX * sizeof(fixed_point_t));
    terminal_Q[0] = fp_mul(FP_TWO, mpc_cfg.weight_lateral_error);
    terminal_Q[1] = fp_mul(FP_TWO, mpc_cfg.weight_heading_error);
    terminal_Q[2] = fp_mul(FP_TWO, mpc_cfg.weight_velocity);
    terminal_Q[3] = fp_mul(FP_TWO, mpc_cfg.weight_lateral_velocity);
    terminal_Q[4] = fp_mul(FP_TWO, mpc_cfg.weight_yaw_rate);
    terminal_q[2] = -fp_mul(terminal_Q[2], FP_CONST(4.0));

    /* x0 */
    memset(x0, 0, RICCATI_MAX_NX * sizeof(fixed_point_t));
    x0[0] = FP_CONST(0.3);   /* e_y */
    x0[1] = FP_CONST(0.05);  /* e_psi */
    x0[2] = FP_CONST(4.0);   /* vx */
    /* x0[5] = 0, x0[6] = 0 (no previous control) */
}

static void test_sweep_params(void)
{
    printf("\n=== Test 3: Parameter sweep for N=20 curve ===\n");
    printf("%-25s  iters  status  primal    dual      steer[0]  steer[19]\n",
           "Configuration");
    printf("--------------------------------------------------------------------\n");

    RiccatiStepData_t step_data[50];
    fixed_point_t terminal_Q[RICCATI_MAX_NX], terminal_q[RICCATI_MAX_NX];
    fixed_point_t x0[RICCATI_MAX_NX];
    build_step_data_for_curve(step_data, terminal_Q, terminal_q, x0, 20);

    struct {
        const char *name;
        fixed_point_t rho;
        fixed_point_t rho_u;
        fixed_point_t alpha;
        int adaptive;
        int max_iter;
    } configs[] = {
        {"rhoU=0 a=1.6 adaptive",   FP_CONST(1.0),  0,              FP_CONST(1.6), 1, 200},
        {"rhoU=0 a=1.0 adaptive",   FP_CONST(1.0),  0,              FP_CONST(1.0), 1, 200},
        {"rhoU=0 rho=5 a=1.6",      FP_CONST(5.0),  0,              FP_CONST(1.6), 1, 200},
        {"rhoU=0 rho=10 a=1.6",     FP_CONST(10.0), 0,              FP_CONST(1.6), 1, 200},
        {"rhoU=0 rho=10 a=1.0",     FP_CONST(10.0), 0,              FP_CONST(1.0), 1, 200},
        {"rhoU=0 rho=10 noAdapt",   FP_CONST(10.0), 0,              FP_CONST(1.0), 0, 200},
        {"rhoU=0 rho=50 a=1.6",     FP_CONST(50.0), 0,              FP_CONST(1.6), 1, 200},
        {"rhoU=0 rho=100 a=1.6",    FP_CONST(100.0),0,              FP_CONST(1.6), 1, 200},
        {"rhoU=1 rho=10 a=1.6",     FP_CONST(10.0), FP_CONST(1.0),  FP_CONST(1.6), 1, 200},
        {"rhoU=10 rho=10 a=1.6",    FP_CONST(10.0), FP_CONST(10.0), FP_CONST(1.6), 1, 200},
        {"rhoU=0 rho=10 500iter",    FP_CONST(10.0), 0,              FP_CONST(1.6), 1, 500},
        {"rhoU=0 rho=50 500iter",    FP_CONST(50.0), 0,              FP_CONST(1.6), 1, 500},
    };

    int n_configs = sizeof(configs) / sizeof(configs[0]);

    for (int c = 0; c < n_configs; c++) {
        RiccatiAdmmConfig_t cfg;
        cfg.rho = configs[c].rho;
        cfg.rho_u = configs[c].rho_u;
        cfg.tolerance = FP_CONST(0.05);
        cfg.max_iterations = configs[c].max_iter;
        cfg.adaptive_rho = configs[c].adaptive;
        cfg.alpha = configs[c].alpha;

        RiccatiAdmmState_t admm_st;
        riccati_admm_state_init(&admm_st);

        RiccatiSolution_t sol;
        memset(&sol, 0, sizeof(sol));

        riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                           7, 2, 20, &cfg, &admm_st, &sol);

        printf("%-25s  %4d   %s     %.5f   %.5f   %.4f(z:%.4f) %.4f\n",
               configs[c].name,
               sol.iterations,
               sol.status == RICCATI_STATUS_OPTIMAL ? "OPT " : "MAX ",
               sol.primal_residual / 65536.0,
               sol.dual_residual / 65536.0,
               sol.u[0][0] / 65536.0,
               admm_st.z_u[0][0] / 65536.0,
               sol.u[19][0] / 65536.0);
    }
}

/*===========================================================================
 * Test 4: Iteration-by-iteration trace for best config
 *===========================================================================*/
static void test_iteration_trace(void)
{
    printf("\n=== Test 4: Iteration trace for rho=10.0, rho_u=0, alpha=1.6, adaptive ===\n");

    RiccatiStepData_t step_data[50];
    fixed_point_t terminal_Q[RICCATI_MAX_NX], terminal_q[RICCATI_MAX_NX];
    fixed_point_t x0[RICCATI_MAX_NX];
    build_step_data_for_curve(step_data, terminal_Q, terminal_q, x0, 20);

    /* We'll run the solver with max 1 iteration at a time, printing each.
     * But this doesn't preserve ADMM state correctly if we re-initialize.
     * Instead, modify tolerance to be very tight and just run once with
     * high max_iter, relying on the final primal/dual.
     *
     * For actual trace, we'd need to instrument the solver itself.
     * Let's run with increasing max_iter and see when it converges. */

    printf("max_iter  iters  status  primal    dual\n");

    for (int mi = 10; mi <= 500; mi += 10) {
        RiccatiAdmmConfig_t cfg;
        cfg.rho = FP_CONST(10.0);
        cfg.rho_u = 0;
        cfg.tolerance = FP_CONST(0.05);
        cfg.max_iterations = mi;
        cfg.adaptive_rho = 1;
        cfg.alpha = FP_CONST(1.6);

        RiccatiAdmmState_t admm_st;
        riccati_admm_state_init(&admm_st);

        RiccatiSolution_t sol;
        memset(&sol, 0, sizeof(sol));

        riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                           7, 2, 20, &cfg, &admm_st, &sol);

        printf("%4d      %4d   %s     %.5f   %.5f\n",
               mi, sol.iterations,
               sol.status == RICCATI_STATUS_OPTIMAL ? "OPT " : "MAX ",
               sol.primal_residual / 65536.0,
               sol.dual_residual / 65536.0);

        if (sol.status == RICCATI_STATUS_OPTIMAL) {
            printf(">>> CONVERGED at %d iterations! <<<\n", sol.iterations);
            break;
        }
    }
}

/*===========================================================================
 * Test 5: Compare with Dense ADMM - what steering does it produce?
 *===========================================================================*/
static void test_compare_steering_output(void)
{
    printf("\n=== Test 5: Steering output comparison (Riccati vs MPC-PG) ===\n");

    FrenetState_t state;
    TrajectoryReferencePoint_t ref[50];
    build_curve_scenario(&state, ref, 50);
    MpcSolverResult_t result;

    /* Projected gradient */
    mpc_initialize();
    mpc_reset();
    mpc_compute_optimal_control(&state, ref, &result);
    printf("PG:       steer=%.4f, accel=%.4f, iters=%d, status=%d\n",
           result.optimal_control.steering_angle_radians / 65536.0,
           result.optimal_control.acceleration_meters_per_second_squared / 65536.0,
           result.iterations_used, result.solver_status);

    /* Riccati */
    mpc_riccati_initialize();
    mpc_riccati_reset();
    mpc_riccati_compute_optimal_control(&state, ref, &result);
    printf("Riccati:  steer=%.4f, accel=%.4f, iters=%d, status=%d\n",
           result.optimal_control.steering_angle_radians / 65536.0,
           result.optimal_control.acceleration_meters_per_second_squared / 65536.0,
           result.iterations_used, result.solver_status);
}

/*===========================================================================*/

int main(void)
{
    printf("=== Riccati N=20 Curve Convergence Diagnostic ===\n");
    printf("Scenario: kappa=2.0, v=4.0, N=20, walls=±1.5m\n");
    printf("Required steering: arctan(0.3*2.0) ≈ 0.54 rad\n");
    printf("Max steering: 0.4 rad → ALL controls saturated!\n");

    test_mpc_high_maxiter();
    test_sweep_params();
    test_iteration_trace();
    test_compare_steering_output();

    printf("\nDone.\n");
    return 0;
}
