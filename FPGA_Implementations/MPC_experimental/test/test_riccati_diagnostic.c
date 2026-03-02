/**
 * @file test_riccati_diagnostic.c
 * @brief Diagnostic tool for Riccati-ADMM convergence on curves
 *
 * Prints per-iteration residuals, P matrix norms, and K gain magnitudes
 * to identify fixed-point precision issues.
 */

#include "riccati_solver.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* Riccati MPC API */
extern void mpc_riccati_initialize(void);
extern void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg);
extern void mpc_riccati_reset(void);
extern MpcSolverStatus_t mpc_riccati_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);

/*===========================================================================
 * Test: Raw Riccati on a curve-like problem with diagnostics
 *===========================================================================*/

static void test_raw_curve_convergence(void)
{
    printf("\n=== Diagnostic: Raw Riccati on constrained curve ===\n");
    printf("Testing if Q16.16 precision limits convergence.\n\n");

    /* Simple 2D system: [position, velocity] with tight control bounds */
    int nx = 2, nu = 1, N = 10;
    fixed_point_t dt = FP_CONST(0.05);

    RiccatiStepData_t step_data[20];
    memset(step_data, 0, sizeof(step_data));

    for (int k = 0; k < N; k++) {
        step_data[k].A[0][0] = FP_ONE;
        step_data[k].A[0][1] = dt;    /* position += dt * velocity */
        step_data[k].A[1][1] = FP_ONE; /* velocity += dt * u */
        step_data[k].B[1][0] = dt;

        step_data[k].Q_diag[0] = FP_CONST(10.0);  /* Strong position tracking */
        step_data[k].Q_diag[1] = FP_CONST(1.0);
        step_data[k].R_diag[0] = FP_CONST(0.1);   /* Cheap control */

        /* Tight control bounds — forces constraint activity */
        step_data[k].u_lb[0] = FP_CONST(-2.0);
        step_data[k].u_ub[0] = FP_CONST(2.0);

        /* Wide state bounds */
        step_data[k].x_lb[0] = FP_CONST(-100.0);
        step_data[k].x_ub[0] = FP_CONST(100.0);
        step_data[k].x_lb[1] = FP_CONST(-100.0);
        step_data[k].x_ub[1] = FP_CONST(100.0);
    }

    fixed_point_t terminal_Q[RICCATI_MAX_NX] = {FP_CONST(20.0), FP_CONST(2.0)};
    fixed_point_t terminal_q[RICCATI_MAX_NX] = {0};
    fixed_point_t x0[RICCATI_MAX_NX] = { FP_CONST(3.0), 0 };  /* Far from origin */

    /* Test multiple rho values */
    fixed_point_t rho_values[] = {
        FP_CONST(0.1), FP_CONST(0.5), FP_CONST(1.0),
        FP_CONST(5.0), FP_CONST(10.0), FP_CONST(50.0)
    };
    int n_rhos = sizeof(rho_values) / sizeof(rho_values[0]);

    for (int ri = 0; ri < n_rhos; ri++) {
        printf("--- rho = %.1f ---\n", FP_TO_DOUBLE(rho_values[ri]));

        RiccatiAdmmConfig_t cfg;
        riccati_admm_config_init(&cfg);
        cfg.rho = rho_values[ri];
        cfg.tolerance = FP_CONST(0.001);  /* Tight tolerance to see convergence floor */
        cfg.max_iterations = 200;
        cfg.adaptive_rho = 0;  /* Fixed rho for this test */

        RiccatiAdmmState_t admm_state;
        riccati_admm_state_init(&admm_state);

        RiccatiSolution_t sol;
        memset(&sol, 0, sizeof(sol));

        riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                           nx, nu, N, &cfg, &admm_state, &sol);

        printf("  Status: %d, Iters: %d, Primal: %.6f, Dual: %.6f\n",
               sol.status, sol.iterations,
               FP_TO_DOUBLE(sol.primal_residual),
               FP_TO_DOUBLE(sol.dual_residual));
        printf("  u[0]=%.4f, x[N]=[%.4f, %.4f]\n\n",
               FP_TO_DOUBLE(sol.u[0][0]),
               FP_TO_DOUBLE(sol.x[N][0]),
               FP_TO_DOUBLE(sol.x[N][1]));
    }

    /* Now with adaptive rho */
    printf("--- Adaptive rho (start=1.0) ---\n");
    {
        RiccatiAdmmConfig_t cfg;
        riccati_admm_config_init(&cfg);
        cfg.tolerance = FP_CONST(0.001);
        cfg.max_iterations = 200;

        RiccatiAdmmState_t admm_state;
        riccati_admm_state_init(&admm_state);

        RiccatiSolution_t sol;
        memset(&sol, 0, sizeof(sol));

        riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                           nx, nu, N, &cfg, &admm_state, &sol);

        printf("  Status: %d, Iters: %d, Primal: %.6f, Dual: %.6f\n",
               sol.status, sol.iterations,
               FP_TO_DOUBLE(sol.primal_residual),
               FP_TO_DOUBLE(sol.dual_residual));
        printf("  u[0]=%.4f, x[N]=[%.4f, %.4f]\n\n",
               FP_TO_DOUBLE(sol.u[0][0]),
               FP_TO_DOUBLE(sol.x[N][0]),
               FP_TO_DOUBLE(sol.x[N][1]));
    }
}

/*===========================================================================
 * Test: Full MPC curve with iteration-level residual output
 *===========================================================================*/

static void test_mpc_curve_detailed(void)
{
    printf("\n=== Diagnostic: Full MPC Curve (iteration-level) ===\n");
    printf("Testing 7-state augmented MPC on κ=2.0 curve.\n\n");

    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 10;
    mpc_riccati_initialize_with_configuration(&cfg);

    FrenetState_t state;
    memset(&state, 0, sizeof(state));
    state.longitudinal_velocity_meters_per_second = FP_CONST(4.0);

    TrajectoryReferencePoint_t refs[20];
    memset(refs, 0, sizeof(refs));
    for (int i = 0; i < 20; i++) {
        refs[i].reference_velocity_meters_per_second = FP_CONST(4.0);
        refs[i].path_curvature_radians_per_meter = FP_CONST(2.0);
        refs[i].left_wall_bound_meters = FP_CONST(5.0);
        refs[i].right_wall_bound_meters = FP_CONST(5.0);
    }

    MpcSolverResult_t result;
    memset(&result, 0, sizeof(result));

    MpcSolverStatus_t status = mpc_riccati_compute_optimal_control(&state, refs, &result);

    printf("  MPC Status: %d, Iters: %d\n", status, result.iterations_used);
    printf("  Steer: %.4f rad, Accel: %.4f m/s^2\n",
           FP_TO_DOUBLE(result.optimal_control.steering_angle_radians),
           FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared));
    printf("  Final cost (primal res): %.6f\n\n", FP_TO_DOUBLE(result.final_cost));
}

/*===========================================================================
 * Test: Precision floor investigation
 *
 * What's the smallest residual that Q16.16 can represent?
 * FP_ONE = 65536 (1.0 in Q16.16)
 * Smallest positive value: 1 ≈ 0.0000153
 * Tolerance of 0.001 = 65 (integer representation)
 * Tolerance of 0.01 = 655
 * Tolerance of 0.05 = 3277
 *===========================================================================*/

static void test_precision_limits(void)
{
    printf("\n=== Q16.16 Precision Limits ===\n");
    printf("  FP_ONE = %d\n", FP_ONE);
    printf("  Resolution: %.10f\n", 1.0 / FP_ONE);
    printf("  Max value: %.2f\n", (double)INT32_MAX / FP_ONE);
    printf("  Min positive: %.10f\n", 1.0 / FP_ONE);
    printf("\n");
    printf("  0.001 in Q16.16 = %d\n", FP_CONST(0.001));
    printf("  0.01  in Q16.16 = %d\n", FP_CONST(0.01));
    printf("  0.05  in Q16.16 = %d\n", FP_CONST(0.05));
    printf("  0.1   in Q16.16 = %d\n", FP_CONST(0.1));
    printf("\n");

    /* Test multiplication precision at small scale */
    fixed_point_t a = FP_CONST(0.001);
    fixed_point_t b = FP_CONST(0.001);
    int64_t product = ((int64_t)a * (int64_t)b) >> FP_FRAC_BITS;
    printf("  0.001 * 0.001 = %d (expect ~0, actual %.10f)\n",
           (int)product, (double)product / FP_ONE);

    a = FP_CONST(0.01);
    b = FP_CONST(0.01);
    product = ((int64_t)a * (int64_t)b) >> FP_FRAC_BITS;
    printf("  0.01 * 0.01 = %d (expect ~6, actual %.10f)\n",
           (int)product, (double)product / FP_ONE);

    /* Test large accumulation */
    printf("\n  === Large P matrix simulation ===\n");
    /* Simulate A^T P A with A having entries ~0.05-1.0 and P having entries ~10-1000 */
    fixed_point_t P_entry = FP_CONST(500.0);
    fixed_point_t A_entry = FP_CONST(0.95);
    int64_t AtPA = ((int64_t)A_entry * (int64_t)P_entry) >> FP_FRAC_BITS;
    AtPA = (AtPA * (int64_t)A_entry) >> FP_FRAC_BITS;
    printf("  0.95 * 500 * 0.95 = %.4f (expect 451.25)\n",
           (double)AtPA / FP_ONE);

    /* Check if P can overflow Q16.16 range */
    printf("\n  If P grows to 1000: %d (max int32: %d) — %s\n",
           FP_CONST(1000.0), INT32_MAX,
           FP_CONST(1000.0) < INT32_MAX ? "OK" : "OVERFLOW");
    printf("  If P grows to 10000: int repr = %lld (max int32: %d) — %s\n",
           (long long)((int64_t)10000 << FP_FRAC_BITS), (int)INT32_MAX,
           ((int64_t)10000 << FP_FRAC_BITS) < INT32_MAX ? "OK" : "OVERFLOW");
    printf("  If P grows to 30000: int repr = %lld (max int32: %d) — %s\n",
           (long long)((int64_t)30000 << FP_FRAC_BITS), (int)INT32_MAX,
           ((int64_t)30000 << FP_FRAC_BITS) < INT32_MAX ? "OK" : "OVERFLOW");
    printf("  Q16.16 max representable: %.2f\n", (double)INT32_MAX / (1 << 16));

    /* Try Q8.24 and Q4.28 representation */
    printf("\n  === Alternative formats ===\n");
    int q8_24_frac = 24;
    int q4_28_frac = 28;
    printf("  Q8.24: range ±%.2f, resolution %.10f\n",
           (double)INT32_MAX / (1 << q8_24_frac),
           1.0 / (1 << q8_24_frac));
    printf("  Q4.28: range ±%.2f, resolution %.10f\n",
           (double)INT32_MAX / (1 << q4_28_frac),
           1.0 / (1 << q4_28_frac));
    printf("  Q16.16: range ±%.2f, resolution %.10f\n",
           (double)INT32_MAX / (1 << FP_FRAC_BITS),
           1.0 / (1 << FP_FRAC_BITS));
}

/*===========================================================================
 * Test: Compare convergence with different max_iterations
 *===========================================================================*/

static void test_convergence_over_iterations(void)
{
    printf("\n=== Convergence Trace for Curve MPC ===\n");
    printf("Running with max_iterations=200, printing final residuals.\n\n");

    /* Use raw Riccati solver with a problem that mimics MPC curve */
    int nx = 7, nu = 2, N = 10;
    fixed_point_t dt = FP_CONST(0.05);

    RiccatiStepData_t step_data[20];
    memset(step_data, 0, sizeof(step_data));

    /* Build augmented dynamics similar to mpc_riccati for κ=2.0, v=4.0 */
    fixed_point_t vx = FP_CONST(4.0);
    fixed_point_t kappa = FP_CONST(2.0);

    for (int k = 0; k < N; k++) {
        /* Frenet 5×5 block (simplified) */
        step_data[k].A[0][0] = FP_ONE;
        step_data[k].A[0][1] = fp_mul(vx, dt);     /* e_y += vx*dt*e_psi */
        step_data[k].A[1][1] = FP_ONE;
        step_data[k].A[1][4] = dt;                   /* e_psi += dt*omega */
        step_data[k].A[1][2] = fp_neg(fp_mul(dt, kappa)); /* e_psi -= dt*kappa*vx approx */
        step_data[k].A[2][2] = FP_ONE;
        step_data[k].A[3][3] = FP_ONE;
        step_data[k].A[4][4] = FP_ONE;

        /* B: steering->omega, accel->vx */
        step_data[k].B[4][0] = FP_CONST(5.0);  /* omega += gain*delta */
        step_data[k].B[2][1] = dt;               /* vx += dt*accel */

        /* Augmented states 5,6 = prev controls */
        step_data[k].B[5][0] = FP_ONE;
        step_data[k].B[6][1] = FP_ONE;

        /* Cost weights */
        step_data[k].Q_diag[0] = FP_CONST(1.0);   /* lateral error */
        step_data[k].Q_diag[1] = FP_CONST(0.5);   /* heading */
        step_data[k].Q_diag[2] = FP_CONST(0.2);   /* velocity */
        step_data[k].Q_diag[3] = FP_CONST(0.1);   /* lat vel */
        step_data[k].Q_diag[4] = FP_CONST(0.1);   /* yaw rate */
        step_data[k].Q_diag[5] = FP_CONST(0.02);  /* rate: prev steer */
        step_data[k].Q_diag[6] = FP_CONST(0.02);  /* rate: prev accel */

        step_data[k].R_diag[0] = FP_CONST(0.5);   /* steering effort + rate */
        step_data[k].R_diag[1] = FP_CONST(0.5);

        /* Cross-cost for rate penalty */
        step_data[k].N[5][0] = FP_CONST(-0.02);  /* -2*w_steer_rate */
        step_data[k].N[6][1] = FP_CONST(-0.02);

        /* Control bounds */
        step_data[k].u_lb[0] = FP_CONST(-0.4);   /* steering */
        step_data[k].u_ub[0] = FP_CONST(0.4);
        step_data[k].u_lb[1] = FP_CONST(-7.7);   /* accel */
        step_data[k].u_ub[1] = FP_CONST(7.7);

        /* State bounds (wide except e_y) */
        for (int s = 0; s < 7; s++) {
            step_data[k].x_lb[s] = FP_CONST(-1000.0);
            step_data[k].x_ub[s] = FP_CONST(1000.0);
        }
        step_data[k].x_lb[0] = FP_CONST(-5.0);   /* wall bounds */
        step_data[k].x_ub[0] = FP_CONST(5.0);
    }

    fixed_point_t terminal_Q[RICCATI_MAX_NX] = {
        FP_CONST(2.0), FP_CONST(1.0), FP_CONST(0.4),
        FP_CONST(0.2), FP_CONST(0.2), FP_CONST(0.04), FP_CONST(0.04)
    };
    fixed_point_t terminal_q[RICCATI_MAX_NX] = {0};
    fixed_point_t x0[RICCATI_MAX_NX] = {0, 0, vx, 0, 0, 0, 0};

    /* Test with increasing iterations */
    int iter_counts[] = {10, 25, 50, 100, 200};
    int n_tests = sizeof(iter_counts) / sizeof(iter_counts[0]);

    for (int t = 0; t < n_tests; t++) {
        RiccatiAdmmConfig_t cfg;
        riccati_admm_config_init(&cfg);
        cfg.rho = FP_CONST(1.0);
        cfg.tolerance = FP_CONST(0.001);  /* Very tight */
        cfg.max_iterations = iter_counts[t];

        RiccatiAdmmState_t admm_state;
        riccati_admm_state_init(&admm_state);

        RiccatiSolution_t sol;
        memset(&sol, 0, sizeof(sol));

        riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                           nx, nu, N, &cfg, &admm_state, &sol);

        printf("  max_iter=%3d → converged=%d, iters=%3d, "
               "primal=%.6f, dual=%.6f, u0=[%.4f, %.4f]\n",
               iter_counts[t],
               sol.status == RICCATI_STATUS_OPTIMAL,
               sol.iterations,
               FP_TO_DOUBLE(sol.primal_residual),
               FP_TO_DOUBLE(sol.dual_residual),
               FP_TO_DOUBLE(sol.u[0][0]),
               FP_TO_DOUBLE(sol.u[0][1]));
    }

    /* Same with adaptive rho */
    printf("\n  With adaptive rho:\n");
    for (int t = 0; t < n_tests; t++) {
        RiccatiAdmmConfig_t cfg;
        riccati_admm_config_init(&cfg);
        cfg.tolerance = FP_CONST(0.001);
        cfg.max_iterations = iter_counts[t];
        cfg.adaptive_rho = 1;

        RiccatiAdmmState_t admm_state;
        riccati_admm_state_init(&admm_state);

        RiccatiSolution_t sol;
        memset(&sol, 0, sizeof(sol));

        riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                           nx, nu, N, &cfg, &admm_state, &sol);

        printf("  max_iter=%3d → converged=%d, iters=%3d, "
               "primal=%.6f, dual=%.6f, u0=[%.4f, %.4f]\n",
               iter_counts[t],
               sol.status == RICCATI_STATUS_OPTIMAL,
               sol.iterations,
               FP_TO_DOUBLE(sol.primal_residual),
               FP_TO_DOUBLE(sol.dual_residual),
               FP_TO_DOUBLE(sol.u[0][0]),
               FP_TO_DOUBLE(sol.u[0][1]));
    }
}

int main(void)
{
    printf("================================================================\n");
    printf("  Riccati-ADMM Convergence Diagnostic\n");
    printf("================================================================\n");

    test_precision_limits();
    test_raw_curve_convergence();
    test_convergence_over_iterations();
    test_mpc_curve_detailed();

    printf("\n================================================================\n");
    return 0;
}
