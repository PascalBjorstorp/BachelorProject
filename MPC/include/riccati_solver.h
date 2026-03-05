/**
 * @file riccati_solver.h
 * @brief Riccati-ADMM Solver for Non-Condensed MPC (Fixed-Point)
 *
 * Solves the constrained linear-quadratic optimal control problem:
 *
 *   min  Σ_{k=0}^{N-1} [l_k(x_k, u_k)] + l_N(x_N)
 *   s.t. x_{k+1} = A_k x_k + B_k u_k
 *        x_lb_k <= x_k <= x_ub_k    (state box constraints)
 *        u_lb_k <= u_k <= u_ub_k    (control box constraints)
 *
 * Using ADMM to handle constraints, with Riccati recursion for the
 * unconstrained LQR sub-problem (O(N) per ADMM iteration).
 *
 * Supports cross-cost term x^T N u for rate-penalty formulations
 * where the state is augmented with previous control inputs.
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#ifndef RICCATI_SOLVER_H
#define RICCATI_SOLVER_H

#include "fp_math.h"
#include <stdint.h>

/*===========================================================================
 * Dimension Limits
 *===========================================================================*/

/** Maximum state dimension (5 Frenet + δ_actual servo state + 2 previous controls) */
#define RICCATI_MAX_NX  8

/** Maximum control dimension */
#define RICCATI_MAX_NU  2

/** Maximum prediction horizon */
#define RICCATI_MAX_HORIZON  20

/*===========================================================================
 * Per-Step Data
 *===========================================================================*/

/**
 * Dynamics and cost data for a single prediction step.
 *
 * Stage cost: l_k = 0.5 x^T Q x + q^T x + 0.5 u^T R u + r^T u + x^T N u
 * Dynamics:   x_{k+1} = A x_k + B u_k
 *
 * Q and R are stored as diagonals (our MPC uses diagonal weight matrices).
 * N is the state-control cross-cost (sparse, for rate penalty augmentation).
 */
typedef struct
{
    /** State transition matrix A (nx × nx), row-major */
    fixed_point_t A[RICCATI_MAX_NX][RICCATI_MAX_NX];

    /** Control input matrix B (nx × nu), row-major */
    fixed_point_t B[RICCATI_MAX_NX][RICCATI_MAX_NU];

    /** Diagonal state cost weights (Q is diagonal) */
    fixed_point_t Q_diag[RICCATI_MAX_NX];

    /** Linear state cost (tracking error contribution) */
    fixed_point_t q[RICCATI_MAX_NX];

    /** Diagonal control cost weights */
    fixed_point_t R_diag[RICCATI_MAX_NU];

    /** Linear control cost */
    fixed_point_t r[RICCATI_MAX_NU];

    /** State-control cross-cost N (nx × nu).
     *  Used for rate penalty via augmented state formulation.
     *  N[i][a] contributes x[i]*N[i][a]*u[a] to the stage cost.
     *  Set to zero if no cross-cost. */
    fixed_point_t N[RICCATI_MAX_NX][RICCATI_MAX_NU];

    /** State box constraint lower bounds.
     *  Set to large negative for unconstrained states. */
    fixed_point_t x_lb[RICCATI_MAX_NX];

    /** State box constraint upper bounds.
     *  Set to large positive for unconstrained states. */
    fixed_point_t x_ub[RICCATI_MAX_NX];

    /** Control box constraint lower bounds */
    fixed_point_t u_lb[RICCATI_MAX_NU];

    /** Control box constraint upper bounds */
    fixed_point_t u_ub[RICCATI_MAX_NU];

} RiccatiStepData_t;

/*===========================================================================
 * Solver Configuration
 *===========================================================================*/

typedef struct
{
    /** ADMM penalty parameter rho for STATE constraints.
     *  Lower rho_x is fine since most states are unconstrained;
     *  only e_y has wall constraints and violations are small. */
    fixed_point_t rho;

    /** ADMM penalty parameter rho for CONTROL constraints.
     *  Should be much higher than rho when controls saturate heavily
     *  (e.g., curve scenarios where all 20 steering commands hit limits).
     *  High rho_u forces the Riccati pass to produce near-feasible controls.
     *  Set to 0 to use the same rho as states (backwards compatible). */
    fixed_point_t rho_u;

    /** Convergence tolerance (infinity-norm of primal/dual residuals) */
    fixed_point_t tolerance;

    /** Maximum ADMM iterations */
    int max_iterations;

    /** Enable adaptive rho scaling (1=enabled, 0=fixed rho).
     *  When enabled, rho is doubled if primal_res > 10*dual_res
     *  and halved if dual_res > 10*primal_res. This dramatically
     *  improves convergence for poorly-scaled problems. */
    int adaptive_rho;

    /** Over-relaxation parameter alpha ∈ (1.0, 2.0).
     *  The z-update uses x̂ = α*x + (1-α)*z_old instead of x.
     *  Default 1.6 provides ~30-40% iteration reduction.
     *  Set to FP_ONE (1.0) to disable over-relaxation. */
    fixed_point_t alpha;

} RiccatiAdmmConfig_t;

/*===========================================================================
 * Solver Status
 *===========================================================================*/

typedef enum
{
    RICCATI_STATUS_OPTIMAL = 0,
    RICCATI_STATUS_MAX_ITERATIONS = 1,
    RICCATI_STATUS_ERROR = 2
} RiccatiStatus_t;

/*===========================================================================
 * Solution Structure
 *===========================================================================*/

typedef struct
{
    /** Optimal state trajectory x[0..N] */
    fixed_point_t x[RICCATI_MAX_HORIZON + 1][RICCATI_MAX_NX];

    /** Optimal control trajectory u[0..N-1] */
    fixed_point_t u[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];

    /** Number of ADMM iterations performed */
    int iterations;

    /** Final primal residual (infinity norm) */
    fixed_point_t primal_residual;

    /** Final dual residual (infinity norm) */
    fixed_point_t dual_residual;

    /** Solver status */
    RiccatiStatus_t status;

} RiccatiSolution_t;

/*===========================================================================
 * ADMM Warm-Start State
 *===========================================================================*/

typedef struct
{
    /** ADMM slack variables for states z_x[0..N] */
    fixed_point_t z_x[RICCATI_MAX_HORIZON + 1][RICCATI_MAX_NX];

    /** ADMM slack variables for controls z_u[0..N-1] */
    fixed_point_t z_u[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];

    /** ADMM dual variables for states y_x[0..N] */
    fixed_point_t y_x[RICCATI_MAX_HORIZON + 1][RICCATI_MAX_NX];

    /** ADMM dual variables for controls y_u[0..N-1] */
    fixed_point_t y_u[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];

    /** Whether warm-start data is valid */
    int initialized;

} RiccatiAdmmState_t;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * Initialize Riccati-ADMM configuration with default values.
 * Default: rho=10.0, tolerance=0.01, max_iterations=30
 */
void riccati_admm_config_init(RiccatiAdmmConfig_t *config);

/**
 * Initialize ADMM state (clear warm-start data).
 */
void riccati_admm_state_init(RiccatiAdmmState_t *state);

/**
 * Solve constrained LQR using Riccati-ADMM.
 *
 * Each ADMM iteration consists of:
 * 1. Riccati backward pass: compute gains K_k, k_k using
 *    augmented costs (Q + rho*I, R + rho*I) and ADMM offsets
 * 2. Riccati forward pass: roll out x_k, u_k from x0
 * 3. z-update: project (x + y, u + y) onto box constraints
 * 4. y-update: dual variable update
 * 5. Convergence check on primal/dual residuals
 *
 * Complexity: O(N × nx³) per iteration (dominated by 5×5 matrix ops)
 *
 * @param step_data   Per-step dynamics and cost data (array of N elements)
 * @param terminal_Q  Terminal state cost (diagonal weights, length nx)
 * @param terminal_q  Terminal state linear cost (length nx)
 * @param x0          Initial state (length nx)
 * @param nx          State dimension (≤ RICCATI_MAX_NX)
 * @param nu          Control dimension (≤ RICCATI_MAX_NU)
 * @param horizon     Prediction steps N (≤ RICCATI_MAX_HORIZON)
 * @param config      ADMM configuration
 * @param admm_state  ADMM warm-start state (modified in-place)
 * @param solution    Output: optimal trajectories and status
 * @return Status code
 */
RiccatiStatus_t riccati_admm_solve(
    const RiccatiStepData_t *step_data,
    const fixed_point_t *terminal_Q,
    const fixed_point_t *terminal_q,
    const fixed_point_t *x0,
    int nx, int nu, int horizon,
    const RiccatiAdmmConfig_t *config,
    RiccatiAdmmState_t *admm_state,
    RiccatiSolution_t *solution);

/** Debug flag: set to 1 to print ADMM iteration details */
extern int riccati_admm_debug;

#endif /* RICCATI_SOLVER_H */
