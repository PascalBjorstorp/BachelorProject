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
 * All arithmetic uses native float operations.
 */

#ifndef RICCATI_SOLVER_H
#define RICCATI_SOLVER_H

#include "mpc_types.h"
#include "util_math.h"
#include <stdint.h>

/*===========================================================================
 * Dimension Limits
 *===========================================================================*/

/** Maximum state dimension (5 Frenet + δ_actual servo state + 2 previous controls) */
#define RICCATI_MAX_NX  8

/** Maximum control dimension */
#define RICCATI_MAX_NU  2

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
    float A[RICCATI_MAX_NX][RICCATI_MAX_NX];

    /** Control input matrix B (nx × nu), row-major */
    float B[RICCATI_MAX_NX][RICCATI_MAX_NU];

    /** Diagonal state cost weights (Q is diagonal) */
    float Q_diag[RICCATI_MAX_NX];

    /** Linear state cost (tracking error contribution) */
    float q[RICCATI_MAX_NX];

    /** Diagonal control cost weights */
    float R_diag[RICCATI_MAX_NU];

    /** Linear control cost */
    float r[RICCATI_MAX_NU];

    /** State-control cross-cost N (nx × nu).
     *  Used for rate penalty via augmented state formulation.
     *  N[i][a] contributes x[i]*N[i][a]*u[a] to the stage cost.
     *  Set to zero if no cross-cost. */
    float N[RICCATI_MAX_NX][RICCATI_MAX_NU];

    /** State box constraint lower bounds.
     *  Set to large negative for unconstrained states. */
    float x_lb[RICCATI_MAX_NX];

    /** State box constraint upper bounds.
     *  Set to large positive for unconstrained states. */
    float x_ub[RICCATI_MAX_NX];

    /** Control box constraint lower bounds */
    float u_lb[RICCATI_MAX_NU];

    /** Control box constraint upper bounds */
    float u_ub[RICCATI_MAX_NU];

    /** Soft constraint stiffness per state.
     *  0 = hard constraint (standard ADMM box projection).
     *  >0 = soft quadratic penalty: g(z) = (k/2)*max(0, z-ub)^2 + (k/2)*max(0, lb-z)^2
     *  The ADMM z-update uses the proximal operator instead of clipping:
     *    if v > ub: z = (k*ub + rho*v) / (k + rho)
     *    if v < lb: z = (k*lb + rho*v) / (k + rho)
     *  Higher k = stiffer (approaches hard constraint). Typical: 200-1000. */
    float x_soft_weight[RICCATI_MAX_NX];

} RiccatiStepData_t;

/*===========================================================================
 * Solver Configuration
 *===========================================================================*/

typedef struct
{
    /** ADMM penalty parameter rho for STATE constraints.
     *  Lower rho_x is fine since most states are unconstrained;
     *  only e_y has wall constraints and violations are small. */
    float rho;

    /** ADMM penalty parameter rho for CONTROL constraints.
     *  Should be much higher than rho when controls saturate heavily
     *  (e.g., curve scenarios where all 20 steering commands hit limits).
     *  High rho_u forces the Riccati pass to produce near-feasible controls.
     *  Set to 0 to use the same rho as states (backwards compatible). */
    float rho_u;

    /** Convergence tolerance (infinity-norm of primal/dual residuals) */
    float tolerance;

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
     *  Set to 1.0f (1.0) to disable over-relaxation. */
    float alpha;

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
    float x[MPC_PREDICTION_HORIZON + 1][RICCATI_MAX_NX];

    /** Optimal control trajectory u[0..N-1] */
    float u[MPC_PREDICTION_HORIZON][RICCATI_MAX_NU];

    /** Number of ADMM iterations performed */
    int iterations;

    /** Final primal residual (infinity norm) */
    float primal_residual;

    /** Final dual residual (infinity norm) */
    float dual_residual;

    /** Solver status */
    RiccatiStatus_t status;

} RiccatiSolution_t;

/*===========================================================================
 * ADMM Warm-Start State
 *===========================================================================*/

typedef struct
{
    /** ADMM slack variables for states z_x[0..N] */
    float z_x[MPC_PREDICTION_HORIZON + 1][RICCATI_MAX_NX];

    /** ADMM slack variables for controls z_u[0..N-1] */
    float z_u[MPC_PREDICTION_HORIZON][RICCATI_MAX_NU];

    /** ADMM dual variables for states y_x[0..N] */
    float y_x[MPC_PREDICTION_HORIZON + 1][RICCATI_MAX_NX];

    /** ADMM dual variables for controls y_u[0..N-1] */
    float y_u[MPC_PREDICTION_HORIZON][RICCATI_MAX_NU];

    /** Persisted adaptive rho (0 = use config default) */
    float rho;

    /** Persisted adaptive rho_u (0 = use config default) */
    float rho_u;

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
 * @param horizon     Prediction steps N (≤ MPC_PREDICTION_HORIZON)
 * @param config      ADMM configuration
 * @param admm_state  ADMM warm-start state (modified in-place)
 * @param solution    Output: optimal trajectories and status
 * @return Status code
 */
RiccatiStatus_t riccati_admm_solve(
    const RiccatiStepData_t *step_data,
    const float *terminal_Q,
    const float *terminal_q,
    const float *x0,
    int nx, int nu, int horizon,
    const RiccatiAdmmConfig_t *config,
    RiccatiAdmmState_t *admm_state,
    RiccatiSolution_t *solution);

/** Debug flag: set to 1 to print ADMM iteration details */
extern int riccati_admm_debug;

#endif /* RICCATI_SOLVER_H */
