/**
 * @file qp_solver_mpcc.h
 * @brief ADMM + Riccati QP Solver for MPCC (Lifted ODE)
 *
 * Solves the structured Quadratic Program arising from the Lifted ODE MPCC:
 *
 *   min  Sum_{k=0}^{N-1} [ 0.5 x_k^T Q_k x_k + q_k^T x_k
 *                         + 0.5 u_k^T R_k u_k + r_k^T u_k ]
 *      + 0.5 x_N^T Q_N x_N + q_N^T x_N
 *
 *   s.t. x_{k+1} = A_k x_k + B_k u_k + d_k    (dynamics)
 *        x_lb <= x_k <= x_ub                    (state bounds)
 *        u_lb <= u_k <= u_ub                    (control bounds)
 *        e_c(X_k, Y_k) within track corridor     (Cartesian projection)
 *        x_0 = x_init                          (initial condition)
 *
 * Track corridor constraints are enforced in the ADMM projection step by
 * projecting Cartesian (X, Y) into the local path frame at each stage,
 * clamping contouring error e_c, and reconstructing Cartesian position.
 *
 * ADMM (Alternating Direction Method of Multipliers):
 *   1. z-step: Equality-constrained QP (dynamics only)
 *      Solved by Riccati recursion (exploits multistage structure)
 *
 *   2. w-step: Box-constrained proximal step
 *      w = clamp(z + lambda, lb, ub)
 *      Per-stage track corridor, speed, steering-rate, and friction
 *      bounds are applied here.
 *
 *   3. Dual update:
 *      lambda += z - w
 *
 * Riccati recursion (for z-step):
 *   Backward pass (k = N-1 -> 0):
 *     G_k = R_tilde_k + B_k^T P_{k+1} B_k        [NU x NU = 3x3]
 *     H_k = B_k^T P_{k+1} A_k + S_k              [NU x NX]
 *     K_k = -G_k^{-1} H_k                         [NU x NX]
 *     kk_k = -G_k^{-1} (r_tilde_k + B_k^T s_{k+1}) [NU]
 *     P_k = Q_tilde_k + A_k^T P_{k+1} A_k + H_k^T K_k
 *     p_k = q_tilde_k + A_k^T s_{k+1} + H_k^T kk_k
 *
 *   Forward pass (k = 0 -> N-1):
 *     x_0 = x_init
 *     u_k = K_k x_k + kk_k
 *     x_{k+1} = A_k x_k + B_k u_k + d_k
 *
 * Matrix dimensions:
 *   NX = MPCC_NX = 7  (s, vx, vy, omega, X, Y, psi)
 *   NU = MPCC_NU = 3  (delta, a_x, v_theta)
 *   G_k inversion is 3x3 -> analytic formula (small fixed matrix)
 *
 * Complexity: O(N * (NX + NU)^3) per ADMM iteration
 * Memory: O(N * (NX^2 + NX*NU)) — all statically allocated
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#ifndef QP_SOLVER_MPCC_H
#define QP_SOLVER_MPCC_H

#include "mpcc_types.h"

/*===========================================================================
 * QP Problem Definition (Multistage Sparse Form)
 *===========================================================================*/

typedef struct
{
    /*--- Dynamics: x_{k+1} = A_k x_k + B_k u_k + d_k ---*/

    /** Linearized dynamics at each stage (k = 0..N-1) */
    MPCCLinearSystem_t dynamics[MPCC_MAX_HORIZON];

    /*--- Stage costs ---*/

    /** Running cost matrices (k = 0..N-1) */
    MPCCStageCost_t stage_cost[MPCC_MAX_HORIZON];

    /** Terminal cost (k = N) — only Q and q fields used */
    MPCCStageCost_t terminal_cost;

    /*--- Global box constraints ---*/

    /** State lower bounds (default, applied at all stages) */
    float x_lower[MPCC_NX];

    /** State upper bounds (default, applied at all stages) */
    float x_upper[MPCC_NX];

    /** Control lower bounds */
    float u_lower[MPCC_NU];

    /** Control upper bounds */
    float u_upper[MPCC_NU];

    /** Per-stage steering bounds [rad].
     *  Tighten the global steering box with a physically reachable envelope
     *  from the currently applied steering angle. */
    float delta_lower_stage[MPCC_MAX_HORIZON];
    float delta_upper_stage[MPCC_MAX_HORIZON];

    /** Arc-length operating point used to linearize each stage. */
    float s_ref_stage[MPCC_MAX_HORIZON + 1];

    /** Per-stage trust-region bounds on s around s_ref_stage. */
    float s_lower_stage[MPCC_MAX_HORIZON + 1];
    float s_upper_stage[MPCC_MAX_HORIZON + 1];

    /*--- Per-stage track bounds in contouring-error coordinates ---*/

    /** Left track boundary at each stage [m] (positive value).
     *  The MPCC contouring error is positive to the right, so the hard
     *  lower bound is e_c >= -track_left[k]. */
    float track_left[MPCC_MAX_HORIZON + 1];

    /** Right track boundary at each stage [m] (positive value).
     *  The hard upper bound is e_c <= track_right[k]. */
    float track_right[MPCC_MAX_HORIZON + 1];

        /** Reference path geometry at each stage — needed to project
     *  Cartesian (X,Y) into the Frenet frame and back.
     *  Populated by build_qp_problem() from mpcc_path_interpolate(). */
    float path_x_ref  [MPCC_MAX_HORIZON + 1];   /* x_ref(s_k)   [m]   */
    float path_y_ref  [MPCC_MAX_HORIZON + 1];   /* y_ref(s_k)   [m]   */
    float path_phi_ref[MPCC_MAX_HORIZON + 1];   /* phi_ref(s_k) [rad] */

    /*--- Friction circle ---*/

    /** Squared friction limit (mu * g)^2 for combined acceleration constraint.
     *  When > 0, per-stage a_x bounds are tightened based on operating point
     *  lateral acceleration: ax_max_k = sqrt(mu_g_sq - ay_k^2). */
    float mu_g_sq;

    /** Per-stage upper bound on |a_x|, tightened by friction circle.
     *  Computed from operating point lateral acceleration at each stage. */
    float ax_lim_stage[MPCC_MAX_HORIZON];

    /** Per-stage upper bound on vx [m/s].
     *  Computed from lookahead curvature to enforce braking before curves.
     *  When > 0, tightens the global vx_max at each stage. */
    float vx_max_stage[MPCC_MAX_HORIZON + 1];

    /*--- Problem size ---*/

    /** Prediction horizon length (0 < N <= MPCC_MAX_HORIZON) */
    uint16_t N;

    /*--- Initial condition ---*/

    /** Fixed initial state x_0 (equality constraint) */
    float x0[MPCC_NX];

} MPCCQPProblem_t;

/*===========================================================================
 * ADMM Solver Configuration
 *===========================================================================*/

typedef struct
{
    /** ADMM penalty parameter (rho).
     *  Typical range: 0.1 to 10.0 */
    float rho;

    /** Maximum number of ADMM iterations (20-100 for real-time) */
    uint16_t max_iterations;

    /** Primal residual tolerance */
    float eps_primal;

    /** Dual residual tolerance */
    float eps_dual;

    /** Use warm start from previous ADMM solution */
    uint8_t warm_start;

    /** ADMM penalty parameter for CONTROL constraints (rho_u).
     *  Separate from state rho because controls may saturate heavily
     *  while states don't (or vice versa). Different penalties allow
     *  the solver to converge faster on both fronts.
     *  Set to 0 to fall back to using the same rho as states. */
    float rho_u;

    /** Enable adaptive rho scaling (1=enabled, 0=fixed rho).
     *  When enabled, state rho and control rho_u are updated independently:
     *  a penalty is multiplied or divided by 1.5 if its own residuals differ
     *  by more than a factor of five, checked every five iterations.
     *  Dual variables are rescaled to maintain ADMM consistency. */
    uint8_t adaptive_rho;

    /** Over-relaxation parameter alpha in (1.0, 2.0).
     *  Standard ADMM uses alpha=1.0. Values 1.5-1.7 typically reduce
     *  iteration count by 30-50%. Set to 1.0f to disable.
     *  FPGA-friendly: one extra multiply per variable per iteration. */
    float alpha_relax;

} ADMMConfig_t;

/*===========================================================================
 * ADMM Solver Workspace (pre-allocated static memory)
 *===========================================================================
 * All intermediate variables for the ADMM solver.
 * Statically sized for worst-case horizon — no dynamic allocation.
 *
 * Memory layout:
 *   z = (z_x[0], z_u[0], ..., z_x[N])  — primal (Riccati output)
 *   w = (w_x[0], w_u[0], ..., w_x[N])  — copy (projection output)
 *   lambda = (lam_x[0], lam_u[0], ..., lam_x[N]) — dual (scaled form)
 */

typedef struct
{
    /*--- ADMM primal variables (z): output of Riccati solve ---*/
    float z_x[MPCC_MAX_HORIZON + 1][MPCC_NX];
    float z_u[MPCC_MAX_HORIZON][MPCC_NU];

    /*--- ADMM copy variables (w): output of box projection ---*/
    float w_x[MPCC_MAX_HORIZON + 1][MPCC_NX];
    float w_u[MPCC_MAX_HORIZON][MPCC_NU];

    /*--- ADMM dual variables (lambda): scaled form (lambda/rho) ---*/
    float lambda_x[MPCC_MAX_HORIZON + 1][MPCC_NX];
    float lambda_u[MPCC_MAX_HORIZON][MPCC_NU];

    /*--- Riccati backward pass: cost-to-go ---*/
    /** P_k: value function Hessian (NX x NX) at each stage */
    float P[MPCC_MAX_HORIZON + 1][MPCC_NX][MPCC_NX];
    /** p_k: value function gradient (NX) at each stage */
    float p[MPCC_MAX_HORIZON + 1][MPCC_NX];

    /*--- Riccati gains ---*/
    /** K_k: feedback gain matrix (NU x NX) at each stage */
    float K[MPCC_MAX_HORIZON][MPCC_NU][MPCC_NX];
    /** kk_k: feedforward term (NU) at each stage */
    float kk[MPCC_MAX_HORIZON][MPCC_NU];

    /*--- Previous w for dual residual computation ---*/
    float w_x_prev[MPCC_MAX_HORIZON + 1][MPCC_NX];
    float w_u_prev[MPCC_MAX_HORIZON][MPCC_NU];

    /*--- Diagnostics ---*/
    float primal_residual;
    float dual_residual;
    uint16_t iterations;
    uint16_t adaptive_rho_updates;
    uint16_t adaptive_rho_state_updates;
    uint16_t adaptive_rho_control_updates;
    uint32_t numeric_clip_count;

    /* Persisted adaptive penalties for warm-started solves.  Fixed-rho
     * solves ignore these values and use the configured penalties. */
    float rho_state;
    float rho_u_state;

} ADMMWorkspace_t;

/*===========================================================================
 * ADMM Solver Result
 *===========================================================================*/

typedef struct
{
    /** Solver termination status */
    MPCCStatus_t status;

    /** Number of ADMM iterations performed */
    uint16_t iterations;

    /** Final primal residual ||z - w|| */
    float primal_residual;

    /** Final dual residual rho * ||w_new - w_old|| */
    float dual_residual;

    /** Final adapted rho values used by this solve */
    float rho_final;
    float rho_u_final;

    /** Number of adaptive rho updates during this solve */
    uint16_t adaptive_rho_updates;

    /** Adaptive update counts for the state and control penalties. */
    uint16_t adaptive_rho_state_updates;
    uint16_t adaptive_rho_control_updates;

    /** Count of int64->int32 clipping events during this solve */
    uint32_t numeric_clip_count;

    /** Optimal state trajectory */
    float x_opt[MPCC_MAX_HORIZON + 1][MPCC_NX];

    /** Optimal control sequence */
    float u_opt[MPCC_MAX_HORIZON][MPCC_NU];

} ADMMResult_t;

/*===========================================================================
 * Solver API
 *===========================================================================*/

/** Initialize the ADMM solver workspace to zero. */
void admm_solver_initialize(ADMMWorkspace_t *workspace);

/** Set ADMM configuration to default values. */
void admm_solver_default_config(ADMMConfig_t *config);

/**
 * Solve the structured QP using ADMM + Riccati.
 *
 * @param problem    QP problem definition (dynamics, costs, constraints)
 * @param config     ADMM solver parameters
 * @param workspace  Pre-allocated solver workspace (modified in-place)
 * @param result     Output: optimal solution and diagnostics
 * @return Solver status code
 */
MPCCStatus_t admm_solver_solve(
    const MPCCQPProblem_t *problem,
    const ADMMConfig_t *config,
    ADMMWorkspace_t *workspace,
    ADMMResult_t *result);

/*===========================================================================
 * Internal Functions (exposed for unit testing)
 *===========================================================================*/

/**
 * Riccati backward pass: compute P_k, p_k, K_k, kk_k for all stages.
 *
 * Uses ADMM-augmented costs with selective augmentation:
 *   For constrained state dimensions (|bound| < threshold):
 *     Q_tilde_k[s][s] = Q_k[s][s] + rho
 *     q_tilde_k[s]    = q_k[s] + rho*(lambda_x_k[s] - w_x_k[s])
 *   For unconstrained state dimensions: no augmentation.
 *
 *   For controls (always constrained):
 *     R_tilde_k[a][a] = R_k[a][a] + rho_u
 *     r_tilde_k[a]    = r_k[a] + rho_u*(lambda_u_k[a] - w_u_k[a])
 *
 * Uses int64 intermediates for P and p to avoid fixed-point truncation
 * accumulation across the backward sweep.
 *
 * G_k = R_tilde + B^T P B is 2x2 -> analytic inverse.
 */
void riccati_backward_pass(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *workspace,
    float rho,
    float rho_u);

/**
 * Riccati forward pass: compute optimal z_x, z_u.
 */
void riccati_forward_pass(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *workspace);

/**
 * ADMM w-step: project z + lambda onto box constraints.
 * Applies per-stage track bounds on n (state index 1).
 */
void admm_projection_step(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *workspace);

/**
 * ADMM dual update (scaled form).
 */
void admm_dual_update(
    ADMMWorkspace_t *workspace,
    uint16_t N);

/**
 * Compute ADMM convergence residuals (infinity norm).
 */
void admm_compute_residuals(
    const ADMMWorkspace_t *workspace,
    float rho,
    float rho_u,
    uint16_t N,
    float *primal_res,
    float *dual_res,
    /* Optional outputs: per-domain residuals (states / controls). */
    float *primal_x_res,
    float *dual_x_res,
    float *primal_u_res,
    float *dual_u_res);

/*===========================================================================
 * Small Matrix Helpers
 *===========================================================================*/

/**
 * Invert a NU x NU (2x2) symmetric positive definite matrix.
 *
 * For the 2x2 case, uses the analytic formula:
 *   det = M[0][0]*M[1][1] - M[0][1]*M[1][0]
 *   inv = (1/det) * [ M[1][1], -M[0][1]; -M[1][0], M[0][0] ]
 *
 * @param mat    Input matrix (NU x NU = 2x2)
 * @param inv    Output: inverse matrix (NU x NU = 2x2)
 * @return 0 on success, -1 if matrix is singular
 */
int mat_nu_inverse(
    const float mat[MPCC_NU][MPCC_NU],
    float inv[MPCC_NU][MPCC_NU]);

#endif /* QP_SOLVER_MPCC_H */
