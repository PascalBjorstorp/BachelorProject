/**
 * @file mpc.c
 * @brief Model Predictive Control Implementation
 *
 * Implements the MPC controller for F1/10th autonomous vehicle.
 * Uses quadratic programming to optimize control inputs over a
 * prediction horizon while respecting vehicle constraints.
 *
 * Algorithm Overview:
 * 1. Linearize vehicle model around current state
 * 2. Build QP cost matrices from tracking error and control effort
 * 3. Build QP constraint matrices from actuator limits
 * 4. Solve QP using projected gradient descent
 * 5. Extract first control input from optimal sequence
 *
 * Mathematical Formulation:
 * 
 *   State vector:   x = [x_pos, y_pos, heading, velocity]^T
 *   Control vector: u = [steering, velocity_cmd]^T
 *   
 *   Discrete-time model: x_{k+1} = f(x_k, u_k)
 *   Linearized:          x_{k+1} = A * x_k + B * u_k
 *
 *   QP Problem:
 *     minimize    0.5 * u^T * H * u + f^T * u
 *     subject to  A_ineq * u <= b_ineq
 *
 *   where u = [u_0, u_1, ..., u_{N-1}]^T (stacked controls)
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#include "mpc.h"
#include "qp_solver.h"
#include "fp_math.h"
#include "vehicle_model.h"
#include <string.h>

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/** Number of states in vehicle model: [x, y, heading, velocity] */
#define STATE_DIM 4

/** Number of control inputs: [steering, velocity_cmd] */
#define CONTROL_DIM 2

/** Maximum prediction horizon supported */
#define MAX_HORIZON 20

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

/** Current MPC configuration */
static MpcConfig_t g_config;

/** Flag indicating MPC has been initialized */
static int g_initialized = 0;

/** Previous control input (for rate limiting) */
static ControlInput_t g_prev_control;

/*===========================================================================
 * Default Configuration
 *===========================================================================*/

static MpcConfig_t get_default_config(void)
{
    MpcConfig_t cfg;

    /* Prediction horizon and timing */
    cfg.horizon = MPC_DEFAULT_HORIZON;
    cfg.dt = MPC_DEFAULT_DT;

    /*
     * State tracking weights Q = diag(w_x, w_y, w_heading, w_vel)
     * Higher weight = penalize deviation more strongly
     * 
     * Position weights (x, y): 10.0
     *   - Strong position tracking for path following
     * 
     * Heading weight: 5.0
     *   - Moderate heading tracking, allows some slip
     *
     * Velocity weight: 2.0
     *   - Soft velocity tracking, prioritizes position
     */
    cfg.w_x = FP_CONST(10.0);
    cfg.w_y = FP_CONST(10.0);
    cfg.w_heading = FP_CONST(5.0);
    cfg.w_vel = FP_CONST(2.0);

    /*
     * Control effort weights R = diag(w_steer, w_vel_cmd)
     * Penalizes large control inputs
     *
     * Steering effort: 1.0
     *   - Moderate penalty on steering magnitude
     *
     * Velocity effort: 0.5
     *   - Lower penalty, allows aggressive speed changes
     */
    cfg.w_steer = FP_CONST(10.0);
    cfg.w_vel_cmd = FP_CONST(2.0);

    /*
     * Control rate weights (smoothness)
     * Penalizes: sum_k ||u_k - u_{k-1}||^2
     *
     * Steering rate: 10.0
     *   - High penalty prevents steering oscillation
     *
     * Velocity rate: 5.0
     *   - Moderate penalty for smooth acceleration
     */
    cfg.w_steer_rate = FP_CONST(2.0);
    cfg.w_vel_rate = FP_CONST(1.0);

    /* Solver parameters */
    cfg.max_iter = MPC_DEFAULT_MAX_ITER;
    cfg.tolerance = MPC_DEFAULT_TOLERANCE;

    return cfg;
}

/*===========================================================================
 * QP Problem Construction
 *===========================================================================*/

/**
 * Build the QP Hessian matrix from cost weights.
 *
 * QP form: minimize 0.5 * u^T * H * u + f^T * u
 *
 * The Hessian H is composed of:
 *   1. Control effort: R = diag(w_steer, w_vel)
 *   2. Rate penalty:   Delta^T * W_rate * Delta
 *      where Delta computes differences u_k - u_{k-1}
 *
 * For N-step horizon with 2 controls per step:
 *
 *   H = [ R + W_rate   -W_rate      0    ...   0     ]
 *       [ -W_rate    R + 2*W_rate  -W_rate ...   0   ]
 *       [   0         -W_rate    R + 2*W_rate ... 0  ]
 *       [  ...          ...        ...   ...  ...    ]
 *       [   0            0          0   ...  R+W_rate]
 *
 * Note: QP convention uses 0.5 * u^T * H * u, so H entries are 2x
 * the actual weight to cancel the 0.5 factor.
 *
 * @param horizon Number of prediction steps
 * @param H Output Hessian matrix (row-major, n*n where n = horizon*2)
 */
static void build_hessian(int horizon, fixed_point_t *H)
{
    int n = horizon * CONTROL_DIM;
    memset(H, 0, n * n * sizeof(fixed_point_t));

    fixed_point_t two = FP_TWO;
    fixed_point_t w_steer_eff = g_config.w_steer;
    fixed_point_t w_vel_eff = g_config.w_vel_cmd;
    fixed_point_t w_steer_rate = g_config.w_steer_rate;
    fixed_point_t w_vel_rate = g_config.w_vel_rate;

    for (int k = 0; k < horizon; k++)
    {
        int base = k * CONTROL_DIM;
        int steer_diag = base * n + base;
        int vel_diag = (base + 1) * n + (base + 1);

        /*
         * Diagonal entries:
         *   H[k,k] = 2 * (w_effort + rate_contribution)
         *
         * Rate contribution:
         *   - First/middle steps: 2 * w_rate (appears in two rate terms)
         *   - Last step: w_rate (only one rate term)
         */
        fixed_point_t steer_rate_diag, vel_rate_diag;

        if (k < horizon - 1)
        {
            /* Interior: u_k appears in (u_k - u_{k-1})^2 AND (u_{k+1} - u_k)^2 */
            steer_rate_diag = fp_mul(two, w_steer_rate);
            vel_rate_diag = fp_mul(two, w_vel_rate);
        }
        else
        {
            /* Last step: only in (u_{N-1} - u_{N-2})^2 */
            steer_rate_diag = w_steer_rate;
            vel_rate_diag = w_vel_rate;
        }

        /* H[k,k] = 2 * (w_effort + rate_diag) */
        H[steer_diag] = fp_mul(two, fp_add(w_steer_eff, steer_rate_diag));
        H[vel_diag] = fp_mul(two, fp_add(w_vel_eff, vel_rate_diag));

        /*
         * Off-diagonal entries (rate coupling):
         *   H[k-1,k] = H[k,k-1] = -2 * w_rate
         *
         * From expanding (u_k - u_{k-1})^2:
         *   = u_k^2 - 2*u_k*u_{k-1} + u_{k-1}^2
         *   Cross term: -2*u_k*u_{k-1} contributes -2*w_rate to H[k-1,k]
         */
        if (k > 0)
        {
            int prev_base = (k - 1) * CONTROL_DIM;
            fixed_point_t neg_2_steer = fp_neg(fp_mul(two, w_steer_rate));
            fixed_point_t neg_2_vel = fp_neg(fp_mul(two, w_vel_rate));

            /* Symmetric off-diagonal */
            int cross_s1 = prev_base * n + base;
            int cross_s2 = base * n + prev_base;
            H[cross_s1] = neg_2_steer;
            H[cross_s2] = neg_2_steer;

            int cross_v1 = (prev_base + 1) * n + (base + 1);
            int cross_v2 = (base + 1) * n + (prev_base + 1);
            H[cross_v1] = neg_2_vel;
            H[cross_v2] = neg_2_vel;
        }
    }
}

/**
 * Build the QP linear cost vector from reference trajectory.
 *
 * QP form: minimize 0.5 * u^T * H * u + f^T * u
 *
 * The linear term f comes from:
 *   1. Reference tracking: f_track = -2 * R * u_ref
 *      where u_ref is feedforward control to reach reference
 *
 *   2. Rate penalty with previous control:
 *      f_rate[0] = -2 * w_rate * u_prev
 *      (first control penalized for deviating from previous)
 *
 * Feedforward control computation:
 *   - Steering: proportional to heading error
 *     u_steer_ff = K_heading * (ref_heading - current_heading)
 *
 *   - Velocity: directly from reference velocity
 *     u_vel_ff = ref_velocity
 *
 * @param state Current vehicle state
 * @param ref Reference trajectory (array of length horizon)
 * @param horizon Number of prediction steps
 * @param f Output linear cost vector (length horizon*2)
 */
static void build_linear_cost(
    const VehicleState_t *state,
    const TrajectoryPoint_t *ref,
    int horizon,
    fixed_point_t *f)
{
    int n = horizon * CONTROL_DIM;
    memset(f, 0, n * sizeof(fixed_point_t));

    /*
     * For each timestep, compute feedforward control and
     * add weighted contribution to linear cost
     */
    for (int k = 0; k < horizon; k++)
    {
        int base = k * CONTROL_DIM;

        /*
         * Heading error -> feedforward steering
         * 
         * CRITICAL: Handle angle wrapping!
         * When heading crosses ±π, naive subtraction gives huge errors.
         * Example: ref=3.06, state=-3.03 → naive_err=6.09, correct_err=-0.11
         * 
         * Normalize heading error to [-π, +π]:
         */
        fixed_point_t heading_err = fp_sub(ref[k].heading, state->heading);
        
        /* Wrap to [-π, π] */
        while (heading_err > FP_PI)
            heading_err = fp_sub(heading_err, FP_TWO_PI);
        while (heading_err < fp_neg(FP_PI))
            heading_err = fp_add(heading_err, FP_TWO_PI);
        
        fixed_point_t ff_steer = heading_err;

        /* Reference velocity directly as feedforward */
        fixed_point_t ff_vel = ref[k].vel;

        /*
         * Linear cost = -2 * w * u_ref
         * Negative because we minimize: 0.5*u^T*H*u + f^T*u
         * Setting f = -2*w*u_ref makes the optimum near u_ref
         */
        f[base] = fp_neg(fp_mul(FP_TWO, fp_mul(g_config.w_steer, ff_steer)));
        f[base + 1] = fp_neg(fp_mul(FP_TWO, fp_mul(g_config.w_vel_cmd, ff_vel)));
    }

    /*
     * Rate penalty: penalize first control for deviating from previous
     * Adds term: w_rate * (u_0 - u_prev)^2
     * Linear part: -2 * w_rate * u_prev
     */
    f[0] = fp_sub(f[0], fp_mul(FP_TWO, fp_mul(g_config.w_steer_rate, g_prev_control.steer)));
    f[1] = fp_sub(f[1], fp_mul(FP_TWO, fp_mul(g_config.w_vel_rate, g_prev_control.vel)));
}

/**
 * Build QP inequality constraints for actuator limits.
 *
 * Constraint form: A * u <= b
 *
 * For each timestep k, we have box constraints:
 *   steer_k <= max_steer         (upper steering limit)
 *   -steer_k <= max_steer        (lower steering limit: steer >= -max)
 *   vel_k <= max_vel             (upper velocity limit)
 *   -vel_k <= -min_vel           (lower velocity limit: vel >= min)
 *
 * For k > 0, we also have rate constraints:
 *   steer_k - steer_{k-1} <= max_steer_vel * dt    (steering rate upper)
 *   steer_{k-1} - steer_k <= max_steer_vel * dt    (steering rate lower)
 *   vel_k - vel_{k-1} <= max_accel * dt            (accel upper)
 *   vel_{k-1} - vel_k <= max_accel * dt            (decel upper)
 *
 * Total constraints:
 *   4 box constraints per timestep (4 * horizon)
 *   4 rate constraints per timestep except first ((horizon-1) * 4)
 *   Total = 4*horizon + 4*(horizon-1) = 8*horizon - 4
 *
 * @param horizon Number of prediction steps
 * @param A Output constraint matrix (m rows, n cols)
 * @param b Output constraint bounds (m entries)
 */
static void build_constraints(int horizon, fixed_point_t *A, fixed_point_t *b)
{
    int n = horizon * CONTROL_DIM;
    /* 4 box constraints per step + 4 rate constraints for steps 1..horizon-1 */
    int m_box = horizon * 4;
    int m_rate = (horizon - 1) * 4;
    int m = m_box + m_rate;

    memset(A, 0, m * n * sizeof(fixed_point_t));
    memset(b, 0, m * sizeof(fixed_point_t));

    VehicleParams_t params = vehicle_model_get_params();
    
    /* Compute max rate changes per timestep */
    fixed_point_t max_steer_change = fp_mul(params.max_steer_vel, g_config.dt);
    fixed_point_t max_vel_change = fp_mul(params.max_accel, g_config.dt);

    /* ========== BOX CONSTRAINTS (4 per timestep) ========== */
    for (int k = 0; k < horizon; k++)
    {
        int ctrl = k * CONTROL_DIM;
        int cons = k * 4;

        /* Constraint 0: steer <= max_steer */
        A[(cons + 0) * n + ctrl] = FP_ONE;
        b[cons + 0] = params.max_steer;

        /* Constraint 1: -steer <= max_steer (i.e., steer >= -max_steer) */
        A[(cons + 1) * n + ctrl] = fp_neg(FP_ONE);
        b[cons + 1] = params.max_steer;

        /* Constraint 2: vel <= max_vel */
        A[(cons + 2) * n + (ctrl + 1)] = FP_ONE;
        b[cons + 2] = params.max_vel;

        /* Constraint 3: -vel <= -min_vel (i.e., vel >= min_vel) */
        A[(cons + 3) * n + (ctrl + 1)] = fp_neg(FP_ONE);
        b[cons + 3] = fp_neg(params.min_vel);
    }

    /* ========== RATE CONSTRAINTS (4 per timestep, starting from k=1) ========== */
    for (int k = 1; k < horizon; k++)
    {
        int ctrl_curr = k * CONTROL_DIM;
        int ctrl_prev = (k - 1) * CONTROL_DIM;
        int cons = m_box + (k - 1) * 4;  /* Offset past box constraints */

        /*
         * Constraint: steer_k - steer_{k-1} <= max_steer_change
         * Row: [0 ... 0 -1 0 ... 0 +1 0 ... 0]
         *              ^prev     ^curr
         */
        A[(cons + 0) * n + ctrl_curr] = FP_ONE;
        A[(cons + 0) * n + ctrl_prev] = fp_neg(FP_ONE);
        b[cons + 0] = max_steer_change;

        /*
         * Constraint: steer_{k-1} - steer_k <= max_steer_change
         * Row: [0 ... 0 +1 0 ... 0 -1 0 ... 0]
         */
        A[(cons + 1) * n + ctrl_prev] = FP_ONE;
        A[(cons + 1) * n + ctrl_curr] = fp_neg(FP_ONE);
        b[cons + 1] = max_steer_change;

        /*
         * Constraint: vel_k - vel_{k-1} <= max_vel_change (acceleration limit)
         */
        A[(cons + 2) * n + (ctrl_curr + 1)] = FP_ONE;
        A[(cons + 2) * n + (ctrl_prev + 1)] = fp_neg(FP_ONE);
        b[cons + 2] = max_vel_change;

        /*
         * Constraint: vel_{k-1} - vel_k <= max_vel_change (deceleration limit)
         */
        A[(cons + 3) * n + (ctrl_prev + 1)] = FP_ONE;
        A[(cons + 3) * n + (ctrl_curr + 1)] = fp_neg(FP_ONE);
        b[cons + 3] = max_vel_change;
    }
}

/*===========================================================================
 * Public API Implementation
 *===========================================================================*/

void mpc_init(void)
{
    vehicle_model_init();
    g_config = get_default_config();
    g_prev_control.steer = 0;
    g_prev_control.vel = 0;
    g_initialized = 1;
}

void mpc_init_config(const MpcConfig_t *config)
{
    vehicle_model_init();
    if (config != NULL)
    {
        g_config = *config;
    }
    else
    {
        g_config = get_default_config();
    }
    g_prev_control.steer = 0;
    g_prev_control.vel = 0;
    g_initialized = 1;
}

/**
 * Compute optimal control for current state and reference trajectory.
 *
 * Algorithm:
 * 1. Build QP matrices (H, f, A, b) from config and reference
 * 2. Solve QP using projected gradient descent
 * 3. Extract first control from optimal sequence
 * 4. Saturate to vehicle limits
 * 5. Store for rate limiting in next call
 *
 * @param state Current vehicle state [x, y, heading, vel]
 * @param trajectory Reference trajectory (horizon points)
 * @param result Output: optimal control and solver status
 * @return Solver status
 */
SolverStatus_t mpc_compute(
    const VehicleState_t *state,
    const TrajectoryPoint_t *trajectory,
    MpcResult_t *result)
{
    /* Validate inputs */
    if (!g_initialized || state == NULL || trajectory == NULL || result == NULL)
    {
        if (result != NULL)
        {
            result->status = SOLVER_ERROR;
            result->iterations = 0;
        }
        return SOLVER_ERROR;
    }

    int horizon = g_config.horizon;
    if (horizon > MAX_HORIZON) horizon = MAX_HORIZON;

    int n_vars = horizon * CONTROL_DIM;
    /* 4 box constraints per step + 4 rate constraints for steps 1..horizon-1 */
    int n_cons = (horizon * 4) + ((horizon - 1) * 4);

    /* Build QP problem (static to persist across calls, avoid stack) */
    static QpProblem_t qp;
    qp_init_problem(&qp);
    qp.n_vars = n_vars;
    qp.n_constraints = n_cons;

    build_hessian(horizon, qp.H);
    build_linear_cost(state, trajectory, horizon, qp.f);
    build_constraints(horizon, qp.A, qp.b);

    /* Configure solver */
    QpConfig_t qp_cfg;
    qp_init_config(&qp_cfg);
    qp_cfg.max_iter = g_config.max_iter;
    qp_cfg.tolerance = g_config.tolerance;
    qp_cfg.step_size = FP_CONST(0.01);

    /* Solve QP */
    static QpSolution_t qp_sol;
    SolverStatus_t status = qp_solve(&qp, &qp_cfg, &qp_sol);

    /* Extract first control from optimal sequence */
    result->status = status;
    result->iterations = qp_sol.iterations;
    result->cost = qp_sol.residual;
    result->control.steer = qp_sol.x[0];
    result->control.vel = qp_sol.x[1];

    /* Saturate to vehicle limits */
    VehicleParams_t params = vehicle_model_get_params();
    result->control.steer = fp_clamp(result->control.steer,
        fp_neg(params.max_steer), params.max_steer);
    result->control.vel = fp_clamp(result->control.vel,
        params.min_vel, params.max_vel);

    /* Store for rate limiting in next iteration */
    g_prev_control = result->control;

    return status;
}

MpcConfig_t mpc_get_config(void)
{
    return g_config;
}

void mpc_set_config(const MpcConfig_t *config)
{
    if (config != NULL)
    {
        g_config = *config;
    }
}

void mpc_reset(void)
{
    g_prev_control.steer = 0;
    g_prev_control.vel = 0;
}
