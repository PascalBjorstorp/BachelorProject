/**
 * @file mpc.c
 * @brief Model Predictive Control Implementation
 *
 * Uses QP to optimize control over prediction horizon.
 * All arithmetic uses Q16.16 fixed-point.
 */

#include "mpc.h"
#include "qp_solver.h"
#include "fp_math.h"
#include "vehicle_model.h"
#include <string.h>

/*===========================================================================
 * Constants
 *===========================================================================*/

#define STATE_DIM   4   /* [x, y, heading, vel] */
#define CONTROL_DIM 2   /* [steer, vel] */
#define MAX_HORIZON 20

/*===========================================================================
 * Module State
 *===========================================================================*/

static MpcConfig_t g_config;
static int g_initialized = 0;
static ControlInput_t g_prev_control;

/*===========================================================================
 * Default Configuration
 *===========================================================================*/

static MpcConfig_t get_default_config(void)
{
    MpcConfig_t cfg;

    cfg.horizon = MPC_DEFAULT_HORIZON;
    cfg.dt = MPC_DEFAULT_DT;

    /* State tracking weights */
    cfg.w_x = FP_CONST(10.0);
    cfg.w_y = FP_CONST(10.0);
    cfg.w_heading = FP_CONST(5.0);
    cfg.w_vel = FP_CONST(2.0);

    /* Control effort weights */
    cfg.w_steer = FP_ONE;
    cfg.w_vel_cmd = FP_HALF;

    /* Control rate weights */
    cfg.w_steer_rate = FP_CONST(10.0);
    cfg.w_vel_rate = FP_CONST(5.0);

    /* Solver params */
    cfg.max_iter = MPC_DEFAULT_MAX_ITER;
    cfg.tolerance = MPC_DEFAULT_TOLERANCE;

    return cfg;
}

/*===========================================================================
 * QP Problem Construction
 *===========================================================================*/

static void build_hessian(int horizon, fixed_point_t *H)
{
    int n = horizon * CONTROL_DIM;
    memset(H, 0, n * n * sizeof(fixed_point_t));

    for (int k = 0; k < horizon; k++)
    {
        int base = k * CONTROL_DIM;
        int steer_idx = base * n + base;
        int vel_idx = (base + 1) * n + (base + 1);

        /* Diagonal: 2 * (effort + rate) weights */
        H[steer_idx] = fp_add(g_config.w_steer, g_config.w_steer_rate);
        H[vel_idx] = fp_add(g_config.w_vel_cmd, g_config.w_vel_rate);
    }
}

static void build_linear_cost(
    const VehicleState_t *state,
    const TrajectoryPoint_t *ref,
    int horizon,
    fixed_point_t *f)
{
    int n = horizon * CONTROL_DIM;
    memset(f, 0, n * sizeof(fixed_point_t));

    /* Simple feedforward: steer toward reference heading */
    for (int k = 0; k < horizon; k++)
    {
        int base = k * CONTROL_DIM;

        /* Heading error -> feedforward steering */
        fixed_point_t heading_err = fp_sub(ref[k].heading, state->heading);
        fixed_point_t ff_steer = fp_mul(heading_err, FP_CONST(1.0));

        /* Velocity error -> feedforward velocity */
        fixed_point_t ff_vel = ref[k].vel;

        /* Linear cost = -2 * w * feedforward */
        f[base] = fp_neg(fp_mul(
            FP_TWO,
            fp_mul(g_config.w_steer, ff_steer)));

        f[base + 1] = fp_neg(fp_mul(
            FP_TWO,
            fp_mul(g_config.w_vel_cmd, ff_vel)));
    }

    /* Rate penalty with previous control */
    f[0] = fp_sub(f[0],
        fp_mul(FP_TWO,
        fp_mul(g_config.w_steer_rate, g_prev_control.steer)));

    f[1] = fp_sub(f[1],
        fp_mul(FP_TWO,
        fp_mul(g_config.w_vel_rate, g_prev_control.vel)));
}

static void build_constraints(int horizon, fixed_point_t *A, fixed_point_t *b)
{
    int n = horizon * CONTROL_DIM;
    int m = horizon * 4;  /* 4 constraints per step */

    memset(A, 0, m * n * sizeof(fixed_point_t));
    memset(b, 0, m * sizeof(fixed_point_t));

    VehicleParams_t params = vehicle_model_get_params();

    for (int k = 0; k < horizon; k++)
    {
        int ctrl = k * CONTROL_DIM;
        int cons = k * 4;

        /* steer <= max_steer */
        A[(cons + 0) * n + ctrl] = FP_ONE;
        b[cons + 0] = params.max_steer;

        /* -steer <= max_steer (steer >= -max_steer) */
        A[(cons + 1) * n + ctrl] = fp_neg(FP_ONE);
        b[cons + 1] = params.max_steer;

        /* vel <= max_vel */
        A[(cons + 2) * n + (ctrl + 1)] = FP_ONE;
        b[cons + 2] = params.max_vel;

        /* -vel <= -min_vel (vel >= min_vel) */
        A[(cons + 3) * n + (ctrl + 1)] = fp_neg(FP_ONE);
        b[cons + 3] = fp_neg(params.min_vel);
    }
}

/*===========================================================================
 * Public API
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

SolverStatus_t mpc_compute(
    const VehicleState_t *state,
    const TrajectoryPoint_t *trajectory,
    MpcResult_t *result)
{
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
    int n_cons = horizon * 4;

    /* Build QP problem directly into struct */
    static QpProblem_t qp;
    qp_init_problem(&qp);
    qp.n_vars = n_vars;
    qp.n_constraints = n_cons;

    build_hessian(horizon, qp.H);
    build_linear_cost(state, trajectory, horizon, qp.f);
    build_constraints(horizon, qp.A, qp.b);

    /* Setup QP config */
    QpConfig_t qp_cfg;
    qp_init_config(&qp_cfg);
    qp_cfg.max_iter = g_config.max_iter;
    qp_cfg.tolerance = g_config.tolerance;
    qp_cfg.step_size = FP_CONST(0.01);

    /* Solve QP */
    static QpSolution_t qp_sol;
    SolverStatus_t status = qp_solve(&qp, &qp_cfg, &qp_sol);

    /* Extract first control */
    result->status = status;
    result->iterations = qp_sol.iterations;
    result->cost = qp_sol.residual;  /* Use residual as cost proxy */
    result->control.steer = qp_sol.x[0];
    result->control.vel = qp_sol.x[1];

    /* Saturate control */
    VehicleParams_t params = vehicle_model_get_params();
    result->control.steer = fp_clamp(result->control.steer,
        fp_neg(params.max_steer), params.max_steer);
    result->control.vel = fp_clamp(result->control.vel,
        params.min_vel, params.max_vel);

    /* Store for rate limiting */
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
