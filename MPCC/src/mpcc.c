/**
 * @file mpcc.c
 * @brief Model Predictive Contouring Control — Implementation
 *        (Global Frame: 7-state Liniger MPCC with Pacejka tires)
 *
 * Implements the MPCC controller using the global-frame formulation:
 *
 * State [7]: [s, vx, vy, omega, X, Y, psi]
 * Control [3]: [delta, a_x, v_theta]
 *
 * Algorithm (each control cycle):
 *   1. Warm-start: shift previous solution by one time step
 *   2. For each stage k = 0..N-1:
 *      a. Interpolate path at predicted s to get kappa(s), phi(s)
 *      b. Linearize dynamics (Pacejka tires, global frame)
 *      c. Build stage cost (contouring/lag errors, progress reward)
 *      d. Set per-stage track bounds from path
 *      e. Linearize obstacle constraints (if active)
 *   3. Solve multistage QP via ADMM + Riccati
 *   4. Extract first control input
 *
 * Dynamics:
 *   ds/dt     = v_theta                         (virtual progress control)
 *   dvx/dt    = (-F_yf*sin(d) + F_x) / m + vy*omega
 *   dvy/dt    = (F_yf*cos(d) + F_yr) / m - vx*omega
 *   domega/dt = (l_f*F_yf*cos(d) - l_r*F_yr) / I_z
 *   dX/dt     = vx*cos(psi) - vy*sin(psi)
 *   dY/dt     = vx*sin(psi) + vy*cos(psi)
 *   dpsi/dt   = omega
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */
#define _GNU_SOURCE

#include <math.h>
#include "mpcc_types.h"
#include "mpcc.h"
#include "mpcc_vehicle_model.h"
#include "qp_solver_mpcc.h"
#include <string.h>


#ifdef MPCC_DEBUG_PRINT
#include <stdio.h>
#endif

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MPCCConfiguration_t config;
static MPCCReferencePath_t ref_path;

/* Path tracking state — reset when path changes or controller resets */
static uint16_t last_closest_idx = 0;
static MPCCObstacleSet_t obstacle_set;
static uint8_t mpcc_initialized = 0;

/** Previous solution for warm-starting */
static MPCCState_t prev_predicted_states[MPCC_MAX_HORIZON + 1];
static MPCCControl_t prev_predicted_controls[MPCC_MAX_HORIZON];
static uint8_t warm_start_available = 0;

/** Previous applied control (for rate penalties) */
static MPCCControl_t prev_control;

/** ADMM solver workspace */
static ADMMWorkspace_t admm_workspace;
static ADMMConfig_t admm_config;

/** QP problem structure */
static MPCCQPProblem_t qp_problem;

/*===========================================================================
 * Default Configuration
 *===========================================================================*/

static MPCCConfiguration_t get_default_config(void)
{
    MPCCConfiguration_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* Prediction horizon */
    cfg.horizon_steps = MPCC_DEFAULT_HORIZON;
    cfg.dt = MPCC_DEFAULT_DT;

    /* Frenet tracking */
    cfg.weight_contouring = MPCC_DEFAULT_WEIGHT_CONTOURING;
    cfg.weight_lag = MPCC_DEFAULT_WEIGHT_LAG;
    cfg.weight_progress = MPCC_DEFAULT_WEIGHT_PROGRESS;

    /* State regularization */
    cfg.weight_vx = MPCC_DEFAULT_WEIGHT_VX;
    cfg.vx_ref = MPCC_DEFAULT_VX_REF;
    cfg.weight_vy = MPCC_DEFAULT_WEIGHT_VY;
    cfg.weight_omega = MPCC_DEFAULT_WEIGHT_OMEGA;

    /* Control effort */
    cfg.weight_delta = MPCC_DEFAULT_WEIGHT_DELTA;
    cfg.weight_ax = MPCC_DEFAULT_WEIGHT_AX;
    cfg.weight_v_theta = MPCC_DEFAULT_WEIGHT_V_THETA;

    /* Control rate */
    cfg.weight_delta_rate = MPCC_DEFAULT_WEIGHT_DELTA_RATE;
    cfg.weight_ax_rate = MPCC_DEFAULT_WEIGHT_AX_RATE;
    cfg.weight_v_theta_rate = MPCC_DEFAULT_WEIGHT_V_THETA_RATE;

    /* Terminal */
    cfg.weight_contouring_terminal = MPCC_DEFAULT_WEIGHT_CONTOURING_TERMINAL;
    cfg.weight_lag_terminal = MPCC_DEFAULT_WEIGHT_LAG_TERMINAL;
    cfg.weight_progress_terminal = MPCC_DEFAULT_WEIGHT_PROGRESS_TERMINAL;

    /* Obstacle avoidance */
    cfg.weight_obstacle = MPCC_DEFAULT_WEIGHT_OBSTACLE;
    cfg.obstacle_margin = MPCC_DEFAULT_OBSTACLE_MARGIN;

    /* ADMM */
    cfg.admm_rho = MPCC_DEFAULT_ADMM_RHO;
    cfg.admm_max_iterations = MPCC_DEFAULT_ADMM_MAX_ITER;
    cfg.admm_tolerance = MPCC_DEFAULT_ADMM_TOLERANCE;

    /* Constraint bounds */
    cfg.delta_max = F110_DEFAULT_MAXIMUM_STEERING_RADIANS;
    cfg.ax_max = MPCC_DEFAULT_AX_MAX;
    cfg.ax_min = MPCC_DEFAULT_AX_MIN;
    cfg.vx_max = F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND;
    cfg.vx_min = F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND;
    cfg.v_theta_max = MPCC_DEFAULT_V_THETA_MAX;
    cfg.v_theta_min = MPCC_DEFAULT_V_THETA_MIN;

    /* Linear tire model */
    cfg.mu   = MPCC_DEFAULT_MU;
    cfg.C_Sf = MPCC_DEFAULT_C_SF;
    cfg.C_Sr = MPCC_DEFAULT_C_SR;

    /* Cross-call rate scaling */
    cfg.cross_call_rate_scale = MPCC_DEFAULT_CROSS_CALL_SCALE;

    return cfg;
}

static void sanitize_config(MPCCConfiguration_t *cfg)
{
    if (!cfg)
        return;

    if (cfg->horizon_steps == 0)
        cfg->horizon_steps = MPCC_DEFAULT_HORIZON;
    if (cfg->horizon_steps > MPCC_MAX_HORIZON)
        cfg->horizon_steps = MPCC_MAX_HORIZON;
}

/*===========================================================================
 * Initialization
 *===========================================================================*/

void mpcc_initialize(void)
{
    config = get_default_config();

    admm_solver_default_config(&admm_config);
    admm_solver_initialize(&admm_workspace);

    memset(&prev_control, 0, sizeof(prev_control));
    memset(&ref_path, 0, sizeof(ref_path));
    memset(&obstacle_set, 0, sizeof(obstacle_set));
    warm_start_available = 0;
    mpcc_initialized = 1;
}

void mpcc_initialize_with_config(const MPCCConfiguration_t *cfg)
{
    config = cfg ? *cfg : get_default_config();
    sanitize_config(&config);

    admm_solver_default_config(&admm_config);
    admm_config.rho = config.admm_rho;
    admm_config.max_iterations = config.admm_max_iterations;
    admm_config.eps_primal = config.admm_tolerance;
    admm_config.eps_dual = config.admm_tolerance;
    admm_config.warm_start = 0;

    admm_solver_initialize(&admm_workspace);

    memset(&prev_control, 0, sizeof(prev_control));
    memset(&obstacle_set, 0, sizeof(obstacle_set));
    warm_start_available = 0;
    mpcc_initialized = 1;
}

/*===========================================================================
 * Reference Path
 *===========================================================================*/

void mpcc_set_reference_path(const MPCCReferencePath_t *path)
{
    ref_path = *path;
    warm_start_available = 0;
    last_closest_idx = 0; /* Reset path tracking when new path is loaded */
}

/*===========================================================================
 * Obstacles
 *===========================================================================*/

void mpcc_clear_obstacles(void)
{
    memset(&obstacle_set, 0, sizeof(obstacle_set));
}

void mpcc_obstacle_compute_sigma_inv(MPCCObstacle_t *obs)
{
    /* Sigma_inv = R(-phi) * diag(1/a^2, 1/b^2) * R(-phi)^T
     *
     * Let c = cos(phi), s = sin(phi), ia2 = 1/a^2, ib2 = 1/b^2
     *   Sigma_inv[0][0] = c^2*ia2 + s^2*ib2
     *   Sigma_inv[0][1] = c*s*(ia2 - ib2)
     *   Sigma_inv[1][0] = Sigma_inv[0][1]
     *   Sigma_inv[1][1] = s^2*ia2 + c^2*ib2
     */
    float c = cosf(obs->phi);
    float s = sinf(obs->phi);
    float c2 = c * c;
    float s2 = s * s;
    float cs = c * s;

    float ia2 = 1.0f / (obs->a * obs->a);
    float ib2 = 1.0f / (obs->b * obs->b);

    obs->Sigma_inv[0][0] = (c2 * ia2) + (s2 * ib2);
    obs->Sigma_inv[0][1] = cs * (ia2 - ib2);
    obs->Sigma_inv[1][0] = obs->Sigma_inv[0][1];
    obs->Sigma_inv[1][1] = (s2 * ia2) + (c2 * ib2);
}

/*===========================================================================
 * Path Interpolation
 *===========================================================================*/

/** Wrap s to [0, total_length) for closed paths. */
static float wrap_s(float s, float total_length)
{
    while (s >= total_length)
        s = s - total_length;
    while (s < 0)
        s = s - total_length;
    return s;
}

void mpcc_path_interpolate(
    const MPCCReferencePath_t *path,
    float s,
    MPCCPathPoint_t *result)
{
    if (path->num_points < 2) {
        memset(result, 0, sizeof(*result));
        return;
    }

    /* Clamp s to valid range */
    float s_min = path->points[0].s_ref;
    float s_max = path->points[path->num_points - 1].s_ref;

    if (path->is_closed && s_max > s_min) {
        /* Wrap s into [s_min, s_max) for closed paths.
         * Use fmodf instead of a while-loop to avoid O(n) iterations
         * when s is very large (e.g. from non-convergent ADMM z_x). */
        float len = s_max - s_min;
        s = s - s_min;
        s = fmodf(s, len);
        if (s < 0) s += len;
        s = s + s_min;
    } else {
        if (s <= s_min) { *result = path->points[0]; result->s_ref = s; return; }
        if (s >= s_max) { *result = path->points[path->num_points - 1]; result->s_ref = s; return; }
    }

    /* Binary search for segment: find i such that points[i].s_ref <= s < points[i+1].s_ref */
    uint16_t lo = 0;
    uint16_t hi = path->num_points - 1;
    while (hi - lo > 1) {
        uint16_t mid = (lo + hi) / 2;
        if (path->points[mid].s_ref <= s)
            lo = mid;
        else
            hi = mid;
    }

    const MPCCPathPoint_t *p0 = &path->points[lo];
    const MPCCPathPoint_t *p1 = &path->points[hi];

    /* Interpolation parameter t = (s - s0) / (s1 - s0) */
    float ds = p1->s_ref - p0->s_ref;
    float t;
    if (ds <= 0) {
        t = 0;
    } else {
        t = (s - p0->s_ref)/ ds;
    }
    float one_minus_t = 1.0f - t;

    /* Linear interpolation: result = (1-t)*p0 + t*p1 */
    result->x_ref       = (one_minus_t* p0->x_ref) + (t * p1->x_ref);
    result->y_ref       = (one_minus_t * p0->y_ref) + (t * p1->y_ref);
    result->kappa_ref   = (one_minus_t * p0->kappa_ref) + (t * p1->kappa_ref);
    result->left_bound  = (one_minus_t * p0->left_bound) +  (t * p1->left_bound);
    result->right_bound = (one_minus_t * p0->right_bound) + (t * p1->right_bound);
    result->vx_ref      = (one_minus_t * p0->vx_ref) + (t * p1->vx_ref);
    result->s_ref       = s;

    /* Angle interpolation: handle wrapping for phi_ref */
    float dphi = remainderf(p1->phi_ref - p0->phi_ref, 2.0f * (float)M_PI);
    result->phi_ref = remainderf(p0->phi_ref + (t * dphi), 2.0f * (float)M_PI);
}

/*===========================================================================
 * Find Closest s
 *===========================================================================*/

float mpcc_find_closest_s(
    const MPCCReferencePath_t *path,
    float X,
    float Y)
{
    if (path->num_points < 1) return 0.0f;

    float best_dist_sq = __FLT_MAX__;
    uint16_t best_idx = 0;

    for (uint16_t i = 0; i < path->num_points; i++) {
        float dx = X - path->points[i].x_ref;
        float dy = Y - path->points[i].y_ref;
        float dist_sq = dx * dx + dy * dy; 
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
        }
    }

    return path->points[best_idx].s_ref;
}

/* Find closest s near a hint to preserve longitudinal continuity. */
static float mpcc_find_closest_s_with_hint(
    const MPCCReferencePath_t *path,
    float X,
    float Y,
    float s_hint)
{
    if (path->num_points < 1) return 0;
    if (path->num_points < 8 || s_hint <= 0)
        return mpcc_find_closest_s(path, X, Y);

    float s_min = path->points[0].s_ref;
    float s_max = path->points[path->num_points - 1].s_ref;
    float s_query = s_hint;
    if (path->is_closed && s_max > s_min) {
        float len = s_max - s_min;
        while (s_query > s_max) s_query = s_query - len;
        while (s_query < s_min) s_query = s_query - len;
    }

    uint16_t lo = 0;
    uint16_t hi = path->num_points - 1;
    while (hi - lo > 1) {
        uint16_t mid = (uint16_t)((lo + hi) / 2);
        if (path->points[mid].s_ref <= s_query) lo = mid;
        else hi = mid;
    }
    uint16_t center = lo;

    const int window = 80;
    float best_dist_sq = __FLT_MAX__;
    uint16_t best_idx = center;
    for (int off = -window; off <= window; off++) {
        int idx = (int)center + off;
        if (path->is_closed) {
            while (idx < 0) idx += path->num_points;
            while (idx >= (int)path->num_points) idx -= path->num_points;
        } else {
            if (idx < 0 || idx >= (int)path->num_points) continue;
        }
        float dx = X - path->points[idx].x_ref;
        float dy = Y - path->points[idx].y_ref;
        float dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = (uint16_t)idx;
        }
    }

    /* Local search failed (far from path): fall back to global search. */
    {
        float max_dist = 3.0f;
        float max_dist_sq = max_dist * max_dist;
        if (best_dist_sq > max_dist_sq)
            return mpcc_find_closest_s(path, X, Y);
    }

    return path->points[best_idx].s_ref;
}

/* MPC-style forward-biased closest-waypoint search with heading penalty. */
static uint16_t mpcc_find_closest_index_forward_biased(
    const MPCCReferencePath_t *path,
    float X,
    float Y,
    float psi,
    uint16_t start_idx)
{
    if (path->num_points == 0) return 0;

    const int search_forward = 200;
    const int search_backward = 100; /* Increased for recovery after segment loss */
    uint16_t best_idx = start_idx;
    float best_score = __FLT_MAX__;

    float veh_dx = cosf(psi);
    float veh_dy = sinf(psi);
    float behind_penalty = 2.0f;

    for (int off = -search_backward; off < search_forward; off++) {
        int idx = (int)start_idx + off;
        if (path->is_closed) {
            while (idx < 0) idx += path->num_points;
            while (idx >= (int)path->num_points) idx -= path->num_points;
        } else {
            if (idx < 0 || idx >= (int)path->num_points) continue;
        }

        float dx = path->points[idx].x_ref - X;
        float dy = path->points[idx].y_ref- Y;

        float dist_sq = dx * dx + dy * dy;
        float dot = (dx * veh_dx) + (dy * veh_dy);
        float score = dist_sq;
        if (dot < 0) score += behind_penalty * behind_penalty;

        if (score < best_score) {
            best_score = score;
            best_idx = (uint16_t)idx;
        }
    }
    return best_idx;
}

/* Project onto segment [idx0, idx1] like MPC does, then compute s on segment. */
static float mpcc_project_s_on_segment(
    const MPCCReferencePath_t *path,
    uint16_t idx0,
    uint16_t idx1,
    float X,
    float Y)
{
    const MPCCPathPoint_t *p0 = &path->points[idx0];
    const MPCCPathPoint_t *p1 = &path->points[idx1];

    float ax = p0->x_ref, ay = p0->y_ref;
    float bx = p1->x_ref, by = p1->y_ref;
    float abx = bx - ax;
    float aby = by - ay;
    float apx = X - ax;
    float apy = Y - ay;
    float ab_len2 = (abx * abx) + (aby * aby);

    float t = 0;
    if (ab_len2 > 1e-9f)
        t = ((apx * abx) + (apy * aby)) / ab_len2;
    if (t < 0) t = 0;
    if (t > 1.0f) t = 1.0f;

    return p0->s_ref + (t * (p1->s_ref - p0->s_ref));
}

/*===========================================================================
 * Vehicle State → MPCC State Conversion
 *===========================================================================*/

MPCCState_t mpcc_state_from_vehicle_state(
    const VehicleState_t *vs,
    float s_hint)
{
    (void)s_hint;
    MPCCState_t st;
    memset(&st, 0, sizeof(st));

    /* Copy Cartesian/body-frame states directly */
    st.X     = vs->pos_x;
    st.Y     = vs->pos_y;
    st.psi   = vs->heading;
    st.vx    = vs->long_vel;
    st.vy    = vs->lat_vel;
    st.omega = vs->yaw_rate;

    /* Compute s via forward-biased closest-waypoint search + segment projection */
    {
        if (last_closest_idx >= ref_path.num_points) last_closest_idx = 0;
        uint16_t idx0 = mpcc_find_closest_index_forward_biased(
            &ref_path, st.X, st.Y, st.psi, last_closest_idx);
        uint16_t idx1 = (uint16_t)(idx0 + 1);
        if (idx1 >= ref_path.num_points)
            idx1 = ref_path.is_closed ? 0 : (ref_path.num_points - 1);
        last_closest_idx = idx0;
        st.s = mpcc_project_s_on_segment(&ref_path, idx0, idx1, st.X, st.Y);
    }

    return st;
}

/*===========================================================================
 * Cost Construction (Global Frame MPCC)
 *===========================================================================*/

/**
 * Build stage cost matrices for the global-frame MPCC formulation.
 *
 * Cost at each stage:
 *   l_k = q_c * e_c^2 + q_l * e_l^2  (contouring/lag, added separately)
 *       + q_vy * vy^2 + q_omega * omega^2
 *       + q_vx * (vx - vx_ref)^2
 *       - q_s * v_theta             (linear progress reward on control)
 *       + u^T R u                   (control effort)
 *
 * @param stage_cost  Output cost structure
 * @param is_terminal  Whether this is the terminal stage (N)
 */
static void build_stage_cost(
    MPCCStageCost_t *cost,
    uint8_t is_terminal,
    uint16_t step_k)
{
    memset(cost, 0, sizeof(*cost));

    /* Quadratic state costs (diagonal).
     * Note: contouring/lag error Q contributions are added separately
     * in add_contouring_lag_cost() since they depend on the operating point. */

    /* State regularization (same for running and terminal) */
    cost->Q[MPCC_IDX_VY][MPCC_IDX_VY] = config.weight_vy;
    cost->Q[MPCC_IDX_OMEGA][MPCC_IDX_OMEGA] = config.weight_omega;

    /* Velocity tracking: q_vx * (vx - vx_ref)^2
     * = q_vx * vx^2 - 2*q_vx*vx_ref*vx + const
     * Q[VX][VX] = q_vx,  q[VX] = -2*q_vx*vx_ref */
    if (config.weight_vx > 0)
    {
        cost->Q[MPCC_IDX_VX][MPCC_IDX_VX] = config.weight_vx;
        cost->q[MPCC_IDX_VX] = -(2.0f * config.weight_vx * config.vx_ref);
    }

    /* Progress reward: -q_theta * v_theta (linear term on v_theta control).
     * Terminal stage: still reward being far along via linear cost on s. */
    if (is_terminal)
    {
        float q_s = config.weight_progress_terminal;
        cost->q[MPCC_IDX_S] = (0 - q_s); /* negative = reward */
    }

    /* Control costs (not used for terminal stage) */
    if (!is_terminal)
    {
        float q_progress = config.weight_progress;
        cost->r[MPCC_IDX_VTHETA] = (0 - q_progress); /* negative = reward */

        /* Rate weights: on step 0 scale by cross_call_rate_scale to
         * compensate for the control callback running faster than the
         * prediction dt. */
        float w_dr = config.weight_delta_rate;
        float w_ar = config.weight_ax_rate;
        float w_vr = config.weight_v_theta_rate;
        if (step_k == 0)
        {
            w_dr = w_dr * config.cross_call_rate_scale;
            w_ar = w_ar * config.cross_call_rate_scale;
            w_vr = w_vr * config.cross_call_rate_scale;
        }

        cost->R[MPCC_IDX_DELTA][MPCC_IDX_DELTA] =
            (config.weight_delta + w_dr);
        cost->R[MPCC_IDX_AX][MPCC_IDX_AX] =
            (config.weight_ax + w_ar);
        cost->R[MPCC_IDX_VTHETA][MPCC_IDX_VTHETA] =
            (config.weight_v_theta + w_vr);
    }
}

/*===========================================================================
 * Contouring / Lag Error Linearization (Real MPCC Errors)
 *===========================================================================
 *
 * Standard MPCC (Liniger 2015) defines errors at the virtual arc parameter s:
 *
 *   e_c = sin(phi(s)) * (X - gamma_x(s)) - cos(phi(s)) * (Y - gamma_y(s))
 *   e_l = -cos(phi(s)) * (X - gamma_x(s)) - sin(phi(s)) * (Y - gamma_y(s))
 *
 * where gamma(s) is the reference path point and phi(s) is the tangent angle.
 *
 * Cost contribution: q_c * e_c^2 + q_l * e_l^2
 *
 * Linearized at operating point (s_bar, X_bar, Y_bar):
 *   e_c ≈ g_c^T x + d_c   where g_c has entries at [s, X, Y]
 *   e_l ≈ g_l^T x + d_l   where g_l has entries at [s, X, Y]
 *
 * Jacobians:
 *   de_c/ds = -kappa * e_l_bar
 *   de_c/dX = sin(phi)
 *   de_c/dY = -cos(phi)
 *
 *   de_l/ds = kappa * e_c_bar + 1
 *   de_l/dX = -cos(phi)
 *   de_l/dY = -sin(phi)
 *
 * Resulting QP cost addition (where cost_form = 0.5 x^T Q x + q^T x):
 *   Q += 2*(q_c * g_c * g_c^T + q_l * g_l * g_l^T)
 *   q += 2*(q_c * d_c * g_c + q_l * d_l * g_l)
 */
static void add_contouring_lag_cost(
    MPCCStageCost_t *cost,
    const MPCCState_t *z_bar,
    const MPCCPathPoint_t *path_pt,
    float q_c,
    float q_l)
{
    if (q_c == 0 && q_l == 0) return;

    /* Path quantities at operating point */
    float sin_phi = sinf(path_pt->phi_ref);
    float cos_phi = cosf(path_pt->phi_ref);
    float kappa   = path_pt->kappa_ref;

    /* Position error in global frame */
    float dX = (z_bar->X - path_pt->x_ref);
    float dY = (z_bar->Y - path_pt->y_ref);

    /* Errors at operating point */
    float e_c_bar = sin_phi * dX - cos_phi * dY;
    float e_l_bar = -(cos_phi * dX + sin_phi * dY);

    /* Gradient vectors (only s, X, Y components are nonzero) */
    float g_c[MPCC_NX];
    float g_l[MPCC_NX];
    memset(g_c, 0, sizeof(g_c));
    memset(g_l, 0, sizeof(g_l));

    g_c[MPCC_IDX_S] = (0 - (kappa * e_l_bar));  /* -kappa * e_l */
    g_c[MPCC_IDX_X] = sin_phi;
    g_c[MPCC_IDX_Y] = (0 - cos_phi);                 /* -cos(phi) */

    g_l[MPCC_IDX_S] = ((kappa + e_c_bar) * 1.0f);  /* kappa * e_c + 1 */
    g_l[MPCC_IDX_X] = (0 - cos_phi);                      /* -cos(phi) */
    g_l[MPCC_IDX_Y] = (0 - sin_phi);                      /* -sin(phi) */

    /* Constant terms: d = e_bar - g^T * x_bar */
    float x_bar[MPCC_NX] = {
        z_bar->s,
        z_bar->vx, z_bar->vy, z_bar->omega,
        z_bar->X, z_bar->Y, z_bar->psi
    };

    /* d_c = e_c_bar - g_c^T * x_bar */
    float gc_xbar = 0.0f;
    float gl_xbar = 0.0f;
    for (int i = 0; i < MPCC_NX; i++) {
        gc_xbar += g_c[i] * x_bar[i];
        gl_xbar += g_l[i] * x_bar[i];
    }
    float d_c = e_c_bar - gc_xbar;
    float d_l = e_l_bar - gl_xbar;

    /* Add to Q matrix:  Q += 2*(q_c * g_c * g_c^T + q_l * g_l * g_l^T)
     * (factor 2 because cost form is 0.5 * x^T Q x) */
    for (int i = 0; i < MPCC_NX; i++) {
        if (g_c[i] == 0 && g_l[i] == 0) continue;
        for (int j = 0; j < MPCC_NX; j++) {
            if (g_c[j] == 0 && g_l[j] == 0) continue;
            float qc_gi_gj = q_c * g_c[i] * g_c[j];
            float ql_gi_gj = q_l * g_l[i] * g_l[j];
            cost->Q[i][j] += 2.0f * (qc_gi_gj + ql_gi_gj);
        }
    }

    /* Add to q vector:  q += 2*(q_c * d_c * g_c + q_l * d_l * g_l) */
    for (int i = 0; i < MPCC_NX; i++) {
        if (g_c[i] == 0 && g_l[i] == 0) continue;
        float qc_dc_gi = q_c * d_c * g_c[i];
        float ql_dl_gi = q_l * d_l * g_l[i];
        cost->q[i] += 2.0f * (qc_dc_gi + ql_dl_gi);
    }
}

/*===========================================================================
 * QP Problem Construction
 *===========================================================================*/

static void build_qp_problem(
    const MPCCState_t *x0,
    MPCCQPProblem_t *qp)
{
    uint16_t N = config.horizon_steps;
    if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;

    memset(qp, 0, sizeof(*qp));
    qp->N = N;

    /* Set initial state */
    qp->x0[MPCC_IDX_S]       = x0->s;
    qp->x0[MPCC_IDX_VX]      = x0->vx;
    qp->x0[MPCC_IDX_VY]      = x0->vy;
    qp->x0[MPCC_IDX_OMEGA]   = x0->omega;
    qp->x0[MPCC_IDX_X]       = x0->X;
    qp->x0[MPCC_IDX_Y]       = x0->Y;
    qp->x0[MPCC_IDX_PSI]     = x0->psi;

    /* --- Global box constraints --- */

    /* State bounds (use large values for unbounded) */
    float big = 1000.0f;
    for (int i = 0; i < MPCC_NX; i++)
    {
        qp->x_lower[i] = (0 - big);
        qp->x_upper[i] = big;
    }

    /* s: must be non-negative */
    qp->x_lower[MPCC_IDX_S] = 0;

    /* vx: bounded */
    qp->x_lower[MPCC_IDX_VX] = config.vx_min;
    qp->x_upper[MPCC_IDX_VX] = config.vx_max;

    /* Control bounds */
    qp->u_lower[MPCC_IDX_DELTA] = (0 - config.delta_max);
    qp->u_upper[MPCC_IDX_DELTA] = config.delta_max;
    qp->u_lower[MPCC_IDX_AX] = config.ax_min;
    qp->u_upper[MPCC_IDX_AX] = config.ax_max;
    qp->u_lower[MPCC_IDX_VTHETA] = config.v_theta_min;
    qp->u_upper[MPCC_IDX_VTHETA] = config.v_theta_max;

    /* Friction circle: (mu * g)^2 for combined acceleration constraint */
    float mu_g = config.mu * F110_GRAVITY_ACCELERATION_MS2;
    qp->mu_g_sq = mu_g * mu_g;

    /* --- Build stages --- */
    /* TODO: Use warm-start trajectory as operating points.
     * For now, use the initial state propagated forward. */

    MPCCState_t z_bar = *x0;
    MPCCControl_t u_bar;
    if (warm_start_available)
        u_bar = prev_predicted_controls[0];
    else {
        u_bar.delta = 0;
        u_bar.a_x = 0;
        u_bar.v_theta = 1.0f;
    }

    for (uint16_t k = 0; k < N; k++)
    {
        /* Get operating point: k=0 always uses the actual current state x0
         * for accurate linearization.  Subsequent stages use the warm-start
         * trajectory (shifted from previous solve). */
        if (warm_start_available && k > 0)
        {
            /* shift_warm_start() already advanced the trajectory by one step,
             * so stage k should use index k (not k+1). */
            z_bar = prev_predicted_states[k];
            u_bar = prev_predicted_controls[k];
        }

        /* Interpolate path at s_bar */
        MPCCPathPoint_t path_pt;
        mpcc_path_interpolate(&ref_path, z_bar.s, &path_pt);

        /* Linearize dynamics */
        mpcc_linearize_dynamics(&z_bar, &u_bar,
                          config.dt, &config, &qp->dynamics[k]);

        /* Build stage cost */
        build_stage_cost(&qp->stage_cost[k], 0, k);

        /* Add real contouring/lag error cost (linearized at operating point) */
        add_contouring_lag_cost(&qp->stage_cost[k], &z_bar, &path_pt,
                                config.weight_contouring, config.weight_lag);

        /* Override vx_ref with the per-stage value from the raceline velocity
         * profile so the solver naturally slows before turns. */
        if (path_pt.vx_ref > 0 && config.weight_vx > 0) {
            qp->stage_cost[k].q[MPCC_IDX_VX] = -(2.0f * config.weight_vx * path_pt.vx_ref);
#ifdef MPCC_DEBUG_PRINT
            if (k == 0) {
                printf("  [QP] s_bar=%.2f vx_ref=%.3f q_vx=%.1f Q_vx=%.1f\n",
                       (double)(z_bar.s), (double)(path_pt.vx_ref),
                       (double)(qp->stage_cost[k].q[MPCC_IDX_VX]),
                       (double)(qp->stage_cost[k].Q[MPCC_IDX_VX][MPCC_IDX_VX]));
            }
#endif
        }

        /* Rate penalty cross-term: penalize (u_k - u_ref_k)^2.
         * The quadratic part (weight_rate * u_k^2) is already in R.
         * Add the linear cross-term: r[i] += -2 * weight_rate * u_ref[i].
         * u_ref comes from shifted warm-start (or zero for cold start).
         * This creates proper rate penalization that allows steady-state
         * steering without penalty but penalizes oscillation.
         * At step 0: scale rate weights by cross_call_rate_scale. */
        {
            MPCCControl_t u_ref;
            if (warm_start_available)
                u_ref = prev_predicted_controls[k];
            else {
                u_ref.delta = 0;
                u_ref.a_x = 0;
                u_ref.v_theta = 1.0f;
            }

            float w_dr = config.weight_delta_rate;
            float w_ar = config.weight_ax_rate;
            float w_vr = config.weight_v_theta_rate;
            if (k == 0)
            {
                w_dr = (w_dr * config.cross_call_rate_scale);
                w_ar = (w_ar * config.cross_call_rate_scale);
                w_vr = (w_vr * config.cross_call_rate_scale);
            }

            qp->stage_cost[k].r[MPCC_IDX_DELTA] =
                qp->stage_cost[k].r[MPCC_IDX_DELTA]
                - 2.0f * w_dr * u_ref.delta;

            qp->stage_cost[k].r[MPCC_IDX_AX] =
                qp->stage_cost[k].r[MPCC_IDX_AX]
                - 2.0f * w_ar * u_ref.a_x;

            qp->stage_cost[k].r[MPCC_IDX_VTHETA] =
                qp->stage_cost[k].r[MPCC_IDX_VTHETA]
                - 2.0f * w_vr * u_ref.v_theta;
        }

        /* Per-stage track bounds on n */
        qp->track_left[k] = path_pt.left_bound;
        qp->track_right[k] = path_pt.right_bound;

        /* Per-stage friction circle: tighten a_x bound based on
         * lateral acceleration at operating point.
         * a_y = vx * omega,  |a_x| <= sqrt((mu*g)^2 - a_y^2) */
        if (qp->mu_g_sq > 0.0f)
        {
            float vx_op = z_bar.vx;
            float omega_op = z_bar.omega;
            float a_y = vx_op * omega_op;
            float a_y_sq = a_y * a_y;
            float ax_box = config.ax_max;
            if (a_y_sq < qp->mu_g_sq) {
                float ax_fc = sqrtf(qp->mu_g_sq - a_y_sq);
                qp->ax_lim_stage[k] = ax_fc < ax_box ? ax_fc : ax_box;
            } else {
                qp->ax_lim_stage[k] = 0.0f;
            }
        }
        else
        {
            qp->ax_lim_stage[k] = config.ax_max;
        }

        /* TODO: If obstacles are active, linearize ellipsoidal constraints
         * and add as softened penalty to the stage cost.
         *
         * For each obstacle j:
         *   h_j(X, Y) = (p - c)^T Sigma_inv (p - c) - 1
         *   If h_j < 0 at the operating point (constraint violated):
         *     Linearize: grad_h^T * [dX; dY] + h_bar >= 0
         *     Add as penalty: weight_obstacle * max(0, -h_lin)^2
         */
    }

    /* Terminal stage track bound */
    {
        MPCCPathPoint_t path_pt;
        mpcc_path_interpolate(&ref_path, z_bar.s, &path_pt);
        qp->track_left[N] = path_pt.left_bound;
        qp->track_right[N] = path_pt.right_bound;

        /* Terminal cost — also override vx_ref so terminal matches stage vx targets */
        build_stage_cost(&qp->terminal_cost, 1, N);

        /* Add real contouring/lag error cost at terminal stage */
        add_contouring_lag_cost(&qp->terminal_cost, &z_bar, &path_pt,
                                config.weight_contouring_terminal,
                                config.weight_lag_terminal);

        if (path_pt.vx_ref > 0 && config.weight_vx > 0) {
            qp->terminal_cost.q[MPCC_IDX_VX] = -(2.0f * config.weight_vx * path_pt.vx_ref);
        }
    }
}

/*===========================================================================
 * Warm Start Management
 *===========================================================================*/

static void state_to_array(const MPCCState_t *st, float arr[MPCC_NX]);

static void shift_warm_start(void)
{
    uint16_t N = config.horizon_steps;
    if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;

    /* Shift states: prev[k] = prev[k+1] for k = 0..N-1 */
    for (uint16_t k = 0; k < N; k++)
        prev_predicted_states[k] = prev_predicted_states[k + 1];
    /* Terminal state stays (last → last) */

    /* Shift controls: prev[k] = prev[k+1] for k = 0..N-2, hold last */
    for (uint16_t k = 0; k + 1 < N; k++)
        prev_predicted_controls[k] = prev_predicted_controls[k + 1];
    /* prev_predicted_controls[N-1] stays as-is (hold last) */

    /* Copy shifted trajectory into ADMM workspace */
    for (uint16_t k = 0; k <= N; k++)
    {
        state_to_array(&prev_predicted_states[k], admm_workspace.z_x[k]);
        memcpy(admm_workspace.w_x[k], admm_workspace.z_x[k],
               sizeof(float) * MPCC_NX);
    }
    for (uint16_t k = 0; k < N; k++)
    {
        admm_workspace.z_u[k][MPCC_IDX_DELTA] = prev_predicted_controls[k].delta;
        admm_workspace.z_u[k][MPCC_IDX_AX]    = prev_predicted_controls[k].a_x;
        admm_workspace.z_u[k][MPCC_IDX_VTHETA] = prev_predicted_controls[k].v_theta;
        memcpy(admm_workspace.w_u[k], admm_workspace.z_u[k],
               sizeof(float) * MPCC_NU);
    }

    /* Shift dual variables (lambda) forward to preserve ADMM convergence
     * history.  Zeroing lambda defeats warm-starting — ADMM needs the
     * accumulated constraint-violation information to converge properly. */
    for (uint16_t k = 0; k < N; k++) {
        memcpy(admm_workspace.lambda_x[k], admm_workspace.lambda_x[k + 1],
               sizeof(float) * MPCC_NX);
        if (k + 1 < N)
            memcpy(admm_workspace.lambda_u[k], admm_workspace.lambda_u[k + 1],
                   sizeof(float) * MPCC_NU);
    }
    /* k=0 state is hard-fixed to x0 each solve; reset its dual row. */
    memset(admm_workspace.lambda_x[0], 0, sizeof(admm_workspace.lambda_x[0]));
    /* Terminal lambda: hold last */
    /* lambda_x[N] stays as lambda_x[N] (already in place after shift) */
    /* lambda_u[N-1] stays as lambda_u[N-1] (already in place after shift) */
}

/*===========================================================================
 * State Pack/Unpack Helpers
 *===========================================================================*/

static void state_to_array(const MPCCState_t *st, float arr[MPCC_NX])
{
    arr[0] = st->s;     arr[1] = st->vx;     arr[2] = st->vy;
    arr[3] = st->omega;  arr[4] = st->X;     arr[5] = st->Y;
    arr[6] = st->psi;
}

static void array_to_state(const float arr[MPCC_NX], MPCCState_t *st)
{
    st->s = arr[0];     st->vx = arr[1];      st->vy = arr[2];
    st->omega = arr[3];  st->X = arr[4];      st->Y = arr[5];
    st->psi = arr[6];
}

/*===========================================================================
 * Main Control Computation
 *===========================================================================*/

MPCCStatus_t mpcc_compute_control(
    const MPCCState_t *current_state,
    MPCCResult_t *result)
{
    if (!mpcc_initialized)
    {
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    if (ref_path.num_points < 2)
    {
        result->status = MPCC_STATUS_ERROR;
#ifdef MPCC_DEBUG_PRINT
        printf("[MPCC] Error: reference path has < 2 points\n");
#endif
        return MPCC_STATUS_ERROR;
    }

    /* Warm-start: shift previous solution forward */
    MPCCControl_t fallback_control = {0, 0, 1.0f};
    if (warm_start_available)
    {
        shift_warm_start();
        admm_config.warm_start = 1;
        /* Save the shifted warm-start's first control as fallback
         * in case the solver doesn't converge this cycle. */
        fallback_control = prev_predicted_controls[0];
    }
    else
    {
        admm_config.warm_start = 0;
    }

    /* Build the multistage QP */
    build_qp_problem(current_state, &qp_problem);

#ifdef MPCC_DEBUG_PRINT
    {
        /* Print B matrix coupling for steering at k=0 */
        printf("  [DBG-QP] k=0 B[1][δ]=%.6f B[2][δ]=%.6f B[3][δ]=%.6f B[0][vθ]=%.6f B[1][ax]=%.6f\n",
               (double)(qp_problem.dynamics[0].B[1][0]),
               (double)(qp_problem.dynamics[0].B[2][0]),
               (double)(qp_problem.dynamics[0].B[3][0]),
               (double)(qp_problem.dynamics[0].B[0][2]),
               (double)(qp_problem.dynamics[0].B[1][1]));
        /* Print key A matrix entries at k=0 */
        printf("  [DBG-QP] k=0 A[1][1]=%.4f A[1][2]=%.4f A[1][3]=%.4f A[6][3]=%.4f\n",
               (double)(qp_problem.dynamics[0].A[1][1]),
               (double)(qp_problem.dynamics[0].A[1][2]),
               (double)(qp_problem.dynamics[0].A[1][3]),
               (double)(qp_problem.dynamics[0].A[6][3]));
        /* Print affine term d */
        printf("  [DBG-QP] k=0 d[0]=%.4f d[1]=%.4f d[2]=%.4f d[3]=%.4f\n",
               (double)(qp_problem.dynamics[0].d[0]),
               (double)(qp_problem.dynamics[0].d[1]),
               (double)(qp_problem.dynamics[0].d[2]),
               (double)(qp_problem.dynamics[0].d[3]));
        /* Print cost weights */
        printf("  [DBG-QP] Q[vx]=%.1f Q[vy]=%.1f Q[ω]=%.1f q[vx]=%.1f R[δ]=%.1f R[ax]=%.1f r[vθ]=%.1f\n",
               (double)(qp_problem.stage_cost[0].Q[1][1]),
               (double)(qp_problem.stage_cost[0].Q[2][2]),
               (double)(qp_problem.stage_cost[0].Q[3][3]),
               (double)(qp_problem.stage_cost[0].q[1]),
               (double)(qp_problem.stage_cost[0].R[0][0]),
               (double)(qp_problem.stage_cost[0].R[1][1]),
               (double)(qp_problem.stage_cost[0].r[2]));
        /* Print rate penalty linear terms (r values) */
        printf("  [DBG-QP] r[δ]=%.4f r[ax]=%.4f r[vθ]=%.4f\n",
               (double)(qp_problem.stage_cost[0].r[0]),
               (double)(qp_problem.stage_cost[0].r[1]),
               (double)(qp_problem.stage_cost[0].r[2]));
        /* Print track bounds */
        printf("  [DBG-QP] track_left[0]=%.2f track_right[0]=%.2f\n",
               (double)(qp_problem.track_left[0]),
               (double)(qp_problem.track_right[0]));
        /* Print initial state */
        printf("  [DBG-QP] x0: s=%.3f vx=%.3f vy=%.4f ω=%.4f X=%.2f Y=%.2f ψ=%.3f\n",
               (double)(qp_problem.x0[0]),
               (double)(qp_problem.x0[1]),
               (double)(qp_problem.x0[2]),
               (double)(qp_problem.x0[3]),
               (double)(qp_problem.x0[4]),
               (double)(qp_problem.x0[5]),
               (double)(qp_problem.x0[6]));
    }
#endif

    /* Solve via ADMM + Riccati */
    ADMMResult_t admm_result;
    MPCCStatus_t status = admm_solver_solve(
        &qp_problem, &admm_config, &admm_workspace, &admm_result);

    /* Extract first control input */
    result->status = status;
    result->admm_iterations = admm_result.iterations;
    result->primal_residual = admm_result.primal_residual;
    result->dual_residual = admm_result.dual_residual;
    result->rho_final = admm_result.rho_final;
    result->rho_u_final = admm_result.rho_u_final;
    result->adaptive_rho_updates = admm_result.adaptive_rho_updates;
    result->numeric_clip_count = admm_result.numeric_clip_count;

    if (status == MPCC_STATUS_SUCCESS || status == MPCC_STATUS_MAX_ITERATIONS) {
        /* Use the latest ADMM iterate for control (same policy as MPC).
         * MAX_ITERATIONS is still a usable solution in practice. */
        result->optimal_control.delta = admm_result.u_opt[0][MPCC_IDX_DELTA];
        result->optimal_control.a_x = admm_result.u_opt[0][MPCC_IDX_AX];
        result->optimal_control.v_theta = admm_result.u_opt[0][MPCC_IDX_VTHETA];

        /* Copy predicted trajectory to result */
        uint16_t N = config.horizon_steps;
        for (uint16_t k = 0; k <= N; k++)
            array_to_state(admm_result.x_opt[k], &result->predicted_states[k]);
        for (uint16_t k = 0; k < N; k++)
        {
            result->predicted_controls[k].delta = admm_result.u_opt[k][MPCC_IDX_DELTA];
            result->predicted_controls[k].a_x = admm_result.u_opt[k][MPCC_IDX_AX];
            result->predicted_controls[k].v_theta = admm_result.u_opt[k][MPCC_IDX_VTHETA];
        }

        /* Store solution for next cycle's warm start */
        for (uint16_t k = 0; k <= N; k++)
            prev_predicted_states[k] = result->predicted_states[k];
        for (uint16_t k = 0; k < N; k++)
            prev_predicted_controls[k] = result->predicted_controls[k];

        prev_control = result->optimal_control;
    } else {
        /* Infeasible/error: fall back to shifted warm-start control, but
         * still update warm-start buffers with the latest ADMM iterate. */
        result->optimal_control = fallback_control;

        /* Copy ADMM predicted trajectory for warm-start AND diagnostics */
        uint16_t N = config.horizon_steps;
        for (uint16_t k = 0; k <= N; k++)
            array_to_state(admm_result.x_opt[k], &result->predicted_states[k]);
        for (uint16_t k = 0; k < N; k++)
        {
            result->predicted_controls[k].delta = admm_result.u_opt[k][MPCC_IDX_DELTA];
            result->predicted_controls[k].a_x = admm_result.u_opt[k][MPCC_IDX_AX];
            result->predicted_controls[k].v_theta = admm_result.u_opt[k][MPCC_IDX_VTHETA];
        }

        /* Update warm-start with ADMM output (approximate but fresh) */
        for (uint16_t k = 0; k <= N; k++)
            prev_predicted_states[k] = result->predicted_states[k];
        for (uint16_t k = 0; k < N; k++)
            prev_predicted_controls[k] = result->predicted_controls[k];
    }
    warm_start_available = 1;

#ifdef MPCC_DEBUG_PRINT
        printf("[MPCC] status=%d  iter=%u  prim=%.4f  dual=%.4f  "
            "rho=%.3f  rho_u=%.3f  rho_upd=%u  clip=%u  "
            "delta=%.3f  a_x=%.3f  v_theta=%.3f  s=%.2f  vx=%.3f\n",
           status, admm_result.iterations,
           (double)(admm_result.primal_residual),
           (double)(admm_result.dual_residual),
            (double)(admm_result.rho_final),
            (double)(admm_result.rho_u_final),
            admm_result.adaptive_rho_updates,
            admm_result.numeric_clip_count,
           (double)(result->optimal_control.delta),
           (double)(result->optimal_control.a_x),
           (double)(result->optimal_control.v_theta),
           (double)(result->predicted_states[0].s),
           (double)(result->predicted_states[0].vx));
    /* Print predicted trajectory: vx, delta for first 5 stages */
    {
        uint16_t N = config.horizon_steps;
        uint16_t print_n = N < 5 ? N : 5;
        printf("  [Traj] vx: ");
        for (uint16_t k = 0; k <= print_n; k++)
            printf("%.3f ", (double)(result->predicted_states[k].vx));
        printf("\n  [Traj] δ: ");
        for (uint16_t k = 0; k < print_n; k++)
            printf("%.4f ", (double)(result->predicted_controls[k].delta));
        printf("\n  [Traj] ax: ");
        for (uint16_t k = 0; k < print_n; k++)
            printf("%.4f ", (double)(result->predicted_controls[k].a_x));
        printf("\n");
        /* Print Riccati K gains at k=0 for steering → states */
        printf("  [K] K[δ,s]=%.6f K[δ,vx]=%.6f K[δ,vy]=%.6f K[δ,ω]=%.6f K[δ,X]=%.6f K[δ,Y]=%.6f K[δ,ψ]=%.6f\n",
               (double)(admm_workspace.K[0][0][0]),
               (double)(admm_workspace.K[0][0][1]),
               (double)(admm_workspace.K[0][0][2]),
               (double)(admm_workspace.K[0][0][3]),
               (double)(admm_workspace.K[0][0][4]),
               (double)(admm_workspace.K[0][0][5]),
               (double)(admm_workspace.K[0][0][6]));
        printf("  [K] K[ax,s]=%.6f K[ax,vx]=%.6f K[ax,vy]=%.6f\n",
               (double)(admm_workspace.K[0][1][0]),
               (double)(admm_workspace.K[0][1][1]),
               (double)(admm_workspace.K[0][1][2]));
        printf("  [K] kk[δ]=%.6f kk[ax]=%.6f kk[vθ]=%.6f\n",
               (double)(admm_workspace.kk[0][0]),
               (double)(admm_workspace.kk[0][1]),
               (double)(admm_workspace.kk[0][2]));
    }
#endif

    return status;
}

/*===========================================================================
 * Configuration Access
 *===========================================================================*/

MPCCConfiguration_t mpcc_get_configuration(void)
{
    return config;
}

void mpcc_set_configuration(const MPCCConfiguration_t *cfg)
{
    if (!cfg)
        return;

    config = *cfg;
    sanitize_config(&config);
    admm_config.rho = cfg->admm_rho;
    admm_config.max_iterations = cfg->admm_max_iterations;
    admm_config.eps_primal = cfg->admm_tolerance;
    admm_config.eps_dual = cfg->admm_tolerance;
}

void mpcc_reset(void)
{
    warm_start_available = 0;
    last_closest_idx = 0; /* Reset path tracking */
    memset(&prev_control, 0, sizeof(prev_control));
    admm_solver_initialize(&admm_workspace);
}
