/**
 * @file mpcc.c
 * @brief Model Predictive Contouring Control — Implementation
 *        (Lifted ODE: Frenet primary + Cartesian redundant)
 *
 * Implements the MPCC controller using the Lifted ODE formulation:
 *
 * Algorithm (each control cycle):
 *   1. Warm-start: shift previous solution by one time step
 *   2. For each stage k = 0..N-1:
 *      a. Interpolate path at predicted s to get kappa(s), phi(s)
 *      b. Linearize Lifted ODE (Frenet + Cartesian) dynamics
 *      c. Build stage cost (n/alpha penalties, progress reward)
 *      d. Set per-stage track bounds on n from path
 *      e. Linearize obstacle constraints (if active)
 *   3. Solve multistage QP via ADMM + Riccati
 *   4. Extract first control input
 *
 * Frenet dynamics linearization:
 *   The 10-state Lifted ODE is linearized via Forward Euler + Jacobians:
 *
 *   Frenet block (7x7 + 7x2):
 *     ds/dt      = (vx*cos(alpha) - vy*sin(alpha)) / (1 - n*kappa)
 *     dn/dt      = vx*sin(alpha) + vy*cos(alpha)
 *     dalpha/dt  = omega - kappa * ds/dt
 *     [vx, vy, omega] dynamics with linear tires
 *
 *   Cartesian block (3x7 coupling + 3x3):
 *     dX/dt  = vx*cos(psi) - vy*sin(psi)
 *     dY/dt  = vx*sin(psi) + vy*cos(psi)
 *     dpsi/dt= omega
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

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

/** QP problem structure (reused each call) */
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
    cfg.weight_n = MPCC_DEFAULT_WEIGHT_N;
    cfg.weight_alpha = MPCC_DEFAULT_WEIGHT_ALPHA;
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
    cfg.weight_n_terminal = MPCC_DEFAULT_WEIGHT_N_TERMINAL;
    cfg.weight_alpha_terminal = MPCC_DEFAULT_WEIGHT_ALPHA_TERMINAL;
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
    cfg.n_max = MPCC_DEFAULT_N_MAX;
    cfg.v_theta_max = MPCC_DEFAULT_V_THETA_MAX;
    cfg.v_theta_min = MPCC_DEFAULT_V_THETA_MIN;

    /* Linear tire model */
    cfg.mu   = MPCC_DEFAULT_MU;
    cfg.C_Sf = MPCC_DEFAULT_C_SF;
    cfg.C_Sr = MPCC_DEFAULT_C_SR;

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
 * Obstacle Management
 *===========================================================================*/

void mpcc_set_obstacles(const MPCCObstacleSet_t *obstacles)
{
    obstacle_set = *obstacles;
}

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
    fixed_point_t c = fp_cos(obs->phi);
    fixed_point_t s = fp_sin(obs->phi);
    fixed_point_t c2 = fp_mul(c, c);
    fixed_point_t s2 = fp_mul(s, s);
    fixed_point_t cs = fp_mul(c, s);

    /* 1/a^2 and 1/b^2 */
    fixed_point_t ia2 = fp_div(FP_ONE, fp_mul(obs->a, obs->a));
    fixed_point_t ib2 = fp_div(FP_ONE, fp_mul(obs->b, obs->b));

    obs->Sigma_inv[0][0] = fp_add(fp_mul(c2, ia2), fp_mul(s2, ib2));
    obs->Sigma_inv[0][1] = fp_mul(cs, fp_sub(ia2, ib2));
    obs->Sigma_inv[1][0] = obs->Sigma_inv[0][1];
    obs->Sigma_inv[1][1] = fp_add(fp_mul(s2, ia2), fp_mul(c2, ib2));
}

/*===========================================================================
 * Path Interpolation
 *===========================================================================*/

/** Wrap s to [0, total_length) for closed paths. */
static fixed_point_t wrap_s(fixed_point_t s, fixed_point_t total_length)
{
    while (s >= total_length)
        s = fp_sub(s, total_length);
    while (s < 0)
        s = fp_add(s, total_length);
    return s;
}

void mpcc_path_interpolate(
    const MPCCReferencePath_t *path,
    fixed_point_t s,
    MPCCPathPoint_t *result)
{
    if (path->num_points < 2) {
        memset(result, 0, sizeof(*result));
        return;
    }

    /* Clamp s to valid range */
    fixed_point_t s_min = path->points[0].s_ref;
    fixed_point_t s_max = path->points[path->num_points - 1].s_ref;

    if (path->is_closed && s_max > s_min) {
        /* Wrap s into [s_min, s_max) for closed paths */
        fixed_point_t len = fp_sub(s_max, s_min);
        while (s > s_max) s = fp_sub(s, len);
        while (s < s_min) s = fp_add(s, len);
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
    fixed_point_t ds = fp_sub(p1->s_ref, p0->s_ref);
    fixed_point_t t;
    if (ds <= 0) {
        t = 0;
    } else {
        t = fp_div(fp_sub(s, p0->s_ref), ds);
    }
    fixed_point_t one_minus_t = fp_sub(FP_ONE, t);

    /* Linear interpolation: result = (1-t)*p0 + t*p1 */
    result->x_ref       = fp_add(fp_mul(one_minus_t, p0->x_ref),       fp_mul(t, p1->x_ref));
    result->y_ref       = fp_add(fp_mul(one_minus_t, p0->y_ref),       fp_mul(t, p1->y_ref));
    result->kappa_ref   = fp_add(fp_mul(one_minus_t, p0->kappa_ref),   fp_mul(t, p1->kappa_ref));
    result->left_bound  = fp_add(fp_mul(one_minus_t, p0->left_bound),  fp_mul(t, p1->left_bound));
    result->right_bound = fp_add(fp_mul(one_minus_t, p0->right_bound), fp_mul(t, p1->right_bound));
    result->vx_ref      = fp_add(fp_mul(one_minus_t, p0->vx_ref),     fp_mul(t, p1->vx_ref));
    result->s_ref       = s;

    /* Angle interpolation: handle wrapping for phi_ref */
    fixed_point_t dphi = fp_normalize_angle(fp_sub(p1->phi_ref, p0->phi_ref));
    result->phi_ref = fp_normalize_angle(fp_add(p0->phi_ref, fp_mul(t, dphi)));
}

/*===========================================================================
 * Find Closest s
 *===========================================================================*/

fixed_point_t mpcc_find_closest_s(
    const MPCCReferencePath_t *path,
    fixed_point_t X,
    fixed_point_t Y)
{
    if (path->num_points < 1) return 0;

    /* Brute-force closest-point search.
     * Use int64 for distance-squared to avoid Q16.16 overflow. */
    int64_t best_dist_sq = INT64_MAX;
    uint16_t best_idx = 0;

    for (uint16_t i = 0; i < path->num_points; i++) {
        int64_t dx = (int64_t)X - (int64_t)path->points[i].x_ref;
        int64_t dy = (int64_t)Y - (int64_t)path->points[i].y_ref;
        int64_t dist_sq = dx * dx + dy * dy;  /* Q32.32, but only need relative comparison */
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
        }
    }

    return path->points[best_idx].s_ref;
}

/* Find closest s near a hint to preserve longitudinal continuity. */
static fixed_point_t mpcc_find_closest_s_with_hint(
    const MPCCReferencePath_t *path,
    fixed_point_t X,
    fixed_point_t Y,
    fixed_point_t s_hint)
{
    if (path->num_points < 1) return 0;
    if (path->num_points < 8 || s_hint <= 0)
        return mpcc_find_closest_s(path, X, Y);

    fixed_point_t s_min = path->points[0].s_ref;
    fixed_point_t s_max = path->points[path->num_points - 1].s_ref;
    fixed_point_t s_query = s_hint;
    if (path->is_closed && s_max > s_min) {
        fixed_point_t len = fp_sub(s_max, s_min);
        while (s_query > s_max) s_query = fp_sub(s_query, len);
        while (s_query < s_min) s_query = fp_add(s_query, len);
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
    int64_t best_dist_sq = INT64_MAX;
    uint16_t best_idx = center;
    for (int off = -window; off <= window; off++) {
        int idx = (int)center + off;
        if (path->is_closed) {
            while (idx < 0) idx += path->num_points;
            while (idx >= (int)path->num_points) idx -= path->num_points;
        } else {
            if (idx < 0 || idx >= (int)path->num_points) continue;
        }
        int64_t dx = (int64_t)X - (int64_t)path->points[idx].x_ref;
        int64_t dy = (int64_t)Y - (int64_t)path->points[idx].y_ref;
        int64_t dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = (uint16_t)idx;
        }
    }

    /* Local search failed (far from path): fall back to global search. */
    {
        int64_t max_dist = (int64_t)FP_CONST(3.0);
        int64_t max_dist_sq = max_dist * max_dist;
        if (best_dist_sq > max_dist_sq)
            return mpcc_find_closest_s(path, X, Y);
    }

    return path->points[best_idx].s_ref;
}

/* MPC-style forward-biased closest-waypoint search with heading penalty. */
static uint16_t mpcc_find_closest_index_forward_biased(
    const MPCCReferencePath_t *path,
    fixed_point_t X,
    fixed_point_t Y,
    fixed_point_t psi,
    uint16_t start_idx)
{
    if (path->num_points == 0) return 0;

    const int search_forward = 200;
    const int search_backward = 100; /* Increased for recovery after segment loss */
    uint16_t best_idx = start_idx;
    int64_t best_score = INT64_MAX;

    fixed_point_t veh_dx = fp_cos(psi);
    fixed_point_t veh_dy = fp_sin(psi);
    fixed_point_t behind_penalty = FP_CONST(2.0);

    for (int off = -search_backward; off < search_forward; off++) {
        int idx = (int)start_idx + off;
        if (path->is_closed) {
            while (idx < 0) idx += path->num_points;
            while (idx >= (int)path->num_points) idx -= path->num_points;
        } else {
            if (idx < 0 || idx >= (int)path->num_points) continue;
        }

        fixed_point_t dx = fp_sub(path->points[idx].x_ref, X);
        fixed_point_t dy = fp_sub(path->points[idx].y_ref, Y);

        int64_t dist_sq = (int64_t)dx * (int64_t)dx + (int64_t)dy * (int64_t)dy;
        fixed_point_t dot = fp_add(fp_mul(dx, veh_dx), fp_mul(dy, veh_dy));
        int64_t score = dist_sq;
        if (dot < 0) score += ((int64_t)behind_penalty * (int64_t)behind_penalty);

        if (score < best_score) {
            best_score = score;
            best_idx = (uint16_t)idx;
        }
    }
    return best_idx;
}

/* Project onto segment [idx0, idx1] like MPC does, then compute s on segment. */
static fixed_point_t mpcc_project_s_on_segment(
    const MPCCReferencePath_t *path,
    uint16_t idx0,
    uint16_t idx1,
    fixed_point_t X,
    fixed_point_t Y)
{
    const MPCCPathPoint_t *p0 = &path->points[idx0];
    const MPCCPathPoint_t *p1 = &path->points[idx1];

    fixed_point_t ax = p0->x_ref, ay = p0->y_ref;
    fixed_point_t bx = p1->x_ref, by = p1->y_ref;
    fixed_point_t abx = fp_sub(bx, ax);
    fixed_point_t aby = fp_sub(by, ay);
    fixed_point_t apx = fp_sub(X, ax);
    fixed_point_t apy = fp_sub(Y, ay);
    fixed_point_t ab_len2 = fp_add(fp_mul(abx, abx), fp_mul(aby, aby));

    fixed_point_t t = 0;
    if (ab_len2 > FP_CONST(1e-9))
        t = fp_div(fp_add(fp_mul(apx, abx), fp_mul(apy, aby)), ab_len2);
    if (t < 0) t = 0;
    if (t > FP_ONE) t = FP_ONE;

    return fp_add(p0->s_ref, fp_mul(t, fp_sub(p1->s_ref, p0->s_ref)));
}

/*===========================================================================
 * Frenet-Cartesian Conversion
 *===========================================================================*/

void mpcc_frenet_to_cartesian(
    fixed_point_t s,
    fixed_point_t n,
    fixed_point_t alpha,
    fixed_point_t *X,
    fixed_point_t *Y,
    fixed_point_t *psi)
{
    /* Inverse Frenet transform:
     *   X   = gamma_x(s) - n * sin(phi_gamma(s))
     *   Y   = gamma_y(s) + n * cos(phi_gamma(s))
     *   psi = phi_gamma(s) + alpha
     */
    MPCCPathPoint_t pt;
    mpcc_path_interpolate(&ref_path, s, &pt);

    fixed_point_t sin_phi = fp_sin(pt.phi_ref);
    fixed_point_t cos_phi = fp_cos(pt.phi_ref);

    *X = fp_sub(pt.x_ref, fp_mul(n, sin_phi));
    *Y = fp_add(pt.y_ref, fp_mul(n, cos_phi));
    *psi = fp_add(pt.phi_ref, alpha);
}

MPCCState_t mpcc_state_from_vehicle_state(
    const VehicleState_t *vs,
    fixed_point_t s_hint)
{
    /* Use module-level last_closest_idx — reset via mpcc_set_reference_path/mpcc_reset */
    MPCCState_t st;
    memset(&st, 0, sizeof(st));

    /* Copy Cartesian states directly */
    st.X = vs->position_x_meters;
    st.Y = vs->position_y_meters;
    st.psi = vs->heading_angle_radians;
    st.vx = vs->longitudinal_velocity_meters_per_second;
    st.vy = vs->lateral_velocity_meters_per_second;
    st.omega = vs->yaw_rate_radians_per_second;
    /* omega_w removed — not needed with a_x control */

    /* Compute Frenet states from Cartesian + path.
     * Use continuity-preserving local search around s_hint to avoid
     * jumps between nearby segments in hairpins/self-near track regions. */
    /* MPC-style closest waypoint search: forward-biased + heading penalty */
    {
        if (last_closest_idx >= ref_path.num_points) last_closest_idx = 0;
        uint16_t idx0 = mpcc_find_closest_index_forward_biased(
            &ref_path, st.X, st.Y, st.psi, last_closest_idx);
        uint16_t idx1 = (uint16_t)(idx0 + 1);
        if (idx1 >= ref_path.num_points) idx1 = ref_path.is_closed ? 0 : (ref_path.num_points - 1);
        last_closest_idx = idx0;
        st.s = mpcc_project_s_on_segment(&ref_path, idx0, idx1, st.X, st.Y);
    }

    /* Interpolate path at s */
    MPCCPathPoint_t pt;
    mpcc_path_interpolate(&ref_path, st.s, &pt);

    /* n: signed lateral distance */
    fixed_point_t dx = fp_sub(st.X, pt.x_ref);
    fixed_point_t dy = fp_sub(st.Y, pt.y_ref);
    fixed_point_t sin_phi = fp_sin(pt.phi_ref);
    fixed_point_t cos_phi = fp_cos(pt.phi_ref);
    st.n = fp_add(fp_mul(fp_sub(0, sin_phi), dx), fp_mul(cos_phi, dy));

    /* alpha: heading error */
    st.alpha = fp_normalize_angle(fp_sub(st.psi, pt.phi_ref));

    return st;
}

fixed_point_t mpcc_compute_progress_rate(
    const MPCCState_t *state,
    fixed_point_t kappa)
{
    /* ds/dt = (vx*cos(alpha) - vy*sin(alpha)) / (1 - n*kappa) */
    fixed_point_t cos_a = fp_cos(state->alpha);
    fixed_point_t sin_a = fp_sin(state->alpha);

    fixed_point_t v_proj = fp_sub(
        fp_mul(state->vx, cos_a),
        fp_mul(state->vy, sin_a));

    fixed_point_t denom = fp_sub(FP_ONE, fp_mul(state->n, kappa));

    /* Guard against division by zero (n*kappa ~ 1 means singularity) */
    if (denom == 0)
        denom = FP_CONST(0.001);

    return fp_div(v_proj, denom);
}


/*===========================================================================
 * Cost Construction (Lifted ODE)
 *===========================================================================*/

/**
 * Build stage cost matrices for the Lifted ODE formulation.
 *
 * Cost at each stage:
 *   l_k = q_n * n^2 + q_alpha * alpha^2
 *       + q_vy * vy^2 + q_omega * omega^2
 *       + q_vx * (vx - vx_ref)^2
 *       - q_s * s                  (linear progress reward)
 *       + u^T R u                  (control effort)
 *
 * @param stage_cost  Output cost structure
 * @param is_terminal  Whether this is the terminal stage (N)
 */
static void build_stage_cost(
    MPCCStageCost_t *cost,
    uint8_t is_terminal)
{
    memset(cost, 0, sizeof(*cost));

    /* Quadratic state costs (diagonal) */
    if (is_terminal)
    {
        cost->Q[MPCC_IDX_N][MPCC_IDX_N] = config.weight_n_terminal;
        cost->Q[MPCC_IDX_ALPHA][MPCC_IDX_ALPHA] = config.weight_alpha_terminal;
    }
    else
    {
        cost->Q[MPCC_IDX_N][MPCC_IDX_N] = config.weight_n;
        cost->Q[MPCC_IDX_ALPHA][MPCC_IDX_ALPHA] = config.weight_alpha;
    }

    /* State regularization (same for running and terminal) */
    cost->Q[MPCC_IDX_VY][MPCC_IDX_VY] = config.weight_vy;
    cost->Q[MPCC_IDX_OMEGA][MPCC_IDX_OMEGA] = config.weight_omega;

    /* Velocity tracking: q_vx * (vx - vx_ref)^2
     * = q_vx * vx^2 - 2*q_vx*vx_ref*vx + const
     * Q[VX][VX] = q_vx,  q[VX] = -2*q_vx*vx_ref */
    if (config.weight_vx > 0)
    {
        cost->Q[MPCC_IDX_VX][MPCC_IDX_VX] = config.weight_vx;
        cost->q[MPCC_IDX_VX] = fp_sub(0,
            fp_mul(FP_CONST(2.0), fp_mul(config.weight_vx, config.vx_ref)));
    }

    /* Progress reward: -q_theta * v_theta (linear term on v_theta control).
     * Terminal stage: still reward being far along via linear cost on s. */
    if (is_terminal)
    {
        fixed_point_t q_s = config.weight_progress_terminal;
        cost->q[MPCC_IDX_S] = fp_sub(0, q_s); /* negative = reward */
    }

    /* Control costs (not used for terminal stage) */
    if (!is_terminal)
    {
        fixed_point_t q_progress = config.weight_progress;
        cost->r[MPCC_IDX_VTHETA] = fp_sub(0, q_progress); /* negative = reward */

        cost->R[MPCC_IDX_DELTA][MPCC_IDX_DELTA] =
            fp_add(config.weight_delta, config.weight_delta_rate);
        cost->R[MPCC_IDX_AX][MPCC_IDX_AX] =
            fp_add(config.weight_ax, config.weight_ax_rate);
        cost->R[MPCC_IDX_VTHETA][MPCC_IDX_VTHETA] =
            fp_add(config.weight_v_theta, config.weight_v_theta_rate);
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
    qp->x0[MPCC_IDX_N]       = x0->n;
    qp->x0[MPCC_IDX_ALPHA]   = x0->alpha;
    qp->x0[MPCC_IDX_VX]      = x0->vx;
    qp->x0[MPCC_IDX_VY]      = x0->vy;
    qp->x0[MPCC_IDX_OMEGA]   = x0->omega;
    qp->x0[MPCC_IDX_X]       = x0->X;
    qp->x0[MPCC_IDX_Y]       = x0->Y;
    qp->x0[MPCC_IDX_PSI]     = x0->psi;

    /* --- Global box constraints --- */

    /* State bounds (use large values for unbounded) */
    fixed_point_t big = FP_CONST(1000.0);
    for (int i = 0; i < MPCC_NX; i++)
    {
        qp->x_lower[i] = fp_sub(0, big);
        qp->x_upper[i] = big;
    }

    /* s: must be non-negative */
    qp->x_lower[MPCC_IDX_S] = 0;

    /* n: bounded by track width (default, overridden per-stage below) */
    qp->x_lower[MPCC_IDX_N] = fp_sub(0, config.n_max);
    qp->x_upper[MPCC_IDX_N] = config.n_max;

    /* vx: bounded */
    qp->x_lower[MPCC_IDX_VX] = config.vx_min;
    qp->x_upper[MPCC_IDX_VX] = config.vx_max;

    /* Control bounds */
    qp->u_lower[MPCC_IDX_DELTA] = fp_sub(0, config.delta_max);
    qp->u_upper[MPCC_IDX_DELTA] = config.delta_max;
    qp->u_lower[MPCC_IDX_AX] = config.ax_min;
    qp->u_upper[MPCC_IDX_AX] = config.ax_max;
    qp->u_lower[MPCC_IDX_VTHETA] = config.v_theta_min;
    qp->u_upper[MPCC_IDX_VTHETA] = config.v_theta_max;

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
        u_bar.v_theta = FP_CONST(1.0);
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
        mpcc_linearize_dynamics(&z_bar, &u_bar, path_pt.kappa_ref,
                          config.dt, &config, &qp->dynamics[k]);

        /* Build stage cost */
        build_stage_cost(&qp->stage_cost[k], 0);

        /* Override vx_ref with the per-stage value from the raceline velocity
         * profile so the solver naturally slows before turns. */
        if (path_pt.vx_ref > 0 && config.weight_vx > 0) {
            qp->stage_cost[k].q[MPCC_IDX_VX] = fp_sub(0,
                fp_mul(FP_CONST(2.0), fp_mul(config.weight_vx, path_pt.vx_ref)));
#ifdef MPCC_DEBUG_PRINT
            if (k == 0) {
                printf("  [QP] s_bar=%.2f vx_ref=%.3f q_vx=%.1f Q_vx=%.1f\n",
                       FP_TO_DOUBLE(z_bar.s), FP_TO_DOUBLE(path_pt.vx_ref),
                       FP_TO_DOUBLE(qp->stage_cost[k].q[MPCC_IDX_VX]),
                       FP_TO_DOUBLE(qp->stage_cost[k].Q[MPCC_IDX_VX][MPCC_IDX_VX]));
            }
#endif
        }

        /* Rate penalty cross-term: penalize (u_k - u_ref_k)^2.
         * The quadratic part (weight_rate * u_k^2) is already in R.
         * Add the linear cross-term: r[i] += -2 * weight_rate * u_ref[i].
         * u_ref comes from shifted warm-start (or zero for cold start).
         * This creates proper rate penalization that allows steady-state
         * steering without penalty but penalizes oscillation. */
        {
            MPCCControl_t u_ref;
            if (warm_start_available)
                u_ref = prev_predicted_controls[k];
            else {
                u_ref.delta = 0;
                u_ref.a_x = 0;
                u_ref.v_theta = FP_CONST(1.0);
            }

            qp->stage_cost[k].r[MPCC_IDX_DELTA] = fp_sub(
                qp->stage_cost[k].r[MPCC_IDX_DELTA],
                fp_mul(FP_CONST(2.0),
                    fp_mul(config.weight_delta_rate, u_ref.delta)));

            qp->stage_cost[k].r[MPCC_IDX_AX] = fp_sub(
                qp->stage_cost[k].r[MPCC_IDX_AX],
                fp_mul(FP_CONST(2.0),
                    fp_mul(config.weight_ax_rate, u_ref.a_x)));

            qp->stage_cost[k].r[MPCC_IDX_VTHETA] = fp_sub(
                qp->stage_cost[k].r[MPCC_IDX_VTHETA],
                fp_mul(FP_CONST(2.0),
                    fp_mul(config.weight_v_theta_rate, u_ref.v_theta)));
        }

        /* Per-stage track bounds on n */
        qp->track_left[k] = path_pt.left_bound;
        qp->track_right[k] = path_pt.right_bound;

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
        build_stage_cost(&qp->terminal_cost, 1);
        if (path_pt.vx_ref > 0 && config.weight_vx > 0) {
            qp->terminal_cost.q[MPCC_IDX_VX] = fp_sub(0,
                fp_mul(FP_CONST(2.0), fp_mul(config.weight_vx, path_pt.vx_ref)));
        }
    }
}

/*===========================================================================
 * Warm Start Management
 *===========================================================================*/

static void state_to_array(const MPCCState_t *st, fixed_point_t arr[MPCC_NX]);

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
               sizeof(fixed_point_t) * MPCC_NX);
    }
    for (uint16_t k = 0; k < N; k++)
    {
        admm_workspace.z_u[k][MPCC_IDX_DELTA] = prev_predicted_controls[k].delta;
        admm_workspace.z_u[k][MPCC_IDX_AX]    = prev_predicted_controls[k].a_x;
        admm_workspace.z_u[k][MPCC_IDX_VTHETA] = prev_predicted_controls[k].v_theta;
        memcpy(admm_workspace.w_u[k], admm_workspace.z_u[k],
               sizeof(fixed_point_t) * MPCC_NU);
    }

    /* Shift dual variables (lambda) forward to preserve ADMM convergence
     * history.  Zeroing lambda defeats warm-starting — ADMM needs the
     * accumulated constraint-violation information to converge properly. */
    for (uint16_t k = 0; k < N; k++) {
        memcpy(admm_workspace.lambda_x[k], admm_workspace.lambda_x[k + 1],
               sizeof(fixed_point_t) * MPCC_NX);
        if (k + 1 < N)
            memcpy(admm_workspace.lambda_u[k], admm_workspace.lambda_u[k + 1],
                   sizeof(fixed_point_t) * MPCC_NU);
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

static void state_to_array(const MPCCState_t *st, fixed_point_t arr[MPCC_NX])
{
    arr[0] = st->s;     arr[1] = st->n;      arr[2] = st->alpha;
    arr[3] = st->vx;    arr[4] = st->vy;     arr[5] = st->omega;
    arr[6] = st->X;     arr[7] = st->Y;      arr[8] = st->psi;
}

static void array_to_state(const fixed_point_t arr[MPCC_NX], MPCCState_t *st)
{
    st->s = arr[0];     st->n = arr[1];       st->alpha = arr[2];
    st->vx = arr[3];    st->vy = arr[4];      st->omega = arr[5];
    st->X = arr[6];     st->Y = arr[7];       st->psi = arr[8];
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
    MPCCControl_t fallback_control = {0, 0, FP_CONST(1.0)};
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
        printf("  [DBG-QP] k=0 B[1][δ]=%.6f B[4][δ]=%.6f B[5][δ]=%.6f B[0][vθ]=%.6f B[2][vθ]=%.6f B[3][ax]=%.6f\n",
               FP_TO_DOUBLE(qp_problem.dynamics[0].B[1][0]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].B[4][0]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].B[5][0]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].B[0][2]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].B[2][2]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].B[3][1]));
        /* Print key A matrix entries at k=0 */
        printf("  [DBG-QP] k=0 A[1][2]=%.4f A[1][3]=%.4f A[1][4]=%.4f A[2][5]=%.4f\n",
               FP_TO_DOUBLE(qp_problem.dynamics[0].A[1][2]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].A[1][3]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].A[1][4]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].A[2][5]));
        /* Print affine term d[1] (n offset) */
        printf("  [DBG-QP] k=0 d[0]=%.4f d[1]=%.4f d[2]=%.4f d[3]=%.4f\n",
               FP_TO_DOUBLE(qp_problem.dynamics[0].d[0]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].d[1]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].d[2]),
               FP_TO_DOUBLE(qp_problem.dynamics[0].d[3]));
        /* Print cost weights */
        printf("  [DBG-QP] Q[n]=%.1f Q[α]=%.1f Q[vx]=%.1f q[vx]=%.1f R[δ]=%.1f R[ax]=%.1f r[vθ]=%.1f\n",
               FP_TO_DOUBLE(qp_problem.stage_cost[0].Q[1][1]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].Q[2][2]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].Q[3][3]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].q[3]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].R[0][0]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].R[1][1]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].r[2]));
        /* Print rate penalty linear terms (r values) */
        printf("  [DBG-QP] r[δ]=%.4f r[ax]=%.4f r[vθ]=%.4f\n",
               FP_TO_DOUBLE(qp_problem.stage_cost[0].r[0]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].r[1]),
               FP_TO_DOUBLE(qp_problem.stage_cost[0].r[2]));
        /* Print track bounds */
        printf("  [DBG-QP] track_left[0]=%.2f track_right[0]=%.2f  n_bound_global=%.2f\n",
               FP_TO_DOUBLE(qp_problem.track_left[0]),
               FP_TO_DOUBLE(qp_problem.track_right[0]),
               FP_TO_DOUBLE(qp_problem.x_upper[1]));
        /* Print initial state */
        printf("  [DBG-QP] x0: s=%.3f n=%.4f α=%.4f vx=%.3f vy=%.4f ω=%.4f\n",
               FP_TO_DOUBLE(qp_problem.x0[0]),
               FP_TO_DOUBLE(qp_problem.x0[1]),
               FP_TO_DOUBLE(qp_problem.x0[2]),
               FP_TO_DOUBLE(qp_problem.x0[3]),
               FP_TO_DOUBLE(qp_problem.x0[4]),
               FP_TO_DOUBLE(qp_problem.x0[5]));
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
            "delta=%.3f  a_x=%.3f  v_theta=%.3f  s=%.2f  n=%.3f\n",
           status, admm_result.iterations,
           FP_TO_DOUBLE(admm_result.primal_residual),
           FP_TO_DOUBLE(admm_result.dual_residual),
            FP_TO_DOUBLE(admm_result.rho_final),
            FP_TO_DOUBLE(admm_result.rho_u_final),
            admm_result.adaptive_rho_updates,
            admm_result.numeric_clip_count,
           FP_TO_DOUBLE(result->optimal_control.delta),
           FP_TO_DOUBLE(result->optimal_control.a_x),
           FP_TO_DOUBLE(result->optimal_control.v_theta),
           FP_TO_DOUBLE(result->predicted_states[0].s),
           FP_TO_DOUBLE(result->predicted_states[0].n));
    /* Print predicted trajectory: n, vx, delta for first 5 stages */
    {
        uint16_t N = config.horizon_steps;
        uint16_t print_n = N < 5 ? N : 5;
        printf("  [Traj] n: ");
        for (uint16_t k = 0; k <= print_n; k++)
            printf("%.4f ", FP_TO_DOUBLE(result->predicted_states[k].n));
        printf("\n  [Traj] vx: ");
        for (uint16_t k = 0; k <= print_n; k++)
            printf("%.3f ", FP_TO_DOUBLE(result->predicted_states[k].vx));
        printf("\n  [Traj] δ: ");
        for (uint16_t k = 0; k < print_n; k++)
            printf("%.4f ", FP_TO_DOUBLE(result->predicted_controls[k].delta));
        printf("\n  [Traj] ax: ");
        for (uint16_t k = 0; k < print_n; k++)
            printf("%.4f ", FP_TO_DOUBLE(result->predicted_controls[k].a_x));
        printf("\n");
        /* Print Riccati K gains at k=0 for steering → lateral states */
        printf("  [K] K[δ,s]=%.6f K[δ,n]=%.6f K[δ,α]=%.6f K[δ,vx]=%.6f K[δ,vy]=%.6f K[δ,ω]=%.6f\n",
               FP_TO_DOUBLE(admm_workspace.K[0][0][0]),
               FP_TO_DOUBLE(admm_workspace.K[0][0][1]),
               FP_TO_DOUBLE(admm_workspace.K[0][0][2]),
               FP_TO_DOUBLE(admm_workspace.K[0][0][3]),
               FP_TO_DOUBLE(admm_workspace.K[0][0][4]),
               FP_TO_DOUBLE(admm_workspace.K[0][0][5]));
        printf("  [K] K[ax,s]=%.6f K[ax,n]=%.6f K[ax,vx]=%.6f\n",
               FP_TO_DOUBLE(admm_workspace.K[0][1][0]),
               FP_TO_DOUBLE(admm_workspace.K[0][1][1]),
               FP_TO_DOUBLE(admm_workspace.K[0][1][3]));
        printf("  [K] kk[δ]=%.6f kk[ax]=%.6f kk[vθ]=%.6f\n",
               FP_TO_DOUBLE(admm_workspace.kk[0][0]),
               FP_TO_DOUBLE(admm_workspace.kk[0][1]),
               FP_TO_DOUBLE(admm_workspace.kk[0][2]));
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
