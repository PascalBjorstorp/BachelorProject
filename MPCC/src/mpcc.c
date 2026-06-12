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
 *   3. Solve multistage QP with the selected backend
 *   4. Extract first control input
 *
 * Dynamics:
 *   ds/dt     = v_theta                         (virtual control)
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
#include <stdio.h>
#include "mpcc_types.h"
#include "mpcc.h"
#include "mpcc_vehicle_model.h"
#include "qp_solver_mpcc.h"
#ifdef USE_OSQP
#include "qp_solver_osqp.h"
#endif
#include <string.h>
#include <stdlib.h>


#ifndef MPCC_SQP_REFINEMENT_STEPS
#define MPCC_SQP_REFINEMENT_STEPS 1
#endif

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MPCCConfiguration_t config;
static MPCCReferencePath_t ref_path;

/* Path tracking state — reset when path changes or controller resets */
static uint16_t last_closest_idx = 0;
static float last_unwrapped_heading = 0.0f;
static uint8_t unwrapped_heading_available = 0;
static MPCCObstacleSet_t obstacle_set;
static uint8_t mpcc_initialized = 0;

/** Previous solution for warm-starting */
static MPCCState_t prev_predicted_states[MPCC_MAX_HORIZON + 1];
static MPCCControl_t prev_predicted_controls[MPCC_MAX_HORIZON];
static uint8_t warm_start_available = 0;

/** Previous applied control (for rate penalties) */
static MPCCControl_t prev_control;
static uint8_t prev_control_available = 0;

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
    cfg.weight_heading = MPCC_DEFAULT_WEIGHT_HEADING;
    cfg.weight_wall_clearance = MPCC_DEFAULT_WEIGHT_WALL_CLEARANCE;
    cfg.wall_clearance_margin = MPCC_DEFAULT_WALL_CLEARANCE_MARGIN;
    cfg.track_safety_buffer = MPCC_DEFAULT_TRACK_SAFETY_BUFFER;
    cfg.weight_progress = MPCC_DEFAULT_WEIGHT_PROGRESS;
    cfg.weight_physical_progress = MPCC_DEFAULT_WEIGHT_PHYSICAL_PROGRESS;
    cfg.s_qp_window = MPCC_DEFAULT_S_QP_WINDOW;

    /* State regularization */
    cfg.weight_vx = MPCC_DEFAULT_WEIGHT_VX;
    cfg.vx_ref = MPCC_DEFAULT_VX_REF;
    cfg.use_raceline_vx_ref = MPCC_DEFAULT_USE_RACELINE_VX_REF;
    cfg.use_raceline_vx_limit = MPCC_DEFAULT_USE_RACELINE_VX_LIMIT;
    cfg.raceline_vx_limit_scale = MPCC_DEFAULT_RACELINE_VX_LIMIT_SCALE;
    cfg.weight_vy = MPCC_DEFAULT_WEIGHT_VY;
    cfg.weight_omega = MPCC_DEFAULT_WEIGHT_OMEGA;

    /* Control effort */
    cfg.weight_delta = MPCC_DEFAULT_WEIGHT_DELTA;
    cfg.weight_ax = MPCC_DEFAULT_WEIGHT_AX;
    cfg.weight_v_theta = MPCC_DEFAULT_WEIGHT_V_THETA;
    cfg.weight_vtheta_physical = MPCC_DEFAULT_WEIGHT_VTHETA_PHYSICAL;

    /* Control rate */
    cfg.weight_delta_rate = MPCC_DEFAULT_WEIGHT_DELTA_RATE;
    cfg.weight_ax_rate = MPCC_DEFAULT_WEIGHT_AX_RATE;
    cfg.weight_v_theta_rate = MPCC_DEFAULT_WEIGHT_V_THETA_RATE;

    /* Terminal */
    cfg.weight_contouring_terminal = MPCC_DEFAULT_WEIGHT_CONTOURING_TERMINAL;
    cfg.weight_lag_terminal = MPCC_DEFAULT_WEIGHT_LAG_TERMINAL;
    cfg.weight_heading_terminal = MPCC_DEFAULT_WEIGHT_HEADING_TERMINAL;
    cfg.weight_progress_terminal = MPCC_DEFAULT_WEIGHT_PROGRESS_TERMINAL;

    /* Obstacle avoidance */
    cfg.weight_obstacle = MPCC_DEFAULT_WEIGHT_OBSTACLE;
    cfg.obstacle_margin = MPCC_DEFAULT_OBSTACLE_MARGIN;

    /* ADMM */
    cfg.admm_rho = MPCC_DEFAULT_ADMM_RHO;
    cfg.admm_max_iterations = MPCC_DEFAULT_ADMM_MAX_ITER;
    cfg.admm_tolerance = MPCC_DEFAULT_ADMM_TOLERANCE;
    cfg.admm_rho_u = MPCC_DEFAULT_ADMM_RHO_U;
    cfg.admm_adaptive_rho = MPCC_DEFAULT_ADMM_ADAPTIVE_RHO;
    cfg.admm_alpha_relax = MPCC_DEFAULT_ADMM_ALPHA_RELAX;
    cfg.accept_max_iterations = MPCC_DEFAULT_ACCEPT_MAX_ITERATIONS;
    cfg.max_iter_primal_tolerance = MPCC_DEFAULT_MAX_ITER_PRIMAL_TOL;
    cfg.max_iter_dual_tolerance = MPCC_DEFAULT_MAX_ITER_DUAL_TOL;
    cfg.max_iter_track_violation_tolerance = MPCC_DEFAULT_MAX_ITER_TRACK_VIOLATION_TOL;
    cfg.warm_start_max_s_error = MPCC_DEFAULT_WARM_START_MAX_S_ERROR;

    /* Constraint bounds */
    cfg.delta_max = F110_DEFAULT_MAXIMUM_STEERING_RADIANS;
    cfg.delta_rate_max = MPCC_DEFAULT_DELTA_RATE_MAX;
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
    if (cfg->weight_heading < 0.0f)
        cfg->weight_heading = 0.0f;
    if (cfg->weight_heading_terminal < 0.0f)
        cfg->weight_heading_terminal = 0.0f;
    if (cfg->weight_physical_progress < 0.0f)
        cfg->weight_physical_progress = 0.0f;
    if (cfg->weight_wall_clearance < 0.0f)
        cfg->weight_wall_clearance = 0.0f;
    if (cfg->wall_clearance_margin < 0.0f)
        cfg->wall_clearance_margin = 0.0f;
    if (cfg->track_safety_buffer < 0.0f)
        cfg->track_safety_buffer = 0.0f;
    if (cfg->delta_rate_max <= 0.0f)
        cfg->delta_rate_max = MPCC_DEFAULT_DELTA_RATE_MAX;
    if (cfg->cross_call_rate_scale <= 0.0f)
        cfg->cross_call_rate_scale = MPCC_DEFAULT_CROSS_CALL_SCALE;
    if (cfg->s_qp_window <= 0.0f)
        cfg->s_qp_window = MPCC_DEFAULT_S_QP_WINDOW;
    if (cfg->weight_vtheta_physical < 0.0f)
        cfg->weight_vtheta_physical = 0.0f;
    /* MPCC speed selection is purely emergent from progress, vehicle dynamics,
     * and the path corridor. Disable all reference-speed plumbing. */
    cfg->weight_vx = 0.0f;
    cfg->vx_ref = 0.0f;
    cfg->use_raceline_vx_ref = 0u;
    cfg->use_raceline_vx_limit = 0u;
    cfg->use_global_vx_limit = 0u;
    cfg->use_curvature_vx_limit = 0u;
    cfg->raceline_vx_limit_scale = 0.0f;
    if (cfg->admm_rho_u < 0.0f)
        cfg->admm_rho_u = 0.0f;
    cfg->admm_adaptive_rho = cfg->admm_adaptive_rho ? 1 : 0;
    if (cfg->admm_alpha_relax < 1.0f)
        cfg->admm_alpha_relax = 1.0f;
    if (cfg->admm_alpha_relax > 1.95f)
        cfg->admm_alpha_relax = 1.95f;
    cfg->accept_max_iterations = cfg->accept_max_iterations ? 1 : 0;
    if (cfg->max_iter_primal_tolerance <= 0.0f)
        cfg->max_iter_primal_tolerance = MPCC_DEFAULT_MAX_ITER_PRIMAL_TOL;
    if (cfg->max_iter_dual_tolerance <= 0.0f)
        cfg->max_iter_dual_tolerance = MPCC_DEFAULT_MAX_ITER_DUAL_TOL;
    if (cfg->max_iter_track_violation_tolerance < 0.0f)
        cfg->max_iter_track_violation_tolerance = 0.0f;
    if (cfg->warm_start_max_s_error <= 0.0f)
        cfg->warm_start_max_s_error = MPCC_DEFAULT_WARM_START_MAX_S_ERROR;
}

/*===========================================================================
 * Initialization
 *===========================================================================*/

void mpcc_initialize(void)
{
    config = get_default_config();

#ifndef USE_OSQP
    admm_solver_default_config(&admm_config);
    admm_solver_initialize(&admm_workspace);
#else
    memset(&admm_config, 0, sizeof(admm_config));
    memset(&admm_workspace, 0, sizeof(admm_workspace));
#endif

    memset(&prev_control, 0, sizeof(prev_control));
    prev_control_available = 0;
    memset(&ref_path, 0, sizeof(ref_path));
    memset(&obstacle_set, 0, sizeof(obstacle_set));
    unwrapped_heading_available = 0;
    warm_start_available = 0;
    mpcc_initialized = 1;
}

void mpcc_initialize_with_config(const MPCCConfiguration_t *cfg)
{
    config = cfg ? *cfg : get_default_config();
    sanitize_config(&config);

#ifndef USE_OSQP
    admm_solver_default_config(&admm_config);
    admm_config.rho = config.admm_rho;
    admm_config.max_iterations = config.admm_max_iterations;
    admm_config.eps_primal = config.admm_tolerance;
    admm_config.eps_dual = config.admm_tolerance;
    admm_config.rho_u = config.admm_rho_u;
    admm_config.adaptive_rho = config.admm_adaptive_rho;
    admm_config.alpha_relax = config.admm_alpha_relax;
    admm_config.warm_start = 0;

    admm_solver_initialize(&admm_workspace);
#else
    memset(&admm_config, 0, sizeof(admm_config));
    memset(&admm_workspace, 0, sizeof(admm_workspace));
#endif

    memset(&prev_control, 0, sizeof(prev_control));
    prev_control_available = 0;
    memset(&obstacle_set, 0, sizeof(obstacle_set));
    unwrapped_heading_available = 0;
    warm_start_available = 0;
    mpcc_initialized = 1;
}

/*===========================================================================
 * Reference Path
 *===========================================================================*/

void mpcc_set_reference_path(const MPCCReferencePath_t *path)
{
    ref_path = *path;

    /* Precompute sin/cos for every stored waypoint so that
     * mpcc_path_interpolate() early-return paths return correct values
     * without an extra sinf/cosf call at each interpolation site. */
    for (uint16_t i = 0; i < ref_path.num_points; i++) {
        ref_path.points[i].sin_phi = sinf(ref_path.points[i].phi_ref);
        ref_path.points[i].cos_phi = cosf(ref_path.points[i].phi_ref);
    }

    warm_start_available = 0;
    memset(&prev_control, 0, sizeof(prev_control));
    prev_control_available = 0;
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
        if (s <= s_min) {
            *result = path->points[0];
            result->s_ref = s;
            result->sin_phi = sinf(result->phi_ref);
            result->cos_phi = cosf(result->phi_ref);
            return;
        }
        if (s >= s_max) {
            *result = path->points[path->num_points - 1];
            result->s_ref = s;
            result->sin_phi = sinf(result->phi_ref);
            result->cos_phi = cosf(result->phi_ref);
            return;
        }
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

    /* Precompute trig — amortises sinf/cosf over all callers of this function */
    result->sin_phi = sinf(result->phi_ref);
    result->cos_phi = cosf(result->phi_ref);
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
        while (s_query < s_min) s_query = s_query + len;
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

static float mpcc_normalize_s_delta(
    const MPCCReferencePath_t *path,
    float delta)
{
    if (!path->is_closed || path->total_length <= 0.0f)
        return delta;

    const float half_length = 0.5f * path->total_length;
    while (delta > half_length) delta -= path->total_length;
    while (delta < -half_length) delta += path->total_length;
    return delta;
}

static float mpcc_unwrap_s_near(
    const MPCCReferencePath_t *path,
    float s_wrapped,
    float s_hint)
{
    if (!path->is_closed || path->total_length <= 0.0f)
        return s_wrapped;

    return s_hint + mpcc_normalize_s_delta(path, s_wrapped - s_hint);
}

static float mpcc_limit_s_jump(
    const MPCCReferencePath_t *path,
    float s_reference,
    float s_candidate,
    float vx)
{
    float delta = mpcc_normalize_s_delta(path, s_candidate - s_reference);
    float max_jump = 0.75f + 2.0f * fabsf(vx) * config.dt;

    if (max_jump < 0.75f) max_jump = 0.75f;
    if (max_jump > 2.5f) max_jump = 2.5f;

    if (delta > max_jump) delta = max_jump;
    if (delta < -max_jump) delta = -max_jump;

    s_candidate = s_reference + delta;
    return s_candidate;
}

static uint16_t mpcc_warm_start_stage_advance(void)
{
    uint16_t N = config.horizon_steps;
    if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;

    if (N == 0)
        return 0;

    if (config.cross_call_rate_scale <= 0.0f)
        return 1;

    if (config.cross_call_rate_scale < 1.0f)
        return 0;

    uint16_t advance = (uint16_t)floorf(config.cross_call_rate_scale);
    if (advance < 1) advance = 1;
    if (advance > N) advance = N;
    return advance;
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

/*===========================================================================
 * Vehicle State → MPCC State Conversion
 *===========================================================================*/

MPCCState_t mpcc_state_from_vehicle_state(
    const VehicleState_t *vs,
    float s_hint)
{
    MPCCState_t st;
    memset(&st, 0, sizeof(st));

    /* Copy Cartesian/body-frame states directly */
    st.X     = vs->pos_x;
    st.Y     = vs->pos_y;
    if (!unwrapped_heading_available || !isfinite((double)last_unwrapped_heading)) {
        last_unwrapped_heading = vs->heading;
        unwrapped_heading_available = 1;
    } else {
        /* ROS yaw is wrapped to [-pi, pi]. Keep the MPCC state continuous so
         * crossing that boundary does not introduce a fictitious 2*pi jump
         * against the warm-started predicted trajectory. */
        float heading_delta = remainderf(
            vs->heading - last_unwrapped_heading, 2.0f * (float)M_PI);
        last_unwrapped_heading += heading_delta;
    }
    st.psi   = last_unwrapped_heading;
    st.vx    = vs->long_vel;
    st.vy    = vs->lat_vel;
    st.omega = vs->yaw_rate;

    /* Compute s by following the previous plan first and only using
     * geometry as a local correction/recovery signal. */
    {
        float s_anchor;
        float s_geom;

        if (ref_path.num_points == 0) {
            st.s = 0.0f;
            return st;
        }

        if (last_closest_idx >= ref_path.num_points) last_closest_idx = 0;

        /* Keep the forward-biased waypoint search for local index tracking. */
        uint16_t idx0 = mpcc_find_closest_index_forward_biased(
            &ref_path, st.X, st.Y, st.psi, last_closest_idx);
        uint16_t idx1 = (uint16_t)(idx0 + 1);
        if (idx1 >= ref_path.num_points)
            idx1 = ref_path.is_closed ? 0 : (ref_path.num_points - 1);

        if (warm_start_available && config.horizon_steps > 0) {
            uint16_t anchor_idx = mpcc_warm_start_stage_advance();
            s_anchor = prev_predicted_states[anchor_idx].s;
        }
        else if (s_hint > 0.0f)
            s_anchor = s_hint;
        else
            s_anchor = mpcc_find_closest_s(&ref_path, st.X, st.Y);

        s_geom = mpcc_find_closest_s_with_hint(&ref_path, st.X, st.Y, s_anchor);
        s_geom = mpcc_unwrap_s_near(&ref_path, s_geom, s_anchor);

        {
            /* signed_delta > 0: geometry is AHEAD of anchor (possible sensor jump).
             * signed_delta < 0: anchor is AHEAD of geometry (virtual-progress runaway).
             * The original symmetric abs(diff) suppressed correction in both directions
             * equally, so a runaway anchor could never be pulled back by geometry.
             * Fixed: only damp forward sensor jumps; allow geometry to correct runaway. */
            float signed_delta = mpcc_normalize_s_delta(&ref_path, s_geom - s_anchor);
            float abs_delta = fabsf(signed_delta);

#ifdef MPCC_DIAG_S_ANCHOR
            printf("[MPCC-S] s_anchor=%.3f s_geom=%.3f signed_delta=%+.3f\n",
                   (double)s_anchor, (double)s_geom, (double)signed_delta);
#endif

            if (abs_delta < 0.75f) {
                st.s = s_geom;
            } else if (signed_delta >= 0.0f) {
                /* Geometry jumped forward past anchor — damp to prevent false progress. */
                if (abs_delta < 2.0f)
                    st.s = s_anchor + 0.25f * signed_delta;
                else
                    st.s = s_anchor;
            } else {
                /* Anchor overshot geometry (virtual-progress runaway).
                 * Pull s back toward geometry so QP stays anchored on the car. */
                if (abs_delta < 3.0f)
                    st.s = s_anchor + 0.75f * signed_delta;
                else
                    st.s = s_geom;
            }
        }

        st.s = mpcc_limit_s_jump(&ref_path, s_anchor, st.s, st.vx);
        if (!ref_path.is_closed) {
            if (st.s < 0.0f) st.s = 0.0f;
            if (st.s > ref_path.total_length) st.s = ref_path.total_length;
        }

        /* Update search locality after the final s decision. */
        if (ref_path.num_points > 0) {
            last_closest_idx = mpcc_find_closest_index_forward_biased(
                &ref_path, st.X, st.Y, st.psi, idx0);
        } else {
            last_closest_idx = 0;
        }
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
    cost->Q[MPCC_IDX_VY][MPCC_IDX_VY] = 2.0f * config.weight_vy;
    cost->Q[MPCC_IDX_OMEGA][MPCC_IDX_OMEGA] = 2.0f * config.weight_omega;

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
            2.0f * (config.weight_delta + w_dr);
        cost->R[MPCC_IDX_AX][MPCC_IDX_AX] =
            2.0f * (config.weight_ax + w_ar);
        cost->R[MPCC_IDX_VTHETA][MPCC_IDX_VTHETA] =
            2.0f * (config.weight_v_theta + w_vr);
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

    /* Path quantities at operating point — sin/cos precomputed in mpcc_path_interpolate */
    float sin_phi = path_pt->sin_phi;
    float cos_phi = path_pt->cos_phi;
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

    g_l[MPCC_IDX_S] = (kappa * e_c_bar) + 1.0f;  /* de_l/ds = kappa * e_c + 1 */
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

static void add_heading_alignment_cost(
    MPCCStageCost_t *cost,
    const MPCCState_t *z_bar,
    const MPCCPathPoint_t *path_pt,
    float weight)
{
    if (!cost || !z_bar || !path_pt || weight <= 0.0f)
        return;

    float alpha_bar =
        remainderf(z_bar->psi - path_pt->phi_ref, 2.0f * (float)M_PI);

    float g[MPCC_NX];
    float x_bar[MPCC_NX] = {
        z_bar->s,
        z_bar->vx,
        z_bar->vy,
        z_bar->omega,
        z_bar->X,
        z_bar->Y,
        z_bar->psi
    };

    memset(g, 0, sizeof(g));
    /* Keep the path tangent fixed for this one-SQP-step heading penalty.
     * Contouring/lag already manage virtual s; coupling heading directly to s
     * can make the horizon jump toward a different tangent in sharp corners. */
    g[MPCC_IDX_PSI] = 1.0f;

    float g_xbar = 0.0f;
    for (int i = 0; i < MPCC_NX; i++)
        g_xbar += g[i] * x_bar[i];

    float d = alpha_bar - g_xbar;

    for (int i = 0; i < MPCC_NX; i++) {
        if (g[i] == 0.0f) continue;
        for (int j = 0; j < MPCC_NX; j++) {
            if (g[j] == 0.0f) continue;
            cost->Q[i][j] += 2.0f * weight * g[i] * g[j];
        }
        cost->q[i] += 2.0f * weight * d * g[i];
    }
}

static void add_physical_progress_reward(
    MPCCStageCost_t *cost,
    const MPCCState_t *z_bar,
    const MPCCPathPoint_t *path_pt,
    float weight,
    float kappa_lookahead)
{
    if (!cost || !z_bar || !path_pt || weight <= 0.0f)
        return;

    /* Curvature-aware progress reward.
     *
     * A uniform progress reward must stay low enough that it never overpowers
     * the contouring cost at the sharpest corner (otherwise the optimizer
     * chooses "go straight = more progress" and understeers into the wall).
     * That cap leaves the straights underpowered, where there is plenty of
     * unused grip.  So we keep the configured `weight` as the corner-floor
     * value (the known-safe level) and boost it where the path is straight.
     *
     *   w_eff = weight * (1 + PROG_STRAIGHT_BOOST * exp(-kappa_la / PROG_KAPPA0))
     *
     * kappa_lookahead is the MAX |kappa| over a short braking window ahead of
     * this stage, not the local curvature.  Using the lookahead value keeps the
     * boost from firing on a straight that feeds directly into a corner (the
     * s~22.6 over-commit case): as soon as a corner is within braking range the
     * reward decays back to the corner floor and the car stays slow enough to
     * make the turn.  No new config knob: Q_PHYSICAL_PROGRESS sets the floor.
     */
    static const float PROG_STRAIGHT_BOOST = 1.0f;   /* clear straights get ~2x  */
    static const float PROG_KAPPA0         = 0.35f;  /* decay scale (rad/m)      */
    weight *= 1.0f + PROG_STRAIGHT_BOOST * expf(-kappa_lookahead / PROG_KAPPA0);

    float alpha = remainderf(z_bar->psi - path_pt->phi_ref,
                             2.0f * (float)M_PI);
    float cos_a = cosf(alpha);
    float sin_a = sinf(alpha);
    float dvpath_dangle = (-z_bar->vx * sin_a) - (z_bar->vy * cos_a);

    float g[MPCC_NX];
    memset(g, 0, sizeof(g));

    /* v_path = vx*cos(alpha) - vy*sin(alpha), alpha = psi - phi_ref(s). */
    g[MPCC_IDX_VX] = cos_a;
    g[MPCC_IDX_VY] = -sin_a;
    g[MPCC_IDX_PSI] = dvpath_dangle;

    for (int i = 0; i < MPCC_NX; i++) {
        if (g[i] == 0.0f) continue;
        cost->q[i] -= weight * g[i];
    }
}

static void add_wall_clearance_cost(
    MPCCStageCost_t *cost,
    const MPCCState_t *z_bar,
    const MPCCPathPoint_t *path_pt,
    float wall_weight,
    float wall_margin)
{
    if (wall_weight <= 0.0f || wall_margin <= 0.0f)
        return;

    float left_bound = path_pt->left_bound;
    float right_bound = path_pt->right_bound;

    float corridor_width = left_bound + right_bound;
    if (corridor_width <= 0.0f)
        return;

    /* sin/cos precomputed in mpcc_path_interpolate */
    float sin_phi = path_pt->sin_phi;
    float cos_phi = path_pt->cos_phi;
    float kappa   = path_pt->kappa_ref;

    float dX = (z_bar->X - path_pt->x_ref);
    float dY = (z_bar->Y - path_pt->y_ref);
    float e_c_bar = sin_phi * dX - cos_phi * dY;
    float e_l_bar = -(cos_phi * dX + sin_phi * dY);

    float g_c[MPCC_NX];
    memset(g_c, 0, sizeof(g_c));
    g_c[MPCC_IDX_S] = (0 - (kappa * e_l_bar));
    g_c[MPCC_IDX_X] = sin_phi;
    g_c[MPCC_IDX_Y] = (0 - cos_phi);

    float x_bar[MPCC_NX] = {
        z_bar->s,
        z_bar->vx, z_bar->vy, z_bar->omega,
        z_bar->X, z_bar->Y, z_bar->psi
    };
    float gc_xbar = 0.0f;
    for (int i = 0; i < MPCC_NX; i++)
        gc_xbar += g_c[i] * x_bar[i];
    float d_c = e_c_bar - gc_xbar;

    /* Helper for quadratic costs on contouring error targets:
     * weight * (e_c - target)^2, with e_c linearized as g_c*x + d_c. */
    #define MPCC_ADD_EC_TARGET_COST(weight_, target_)                                      \
        do {                                                                               \
            float mpcc_ec_weight_ = (weight_);                                             \
            if (mpcc_ec_weight_ > 0.0f) {                                                  \
                float mpcc_d_target_ = d_c - (target_);                                    \
                for (int mpcc_i_ = 0; mpcc_i_ < MPCC_NX; mpcc_i_++) {                     \
                    if (g_c[mpcc_i_] == 0.0f) continue;                                    \
                    for (int mpcc_j_ = 0; mpcc_j_ < MPCC_NX; mpcc_j_++) {                 \
                        if (g_c[mpcc_j_] == 0.0f) continue;                                \
                        cost->Q[mpcc_i_][mpcc_j_] +=                                       \
                            2.0f * mpcc_ec_weight_ * g_c[mpcc_i_] * g_c[mpcc_j_];         \
                    }                                                                      \
                    cost->q[mpcc_i_] += 2.0f * mpcc_ec_weight_ * mpcc_d_target_ *         \
                                       g_c[mpcc_i_];                                       \
                }                                                                          \
            }                                                                              \
        } while (0)

    /* Define a soft inner corridor. Outside it, penalize the projected
     * distance back into the band. If the corridor is tighter than the
     * requested margin, collapse to corridor centering. */
    float safe_lower = (0 - left_bound) + wall_margin;
    float safe_upper = right_bound - wall_margin;
    float target = e_c_bar;
    float stage_weight = wall_weight;
    float narrow_deficit = 0.0f;
    float corridor_center = 0.5f * (right_bound - left_bound);
    float left_slack = left_bound + e_c_bar;
    float right_slack = right_bound - e_c_bar;
    float min_slack = (left_slack < right_slack) ? left_slack : right_slack;

    if (left_bound < wall_margin)
        narrow_deficit = wall_margin - left_bound;
    if ((right_bound < wall_margin) && ((wall_margin - right_bound) > narrow_deficit))
        narrow_deficit = wall_margin - right_bound;

    if (narrow_deficit > 0.0f) {
        stage_weight = wall_weight * (1.0f + (narrow_deficit / wall_margin));
        if (stage_weight > 3.0f * wall_weight)
            stage_weight = 3.0f * wall_weight;
    }

    /* Future wall commits happen while the horizon is still technically inside
     * the hard corridor but has already spent most of its lateral slack. Add a
     * soft recentering term that only activates when the stage is close to one
     * wall, leaving the optimizer free elsewhere. */
    {
        float activation_slack = 3.0f * wall_margin;
        float half_width = 0.5f * corridor_width;
        if (activation_slack > half_width)
            activation_slack = half_width;
        if (activation_slack < wall_margin)
            activation_slack = wall_margin;

        if (min_slack < activation_slack) {
            float ratio = (activation_slack - min_slack) / activation_slack;
            float center_weight = stage_weight * ratio * ratio;

            if (corridor_width < (4.0f * wall_margin)) {
                float width_ratio = (4.0f * wall_margin - corridor_width) /
                                    (2.0f * wall_margin);
                if (width_ratio > 0.0f)
                    center_weight *= (1.0f + width_ratio);
            }

            MPCC_ADD_EC_TARGET_COST(center_weight, corridor_center);
        }
    }

    if (safe_lower <= safe_upper)
    {
        if (e_c_bar < safe_lower) {
            target = safe_lower;
        } else if (e_c_bar > safe_upper) {
            target = safe_upper;
        } else {
            return;
        }
    }
    else
    {
        target = 0.5f * (right_bound - left_bound);

        float deficit = (2.0f * wall_margin) - corridor_width;
        if (deficit > 0.0f)
            stage_weight = wall_weight * (1.0f + (deficit / wall_margin));

        if (fabsf(e_c_bar - target) < 0.005f)
            return;
    }

    MPCC_ADD_EC_TARGET_COST(stage_weight, target);
    #undef MPCC_ADD_EC_TARGET_COST
}

static void mpcc_tightened_track_bounds(
    const MPCCPathPoint_t *path_pt,
    float *left,
    float *right)
{
    float left_b = path_pt ? path_pt->left_bound : 0.0f;
    float right_b = path_pt ? path_pt->right_bound : 0.0f;

    left_b -= config.track_safety_buffer;
    right_b -= config.track_safety_buffer;

    if ((left_b + right_b) < 0.0f) {
        float lower = -left_b;
        float upper = right_b;
        float center = 0.5f * (lower + upper);
        left_b = -center;
        right_b = center;
    }

    if (left_b < 0.0f) left_b = 0.0f;
    if (right_b < 0.0f) right_b = 0.0f;

    if (left) *left = left_b;
    if (right) *right = right_b;
}

static void mpcc_set_stage_s_bounds(
    MPCCQPProblem_t *qp,
    uint16_t k,
    float s_center)
{
    float s_window = config.s_qp_window;
    if (s_window <= 0.0f)
        s_window = MPCC_DEFAULT_S_QP_WINDOW;

    float s_lower = s_center - s_window;
    float s_upper = s_center + s_window;

    if (!ref_path.is_closed && ref_path.total_length > 0.0f) {
        if (s_lower < 0.0f) s_lower = 0.0f;
        if (s_upper > ref_path.total_length) s_upper = ref_path.total_length;
    }

    qp->s_ref_stage[k] = s_center;
    qp->s_lower_stage[k] = s_lower;
    qp->s_upper_stage[k] = s_upper;
}

static void add_vtheta_physical_cost(
    MPCCStageCost_t *cost,
    const MPCCState_t *z_bar,
    const MPCCControl_t *u_bar,
    const MPCCPathPoint_t *path_pt,
    float weight)
{
    if (!cost || !z_bar || !u_bar || !path_pt || weight <= 0.0f)
        return;

    float alpha = z_bar->psi - path_pt->phi_ref;
    float cos_a = cosf(alpha);
    float sin_a = sinf(alpha);
    float v_path = z_bar->vx * cos_a - z_bar->vy * sin_a;
    float h_bar = u_bar->v_theta - v_path;
    float gx[MPCC_NX];
    float gu[MPCC_NU];
    float x_bar[MPCC_NX] = {
        z_bar->s,
        z_bar->vx,
        z_bar->vy,
        z_bar->omega,
        z_bar->X,
        z_bar->Y,
        z_bar->psi
    };
    float u_arr[MPCC_NU] = {
        u_bar->delta,
        u_bar->a_x,
        u_bar->v_theta
    };

    memset(gx, 0, sizeof(gx));
    memset(gu, 0, sizeof(gu));

    {
        float dvpath_dangle = (-z_bar->vx * sin_a) - (z_bar->vy * cos_a);
        gx[MPCC_IDX_S] = path_pt->kappa_ref * dvpath_dangle;
        gx[MPCC_IDX_VX] = -cos_a;
        gx[MPCC_IDX_VY] = sin_a;
        gx[MPCC_IDX_PSI] = -dvpath_dangle;
        gu[MPCC_IDX_VTHETA] = 1.0f;
    }

    float gx_xbar = 0.0f;
    float gu_ubar = 0.0f;
    for (int i = 0; i < MPCC_NX; i++)
        gx_xbar += gx[i] * x_bar[i];
    for (int i = 0; i < MPCC_NU; i++)
        gu_ubar += gu[i] * u_arr[i];

    float d = h_bar - gx_xbar - gu_ubar;

    for (int i = 0; i < MPCC_NX; i++) {
        if (gx[i] == 0.0f) continue;
        for (int j = 0; j < MPCC_NX; j++) {
            if (gx[j] == 0.0f) continue;
            cost->Q[i][j] += 2.0f * weight * gx[i] * gx[j];
        }
        cost->q[i] += 2.0f * weight * d * gx[i];
    }

    for (int i = 0; i < MPCC_NU; i++) {
        if (gu[i] == 0.0f) continue;
        for (int j = 0; j < MPCC_NU; j++) {
            if (gu[j] == 0.0f) continue;
            cost->R[i][j] += 2.0f * weight * gu[i] * gu[j];
        }
        for (int j = 0; j < MPCC_NX; j++) {
            if (gx[j] == 0.0f) continue;
            cost->S[i][j] += 2.0f * weight * gu[i] * gx[j];
        }
        cost->r[i] += 2.0f * weight * d * gu[i];
    }
}

/*===========================================================================
 * QP Diagnostics and Reachability Bounds
 *===========================================================================*/

#define MPCC_QP_DIAG_MAX_DIM (MPCC_NX + MPCC_NU)

static float mpcc_effective_control_dt(void)
{
    float control_dt = config.cross_call_rate_scale * config.dt;

    if (!isfinite((double)control_dt) || control_dt <= 0.0f)
        control_dt = config.dt;
    if (!isfinite((double)control_dt) || control_dt <= 0.0f)
        control_dt = MPCC_DEFAULT_DT;

    return control_dt;
}

static void mpcc_populate_delta_rate_bounds(MPCCQPProblem_t *qp, uint16_t N)
{
    float prev_delta = prev_control.delta;
    float rate_limit = config.delta_rate_max;
    const float global_lower = qp->u_lower[MPCC_IDX_DELTA];
    const float global_upper = qp->u_upper[MPCC_IDX_DELTA];
    const float control_dt = mpcc_effective_control_dt();

    if (!isfinite((double)prev_delta))
        prev_delta = 0.0f;
    if (prev_delta < global_lower) prev_delta = global_lower;
    if (prev_delta > global_upper) prev_delta = global_upper;

    if (!isfinite((double)rate_limit) || rate_limit <= 0.0f)
        rate_limit = MPCC_DEFAULT_DELTA_RATE_MAX;

    for (uint16_t k = 0; k < N; k++)
    {
        float elapsed = control_dt + ((float)k * config.dt);
        float lower;
        float upper;

        if (!isfinite((double)elapsed) || elapsed <= 0.0f)
            elapsed = control_dt;

        lower = prev_delta - (rate_limit * elapsed);
        upper = prev_delta + (rate_limit * elapsed);

        if (lower < global_lower) lower = global_lower;
        if (upper > global_upper) upper = global_upper;

        if (lower > upper)
        {
            lower = global_lower;
            upper = global_upper;
        }

        qp->delta_lower_stage[k] = lower;
        qp->delta_upper_stage[k] = upper;
    }
}

static float mpcc_jacobi_min_eigenvalue(
    float a[MPCC_QP_DIAG_MAX_DIM][MPCC_QP_DIAG_MAX_DIM],
    uint8_t n)
{
    for (uint8_t sweep = 0; sweep < 32u; sweep++)
    {
        uint8_t p = 0;
        uint8_t q = 1;
        float max_offdiag = 0.0f;

        for (uint8_t i = 0; i < n; i++)
        {
            for (uint8_t j = (uint8_t)(i + 1u); j < n; j++)
            {
                float offdiag = fabsf(a[i][j]);
                if (offdiag > max_offdiag)
                {
                    max_offdiag = offdiag;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_offdiag < 1e-6f)
            break;

        {
            const float app = a[p][p];
            const float aqq = a[q][q];
            const float apq = a[p][q];
            const float tau = (aqq - app) / (2.0f * apq);
            const float tau_sign = (tau >= 0.0f) ? 1.0f : -1.0f;
            const float t = tau_sign / (fabsf(tau) + sqrtf(1.0f + tau * tau));
            const float c = 1.0f / sqrtf(1.0f + t * t);
            const float s = t * c;

            for (uint8_t i = 0; i < n; i++)
            {
                if (i == p || i == q)
                    continue;

                {
                    const float aip = a[i][p];
                    const float aiq = a[i][q];
                    a[i][p] = (c * aip) - (s * aiq);
                    a[p][i] = a[i][p];
                    a[i][q] = (s * aip) + (c * aiq);
                    a[q][i] = a[i][q];
                }
            }

            a[p][p] = (c * c * app) - (2.0f * s * c * apq) + (s * s * aqq);
            a[q][q] = (s * s * app) + (2.0f * s * c * apq) + (c * c * aqq);
            a[p][q] = 0.0f;
            a[q][p] = 0.0f;
        }
    }

    {
        float min_eig = a[0][0];
        for (uint8_t i = 1; i < n; i++)
        {
            if (a[i][i] < min_eig)
                min_eig = a[i][i];
        }
        return min_eig;
    }
}

static float mpcc_stage_hessian_min_eigenvalue(const MPCCStageCost_t *cost,
                                               uint8_t include_controls)
{
    float hessian[MPCC_QP_DIAG_MAX_DIM][MPCC_QP_DIAG_MAX_DIM];
    const uint8_t n = include_controls
        ? (uint8_t)(MPCC_NX + MPCC_NU)
        : (uint8_t)MPCC_NX;

    memset(hessian, 0, sizeof(hessian));

    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
            hessian[i][j] = cost->Q[i][j];
    }

    if (include_controls)
    {
        for (int i = 0; i < MPCC_NU; i++)
        {
            for (int j = 0; j < MPCC_NU; j++)
                hessian[MPCC_NX + i][MPCC_NX + j] = cost->R[i][j];
            for (int j = 0; j < MPCC_NX; j++)
            {
                hessian[MPCC_NX + i][j] = cost->S[i][j];
                hessian[j][MPCC_NX + i] = cost->S[i][j];
            }
        }
    }

    for (uint8_t i = 0; i < n; i++)
    {
        for (uint8_t j = 0; j < n; j++)
        {
            if (!isfinite((double)hessian[i][j]))
                return -INFINITY;
        }
    }

    return mpcc_jacobi_min_eigenvalue(hessian, n);
}

static void mpcc_update_qp_diagnostics(const MPCCQPProblem_t *qp,
                                       MPCCResult_t *result)
{
    uint16_t N;
    float min_hessian;

    if (!qp || !result)
        return;

    N = qp->N;
    if (N > MPCC_MAX_HORIZON)
        N = MPCC_MAX_HORIZON;

    result->qp_diagnostic_flags = 0u;
    result->qp_min_hessian_eigenvalue = FLT_MAX;
    result->qp_min_track_width = FLT_MAX;
    result->qp_min_delta_width = FLT_MAX;
    result->qp_min_ax_limit = FLT_MAX;
    result->qp_min_vx_margin = FLT_MAX;

    for (uint16_t k = 0; k < N; k++)
    {
        const float track_width = qp->track_left[k] + qp->track_right[k];
        float delta_width = qp->delta_upper_stage[k] - qp->delta_lower_stage[k];
        const float accel_authority =
            fminf(fabsf(qp->u_lower[MPCC_IDX_AX]), fabsf(qp->u_upper[MPCC_IDX_AX]));
        const float vx_margin = qp->x_upper[MPCC_IDX_VX] - qp->x_lower[MPCC_IDX_VX];

        if (!isfinite((double)track_width) ||
            !isfinite((double)delta_width) ||
            !isfinite((double)accel_authority) ||
            !isfinite((double)vx_margin))
        {
            result->qp_diagnostic_flags |= MPCC_QP_DIAG_NONFINITE;
        }

        if (delta_width < 0.0f)
            delta_width = 0.0f;

        if (track_width < result->qp_min_track_width)
            result->qp_min_track_width = track_width;
        if (delta_width < result->qp_min_delta_width)
            result->qp_min_delta_width = delta_width;
        if (accel_authority < result->qp_min_ax_limit)
            result->qp_min_ax_limit = accel_authority;
        if (vx_margin < result->qp_min_vx_margin)
            result->qp_min_vx_margin = vx_margin;

        min_hessian = mpcc_stage_hessian_min_eigenvalue(&qp->stage_cost[k], 1);
        if (min_hessian < result->qp_min_hessian_eigenvalue)
            result->qp_min_hessian_eigenvalue = min_hessian;
    }

    {
        const float track_width = qp->track_left[N] + qp->track_right[N];
        const float vx_margin = qp->x_upper[MPCC_IDX_VX] - qp->x_lower[MPCC_IDX_VX];

        if (!isfinite((double)track_width) || !isfinite((double)vx_margin))
            result->qp_diagnostic_flags |= MPCC_QP_DIAG_NONFINITE;

        if (track_width < result->qp_min_track_width)
            result->qp_min_track_width = track_width;
        if (vx_margin < result->qp_min_vx_margin)
            result->qp_min_vx_margin = vx_margin;

        min_hessian = mpcc_stage_hessian_min_eigenvalue(&qp->terminal_cost, 0);
        if (min_hessian < result->qp_min_hessian_eigenvalue)
            result->qp_min_hessian_eigenvalue = min_hessian;
    }

    if (result->qp_min_hessian_eigenvalue < -1e-5f)
        result->qp_diagnostic_flags |= MPCC_QP_DIAG_HESSIAN_INDEFINITE;
    if (result->qp_min_track_width < 0.01f)
        result->qp_diagnostic_flags |= MPCC_QP_DIAG_TRACK_COLLAPSED;
    if (result->qp_min_delta_width < 1e-4f)
        result->qp_diagnostic_flags |= MPCC_QP_DIAG_DELTA_COLLAPSED;
    if (result->qp_min_ax_limit < 0.05f)
        result->qp_diagnostic_flags |= MPCC_QP_DIAG_AX_COLLAPSED;
    if (result->qp_min_vx_margin < 0.01f)
        result->qp_diagnostic_flags |= MPCC_QP_DIAG_VX_COLLAPSED;
}

/*===========================================================================
 * QP Problem Construction
 *===========================================================================*/

static void state_to_array(const MPCCState_t *st, float arr[MPCC_NX]);
static void array_to_state(const float arr[MPCC_NX], MPCCState_t *st);

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

    /* s: must be non-negative. For closed paths, let the horizon cross the
     * lap seam in unwrapped s-space; path interpolation already wraps stage
     * references back onto the track geometry. */
    qp->x_lower[MPCC_IDX_S] = 0;
    qp->x_upper[MPCC_IDX_S] =
        (ref_path.is_closed || ref_path.total_length <= 0.0f)
            ? big
            : ref_path.total_length;

    /* vx: bounded only by the global physical envelope */
    qp->x_lower[MPCC_IDX_VX] = config.vx_min;
    qp->x_upper[MPCC_IDX_VX] = config.vx_max;

    /* vy: physical lateral velocity bound [m/s] */
    qp->x_lower[MPCC_IDX_VY] = -5.0f;
    qp->x_upper[MPCC_IDX_VY] =  5.0f;

    /* omega: use a fixed physical bound. The previous speed-dependent grip
     * clamp introduced abrupt feasibility changes near high-speed corners. */
    qp->x_lower[MPCC_IDX_OMEGA] = -15.0f;
    qp->x_upper[MPCC_IDX_OMEGA] =  15.0f;

    /* Control bounds */
    qp->u_lower[MPCC_IDX_DELTA] = (0 - config.delta_max);
    qp->u_upper[MPCC_IDX_DELTA] = config.delta_max;
    qp->u_lower[MPCC_IDX_AX] = config.ax_min;
    qp->u_upper[MPCC_IDX_AX] = config.ax_max;
    qp->u_lower[MPCC_IDX_VTHETA] = config.v_theta_min;
    qp->u_upper[MPCC_IDX_VTHETA] = config.v_theta_max;

    mpcc_populate_delta_rate_bounds(qp, N);

    /* --- Build stages --- */
    /* Operating point: x0 for k=0; warm-start prev_predicted_states[k] for k>0
     * (set inside the loop — see `if (warm_start_available && k > 0)` below). */
    MPCCState_t z_bar = *x0;
    MPCCControl_t u_bar;

    if (warm_start_available)
        u_bar = prev_predicted_controls[0];
    else {
        u_bar.delta   = 0.0f;
        u_bar.a_x     = 0.0f;
        u_bar.v_theta = fminf(config.v_theta_max, fmaxf(1.0f, x0->vx));
    }

    for (uint16_t k = 0; k < N; k++)
    {
        /* --- Pick operating point for this stage --- */
        if (warm_start_available && k > 0)
        {
            /* Warm start: use the shifted previous solution.
             * Already correct — shift_warm_start() moved everything
             * forward by one step so index k matches stage k. */
            z_bar = prev_predicted_states[k];
            u_bar = prev_predicted_controls[k];
        }

        MPCCPathPoint_t path_pt;
        /* Keep stage geometry anchored to the MPCC arc-length state. */
        mpcc_path_interpolate(&ref_path, z_bar.s, &path_pt);

        /* --- ADD: store path geometry for ADMM track projection --- */
        qp->path_x_ref[k]   = path_pt.x_ref;
        qp->path_y_ref[k]   = path_pt.y_ref;
        qp->path_phi_ref[k] = path_pt.phi_ref;
        qp->path_sin_phi[k] = path_pt.sin_phi;
        qp->path_cos_phi[k] = path_pt.cos_phi;
        mpcc_set_stage_s_bounds(qp, k, z_bar.s);


        /* Linearize dynamics — produces A, B, d for this stage */
        mpcc_linearize_dynamics(&z_bar, &u_bar,
                                config.dt, &config, &qp->dynamics[k]);

        /* Build costs */
        build_stage_cost(&qp->stage_cost[k], 0, k);
        add_contouring_lag_cost(&qp->stage_cost[k], &z_bar, &path_pt,
                                config.weight_contouring, config.weight_lag);
        add_heading_alignment_cost(&qp->stage_cost[k], &z_bar, &path_pt,
                                   config.weight_heading);
        add_wall_clearance_cost(&qp->stage_cost[k], &z_bar, &path_pt,
                    config.weight_wall_clearance,
                    config.wall_clearance_margin);
        add_vtheta_physical_cost(&qp->stage_cost[k], &z_bar, &u_bar, &path_pt,
                                  config.weight_vtheta_physical);

        /* Max |kappa| over a braking window ahead, so the progress boost is
         * suppressed before a corner the car must slow for.  The window scales
         * with speed (braking distance grows with v) but never drops below a
         * fixed floor large enough to catch a corner from a standing start. */
        float kappa_lookahead = fabsf(path_pt.kappa_ref);
        {
            const float PROG_LOOKAHEAD_TS = 0.8f;  /* look-ahead time (s)      */
            const float PROG_LOOKAHEAD_MIN = 3.0f; /* floor at low speed (m)   */
            const float PROG_LOOKAHEAD_DS = 0.5f;  /* sample spacing (m)       */
            float la_m = PROG_LOOKAHEAD_TS * fmaxf(z_bar.vx, 0.0f);
            if (la_m < PROG_LOOKAHEAD_MIN) la_m = PROG_LOOKAHEAD_MIN;
            MPCCPathPoint_t la_pt;
            for (float ds = PROG_LOOKAHEAD_DS; ds <= la_m;
                 ds += PROG_LOOKAHEAD_DS) {
                mpcc_path_interpolate(&ref_path, z_bar.s + ds, &la_pt);
                float ak = fabsf(la_pt.kappa_ref);
                if (ak > kappa_lookahead) kappa_lookahead = ak;
            }
        }
        add_physical_progress_reward(&qp->stage_cost[k], &z_bar, &path_pt,
                                     config.weight_physical_progress,
                                     kappa_lookahead);

        /* Rate penalty cross-term: penalize (u_k - u_ref_k)^2.
         * The quadratic part (weight_rate * u_k^2) is already in R.
         * Add the linear cross-term: r[i] += -2 * weight_rate * u_ref[i].
         * u_ref comes from shifted warm-start (or zero for cold start).
         * This creates proper rate penalization that allows steady-state
         * steering without penalty but penalizes oscillation.
         * At step 0: scale rate weights by cross_call_rate_scale. */
        {
            MPCCControl_t u_ref;
            if (warm_start_available) {
                /* k=0: penalize deviation from the ACTUALLY applied control,
                 * not the shifted plan. After shift_warm_start(),
                 * prev_predicted_controls[0] holds what was planned for
                 * stage 1 last cycle - wrong reference for the rate penalty.
                 * prev_control always holds the last commanded input. */
                u_ref = (k == 0) ? prev_control : prev_predicted_controls[k];
            } else {
                u_ref.delta   = 0.0f;
                u_ref.a_x     = 0.0f;
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

        /* Per-stage track bounds in contouring-error coordinates. */
        mpcc_tightened_track_bounds(&path_pt, &qp->track_left[k],
                                    &qp->track_right[k]);

        if (!warm_start_available)
        {
            float x_arr[MPCC_NX], x_next[MPCC_NX];
            state_to_array(&z_bar, x_arr);

            float u_arr[MPCC_NU] = {
                u_bar.delta,
                u_bar.a_x,
                u_bar.v_theta
            };

            /* x_next = A·x + B·u + d */
            for (int i = 0; i < MPCC_NX; i++) {
                x_next[i] = qp->dynamics[k].d[i];          /* start with d */
                for (int j = 0; j < MPCC_NX; j++)
                    x_next[i] += qp->dynamics[k].A[i][j] * x_arr[j];
                for (int j = 0; j < MPCC_NU; j++)
                    x_next[i] += qp->dynamics[k].B[i][j] * u_arr[j];
            }

            array_to_state(x_next, &z_bar);   /* z_bar is now x_{k+1} */
            /* u_bar stays constant (zero-input rollout — simple & stable) */
        }
    }

    /* Terminal stage track bound */
    {
        MPCCState_t z_terminal;

        if (warm_start_available) {
            /* prev_predicted_states[N] is the terminal state from the
             * previous solve, shifted by shift_warm_start(). This is the
             * correct operating point for the terminal cost linearization. */
            z_terminal = prev_predicted_states[N];
        } else {
            /* Cold start: no previous trajectory. z_bar (stage N-1 op-point)
             * is the best approximation available. */
            z_terminal = z_bar;
        }

        MPCCPathPoint_t path_pt;
        mpcc_path_interpolate(&ref_path, z_terminal.s, &path_pt);

        mpcc_tightened_track_bounds(&path_pt, &qp->track_left[N],
                                    &qp->track_right[N]);

        /* --- terminal stage geometry --- */
        qp->path_x_ref[N]   = path_pt.x_ref;
        qp->path_y_ref[N]   = path_pt.y_ref;
        qp->path_phi_ref[N] = path_pt.phi_ref;
        qp->path_sin_phi[N] = path_pt.sin_phi;
        qp->path_cos_phi[N] = path_pt.cos_phi;
        mpcc_set_stage_s_bounds(qp, N, z_terminal.s);

        build_stage_cost(&qp->terminal_cost, 1, N);

        add_contouring_lag_cost(&qp->terminal_cost, &z_terminal, &path_pt,  // ✓ correct op-point
                                config.weight_contouring_terminal,
                                config.weight_lag_terminal);
        add_heading_alignment_cost(&qp->terminal_cost, &z_terminal, &path_pt,
                                   config.weight_heading_terminal);
        add_wall_clearance_cost(&qp->terminal_cost, &z_terminal, &path_pt,
                                config.weight_wall_clearance,
                                config.wall_clearance_margin);

    }
}

/*===========================================================================
 * Warm Start Management
 *===========================================================================*/

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

    /* Extrapolate terminal state: prev_predicted_states[N] is now stale
     * (the shift loop never updates index N). Propagate one step forward
     * from the new [N-1] using the linearized dynamics so the terminal
     * operating point used by build_qp_problem() is fresh each cycle.
     *
     *   x_N = A * x_{N-1} + B * u_{N-1} + d
     */
    {
        MPCCLinearSystem_t dyn_terminal;
        mpcc_linearize_dynamics(
            &prev_predicted_states[N - 1],    /* new [N-1] after shift */
            &prev_predicted_controls[N - 1],  /* held last control */
            config.dt, &config, &dyn_terminal);

        float x_arr[MPCC_NX];
        float x_next[MPCC_NX];
        state_to_array(&prev_predicted_states[N - 1], x_arr);

        float u_arr[MPCC_NU] = {
            prev_predicted_controls[N - 1].delta,
            prev_predicted_controls[N - 1].a_x,
            prev_predicted_controls[N - 1].v_theta
        };

        for (int i = 0; i < MPCC_NX; i++) {
            x_next[i] = dyn_terminal.d[i];                      /* affine offset */
            for (int j = 0; j < MPCC_NX; j++)
                x_next[i] += dyn_terminal.A[i][j] * x_arr[j];  /* A * x */
            for (int j = 0; j < MPCC_NU; j++)
                x_next[i] += dyn_terminal.B[i][j] * u_arr[j];  /* B * u */
        }

        {
            float s_geom = mpcc_find_closest_s_with_hint(
                &ref_path, x_next[MPCC_IDX_X], x_next[MPCC_IDX_Y],
                x_next[MPCC_IDX_S]);
            x_next[MPCC_IDX_S] =
                mpcc_unwrap_s_near(&ref_path, s_geom, x_next[MPCC_IDX_S]);
        }

        array_to_state(x_next, &prev_predicted_states[N]);      /* fresh x_N */
    }

    /* Copy shifted trajectory into ADMM workspace */
    for (uint16_t k = 0; k <= N; k++)
    {
        state_to_array(&prev_predicted_states[k], admm_workspace.z_x[k]);
        memcpy(admm_workspace.w_x[k], admm_workspace.z_x[k],
               sizeof(float) * MPCC_NX);
    }
    for (uint16_t k = 0; k < N; k++)
    {

        admm_workspace.z_u[k][MPCC_IDX_DELTA]  = prev_predicted_controls[k].delta;
        admm_workspace.z_u[k][MPCC_IDX_AX]     = prev_predicted_controls[k].a_x;
        admm_workspace.z_u[k][MPCC_IDX_VTHETA] = prev_predicted_controls[k].v_theta;
        memcpy(admm_workspace.w_u[k], admm_workspace.z_u[k],
               sizeof(float) * MPCC_NU);
    }

    /* Shift dual variables (lambda) forward */
    for (uint16_t k = 0; k < N; k++) {
        memcpy(admm_workspace.lambda_x[k], admm_workspace.lambda_x[k + 1],
               sizeof(float) * MPCC_NX);
        if (k + 1 < N)
            memcpy(admm_workspace.lambda_u[k], admm_workspace.lambda_u[k + 1],
                   sizeof(float) * MPCC_NU);
    }

    /* The shift leaves the terminal dual slots stale because no source
     * exists beyond the horizon. Clear those end slots here; the warm-start
     * solve path separately enforces lambda_x[0] = 0 for the fixed x0 state. */
    memset(admm_workspace.lambda_x[N], 0, sizeof(admm_workspace.lambda_x[N]));
    if (N > 0)
        memset(admm_workspace.lambda_u[N - 1], 0,
               sizeof(admm_workspace.lambda_u[N - 1]));
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

static void mpcc_solution_track_metrics(
    const MPCCQPProblem_t *prob,
    const ADMMResult_t *solver_result,
    float *max_violation_out,
    uint16_t *max_violation_stage_out,
    float *min_slack_out,
    uint16_t *min_slack_stage_out)
{
    float max_violation = 0.0f;
    float min_slack = FLT_MAX;
    uint16_t max_violation_stage = 0u;
    uint16_t min_slack_stage = 0u;

    if (max_violation_out) *max_violation_out = FLT_MAX;
    if (max_violation_stage_out) *max_violation_stage_out = 0u;
    if (min_slack_out) *min_slack_out = -FLT_MAX;
    if (min_slack_stage_out) *min_slack_stage_out = 0u;

    if (!prob || !solver_result)
        return;

    uint16_t N = prob->N;
    if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;

    for (uint16_t k = 0; k <= N; k++)
    {
        const float s = solver_result->x_opt[k][MPCC_IDX_S];
        const float X = solver_result->x_opt[k][MPCC_IDX_X];
        const float Y = solver_result->x_opt[k][MPCC_IDX_Y];
        if (!isfinite((double)s) ||
            !isfinite((double)X) ||
            !isfinite((double)Y))
        {
            max_violation = FLT_MAX;
            max_violation_stage = k;
            min_slack = -FLT_MAX;
            min_slack_stage = k;
            break;
        }

        MPCCPathPoint_t path_pt;
        mpcc_path_interpolate(&ref_path, s, &path_pt);

        float left_b = prob->track_left[k];
        float right_b = prob->track_right[k];
        if (ref_path.num_points >= 2 &&
            isfinite((double)path_pt.left_bound) &&
            isfinite((double)path_pt.right_bound))
        {
            mpcc_tightened_track_bounds(&path_pt, &left_b, &right_b);
        }

        const float phi = path_pt.phi_ref;
        const float x_ref = path_pt.x_ref;
        const float y_ref = path_pt.y_ref;
        const float e_c = sinf(phi) * (X - x_ref) - cosf(phi) * (Y - y_ref);
        const float left_slack = left_b + e_c;
        const float right_slack = right_b - e_c;
        const float slack = (left_slack < right_slack) ? left_slack : right_slack;

        float violation = 0.0f;
        if (e_c > right_b)
            violation = e_c - right_b;
        else if (e_c < -left_b)
            violation = (-left_b) - e_c;

        if (violation > max_violation) {
            max_violation = violation;
            max_violation_stage = k;
        }
        if (slack < min_slack) {
            min_slack = slack;
            min_slack_stage = k;
        }
    }

    if (max_violation_out) *max_violation_out = max_violation;
    if (max_violation_stage_out) *max_violation_stage_out = max_violation_stage;
    if (min_slack_out) *min_slack_out = min_slack;
    if (min_slack_stage_out) *min_slack_stage_out = min_slack_stage;
}

static float mpcc_warm_start_max_geometry_s_error(uint16_t N)
{
    if (ref_path.num_points < 2)
        return 0.0f;
    if (N > MPCC_MAX_HORIZON)
        N = MPCC_MAX_HORIZON;

    float max_error = 0.0f;
    for (uint16_t k = 0; k <= N; k++)
    {
        const MPCCState_t *st = &prev_predicted_states[k];
        if (!isfinite((double)st->s) ||
            !isfinite((double)st->X) ||
            !isfinite((double)st->Y))
            return FLT_MAX;

        float s_geom = mpcc_find_closest_s_with_hint(&ref_path, st->X, st->Y, st->s);
        float error = mpcc_normalize_s_delta(&ref_path, st->s - s_geom);
        error = fabsf(error);
        if (!isfinite((double)error))
            return FLT_MAX;
        if (error > max_error)
            max_error = error;
    }

    return max_error;
}

static void mpcc_clamp_fallback_vtheta(
    MPCCControl_t *fallback_control,
    const MPCCState_t *current_state)
{
    if (!fallback_control || !current_state)
        return;

    float conservative_max = current_state->vx;

    if (ref_path.num_points >= 2) {
        MPCCPathPoint_t path_pt;
        mpcc_path_interpolate(&ref_path, current_state->s, &path_pt);

        {
            float alpha = remainderf(current_state->psi - path_pt.phi_ref,
                                     2.0f * (float)M_PI);
            float v_path =
                current_state->vx * cosf(alpha) - current_state->vy * sinf(alpha);
            if (isfinite((double)v_path) && v_path < conservative_max)
                conservative_max = v_path;
        }
    }

    conservative_max += 0.10f;
    if (conservative_max < 0.10f)
        conservative_max = 0.10f;
    if (conservative_max > config.v_theta_max)
        conservative_max = config.v_theta_max;
    if (conservative_max < config.v_theta_min)
        conservative_max = config.v_theta_min;

    if (!isfinite((double)fallback_control->v_theta))
        fallback_control->v_theta = config.v_theta_min;
    if (fallback_control->v_theta > conservative_max)
        fallback_control->v_theta = conservative_max;
    if (fallback_control->v_theta < config.v_theta_min)
        fallback_control->v_theta = config.v_theta_min;
}

static void mpcc_debug_print_rejected_horizon(
    const MPCCQPProblem_t *prob,
    const ADMMResult_t *solver_result,
    const MPCCState_t *current_state,
    const MPCCResult_t *result)
{
    if (!prob || !solver_result || !current_state || !result)
        return;

    uint16_t N = prob->N;
    if (N > MPCC_MAX_HORIZON)
        N = MPCC_MAX_HORIZON;

    {
        uint16_t first_k = (N > 5u) ? (uint16_t)(N - 5u) : 0u;

        fprintf(stderr,
                "[MPCC-REJECT] status=%d rej=0x%X fb=%u s=%.2f vx=%.2f "
                "vtheta_cmd=%.2f slack=%.3f@k%u trkv=%.3f@k%u\n",
                (int)result->status,
                (unsigned)result->reject_reason_flags,
                (unsigned)result->used_fallback_control,
                current_state->s,
                current_state->vx,
                result->optimal_control.v_theta,
                result->min_track_slack,
                (unsigned)result->min_track_slack_stage,
                result->max_track_violation,
                (unsigned)result->max_track_violation_stage);

        for (uint16_t k = first_k; k <= N; k++)
        {
            const float s = solver_result->x_opt[k][MPCC_IDX_S];
            const float vx = solver_result->x_opt[k][MPCC_IDX_VX];
            const float X = solver_result->x_opt[k][MPCC_IDX_X];
            const float Y = solver_result->x_opt[k][MPCC_IDX_Y];
            const float v_theta = (k < N)
                ? solver_result->u_opt[k][MPCC_IDX_VTHETA]
                : solver_result->u_opt[(N > 0u) ? (N - 1u) : 0u][MPCC_IDX_VTHETA];
            float left_b = prob->track_left[k];
            float right_b = prob->track_right[k];
            float e_c = 0.0f;
            float slack = -FLT_MAX;
            float violation = FLT_MAX;

            if (isfinite((double)s) && isfinite((double)X) && isfinite((double)Y))
            {
                MPCCPathPoint_t path_pt;
                mpcc_path_interpolate(&ref_path, s, &path_pt);
                if (ref_path.num_points >= 2 &&
                    isfinite((double)path_pt.left_bound) &&
                    isfinite((double)path_pt.right_bound))
                {
                    mpcc_tightened_track_bounds(&path_pt, &left_b, &right_b);
                }

                e_c = sinf(path_pt.phi_ref) * (X - path_pt.x_ref) -
                      cosf(path_pt.phi_ref) * (Y - path_pt.y_ref);
                {
                    float left_slack = left_b + e_c;
                    float right_slack = right_b - e_c;
                    slack = (left_slack < right_slack) ? left_slack : right_slack;
                    violation = 0.0f;
                    if (e_c > right_b)
                        violation = e_c - right_b;
                    else if (e_c < -left_b)
                        violation = (-left_b) - e_c;
                }
            }

            fprintf(stderr,
                "[MPCC-REJECT] k=%u s=%.2f ds=%.2f vx=%.2f vt=%.2f "
                    "ec=%.3f slack=%.3f viol=%.3f left=%.3f right=%.3f\n",
                    (unsigned)k,
                    s,
                    s - current_state->s,
                    vx,
                    v_theta,
                    e_c,
                    slack,
                    violation,
                    left_b,
                    right_b);
        }
        fflush(stderr);
    }
}

#ifdef MPCC_DEBUG_PRINT
static void mpcc_debug_print_consistency(
    const MPCCQPProblem_t *prob,
    const ADMMResult_t *solver_result)
{
    if (!prob || !solver_result)
        return;

    uint16_t N = prob->N;
    if (N > MPCC_MAX_HORIZON)
        N = MPCC_MAX_HORIZON;

    uint16_t stages[3] = {0, (uint16_t)(N / 2u), N};
    for (int idx = 0; idx < 3; idx++)
    {
        uint16_t k = stages[idx];
        if (idx > 0 && k == stages[idx - 1])
            continue;

        float s_bar = prob->s_ref_stage[k];
        float s_opt = solver_result->x_opt[k][MPCC_IDX_S];
        float X = solver_result->x_opt[k][MPCC_IDX_X];
        float Y = solver_result->x_opt[k][MPCC_IDX_Y];
        float vx = solver_result->x_opt[k][MPCC_IDX_VX];
        float vy = solver_result->x_opt[k][MPCC_IDX_VY];
        float psi = solver_result->x_opt[k][MPCC_IDX_PSI];

        if (!isfinite((double)s_opt) ||
            !isfinite((double)X) ||
            !isfinite((double)Y) ||
            !isfinite((double)vx) ||
            !isfinite((double)vy) ||
            !isfinite((double)psi))
        {
            printf("  [CONS] k=%u non-finite solver iterate, skipping consistency details\n",
                   k);
            continue;
        }

        MPCCPathPoint_t actual_pt;
        mpcc_path_interpolate(&ref_path, s_opt, &actual_pt);

        float phi_lin = prob->path_phi_ref[k];
        float ec_linearized =
            sinf(phi_lin) * (X - prob->path_x_ref[k]) -
            cosf(phi_lin) * (Y - prob->path_y_ref[k]);
        float ec_actual =
            sinf(actual_pt.phi_ref) * (X - actual_pt.x_ref) -
            cosf(actual_pt.phi_ref) * (Y - actual_pt.y_ref);

        float left_b = prob->track_left[k];
        float right_b = prob->track_right[k];
        mpcc_tightened_track_bounds(&actual_pt, &left_b, &right_b);

        float violation = 0.0f;
        if (ec_actual > right_b)
            violation = ec_actual - right_b;
        else if (ec_actual < -left_b)
            violation = (-left_b) - ec_actual;

        float v_theta = 0.0f;
        if (N > 0) {
            uint16_t u_idx = (k < N) ? k : (uint16_t)(N - 1u);
            v_theta = solver_result->u_opt[u_idx][MPCC_IDX_VTHETA];
        }
        float alpha = psi - actual_pt.phi_ref;
        float v_path = vx * cosf(alpha) - vy * sinf(alpha);

        printf("  [CONS] k=%u s_bar=%.3f s_opt=%.3f s_err=%.3f "
               "ec_lin=%.4f ec_actual=%.4f track_violation=%.4f "
               "v_theta=%.3f v_path=%.3f\n",
               k,
               (double)s_bar,
               (double)s_opt,
               (double)(s_opt - s_bar),
               (double)ec_linearized,
               (double)ec_actual,
               (double)violation,
               (double)v_theta,
               (double)v_path);
    }
}
#endif

/*===========================================================================
 * Main Control Computation
 *===========================================================================*/

MPCCStatus_t mpcc_compute_control(
    const MPCCState_t *current_state,
    MPCCResult_t *result)
{
    if (!result)
        return MPCC_STATUS_ERROR;

    memset(result, 0, sizeof(*result));

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

    /* Shift the warm-start trajectory forward BEFORE validity checks.
     * Order matters: shift_warm_start() advances prev_predicted_states[0]
     * to the stage-1 prediction, which is the correct anchor for comparing
     * against current_state->s below.  Checking before shifting would compare
     * against the stage-0 anchor from the PREVIOUS cycle (one step stale). */
    MPCCControl_t fallback_control = prev_control_available
        ? prev_control
        : (MPCCControl_t){0, 0, 1.0f};
    if (!isfinite((double)fallback_control.delta)) fallback_control.delta = 0.0f;
    if (!isfinite((double)fallback_control.a_x)) fallback_control.a_x = 0.0f;
    if (!isfinite((double)fallback_control.v_theta)) fallback_control.v_theta = 1.0f;
    if (warm_start_available)
    {
        uint16_t stage_advance = mpcc_warm_start_stage_advance();
        for (uint16_t shift = 0; shift < stage_advance; shift++)
            shift_warm_start();

        uint8_t keep_warm_start = 1;

        /* Detect s-wrap (lap boundary crossing): if s jumped backward
         * by more than half the track length, the warm-started workspace
         * has s values from the old lap. The wrapped state representation
         * is discontinuous at the seam, so reusing the old trajectory can
         * inject a large fictitious jump into the next QP. */
        float s_jump = current_state->s - prev_predicted_states[0].s;
        if (ref_path.is_closed && ref_path.total_length > 0.0f
            && s_jump < -(ref_path.total_length * 0.5f)) {
            keep_warm_start = 0;
            warm_start_available = 0;
#ifdef MPCC_DEBUG_PRINT
            printf("[MPCC] s-wrap detected (%.2f → %.2f), dropping warm-start for this cycle\n",
                   (double)(prev_predicted_states[0].s),
                   (double)current_state->s);
#endif
        }

        if (keep_warm_start) {
            uint16_t N = config.horizon_steps;
            if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;
            float max_s_error = mpcc_warm_start_max_geometry_s_error(N);
            if (max_s_error > config.warm_start_max_s_error) {
                keep_warm_start = 0;
                warm_start_available = 0;
#ifdef MPCC_DEBUG_PRINT
                printf("[MPCC] warm-start geometry mismatch %.2f m > %.2f m, dropping warm-start\n",
                       (double)max_s_error,
                       (double)config.warm_start_max_s_error);
#endif
            }
        }

        if (keep_warm_start) {
            admm_config.warm_start = 1;

            /* Save the shifted warm-start's first control as fallback
             * in case the solver doesn't converge this cycle. */
            fallback_control = prev_predicted_controls[0];
            if (!isfinite((double)fallback_control.delta)) fallback_control.delta = 0.0f;
            if (!isfinite((double)fallback_control.a_x)) fallback_control.a_x = 0.0f;
            if (!isfinite((double)fallback_control.v_theta)) fallback_control.v_theta = 0.0f;
        } else {
            admm_config.warm_start = 0;
        }
    }
    else
    {
        admm_config.warm_start = 0;
    }

    /* Build the multistage QP */
    build_qp_problem(current_state, &qp_problem);
    mpcc_update_qp_diagnostics(&qp_problem, result);

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

    /* Solve QP */
    ADMMResult_t admm_result;
    MPCCStatus_t status = MPCC_STATUS_ERROR;
#if MPCC_SQP_REFINEMENT_STEPS > 1
    MPCCState_t saved_prev_states[MPCC_MAX_HORIZON + 1];
    MPCCControl_t saved_prev_controls[MPCC_MAX_HORIZON];
    uint8_t saved_warm_start_available = warm_start_available;
    uint8_t saved_admm_warm_start = admm_config.warm_start;

    memcpy(saved_prev_states, prev_predicted_states, sizeof(saved_prev_states));
    memcpy(saved_prev_controls, prev_predicted_controls, sizeof(saved_prev_controls));

    for (uint8_t sqp_step = 0; sqp_step < (uint8_t)MPCC_SQP_REFINEMENT_STEPS; sqp_step++)
    {
        if (sqp_step > 0)
            build_qp_problem(current_state, &qp_problem);

        memset(&admm_result, 0, sizeof(admm_result));
#ifdef USE_OSQP
        status = osqp_solver_solve(&qp_problem, &admm_result);
#else
        status = admm_solver_solve(
            &qp_problem, &admm_config, &admm_workspace, &admm_result);
#endif

        if ((sqp_step + 1u) >= (uint8_t)MPCC_SQP_REFINEMENT_STEPS ||
            status == MPCC_STATUS_ERROR ||
            status == MPCC_STATUS_INFEASIBLE)
            break;

        {
            uint16_t N = config.horizon_steps;
            if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;
            for (uint16_t k = 0; k <= N; k++)
                array_to_state(admm_result.x_opt[k], &prev_predicted_states[k]);
            for (uint16_t k = 0; k < N; k++) {
                prev_predicted_controls[k].delta = admm_result.u_opt[k][MPCC_IDX_DELTA];
                prev_predicted_controls[k].a_x = admm_result.u_opt[k][MPCC_IDX_AX];
                prev_predicted_controls[k].v_theta = admm_result.u_opt[k][MPCC_IDX_VTHETA];
            }
            warm_start_available = 1;
            admm_config.warm_start = 1;
        }
    }

    memcpy(prev_predicted_states, saved_prev_states, sizeof(saved_prev_states));
    memcpy(prev_predicted_controls, saved_prev_controls, sizeof(saved_prev_controls));
    warm_start_available = saved_warm_start_available;
    admm_config.warm_start = saved_admm_warm_start;
#else
    memset(&admm_result, 0, sizeof(admm_result));
#ifdef USE_OSQP
    status = osqp_solver_solve(&qp_problem, &admm_result);
#else
    status = admm_solver_solve(
        &qp_problem, &admm_config, &admm_workspace, &admm_result);
#endif
#endif

    /* Extract first control input */
    MPCCStatus_t return_status = status;
    result->status = status;
    result->used_fallback_control = 0u;
    result->reject_reason_flags = 0u;
    result->max_track_violation = 0.0f;
    result->min_track_slack = FLT_MAX;
    result->max_track_violation_stage = 0u;
    result->min_track_slack_stage = 0u;
    result->admm_iterations = admm_result.iterations;
    result->primal_residual = admm_result.primal_residual;
    result->dual_residual = admm_result.dual_residual;
    result->rho_final = admm_result.rho_final;
    result->rho_u_final = admm_result.rho_u_final;
    result->adaptive_rho_updates = admm_result.adaptive_rho_updates;
    result->adaptive_rho_state_updates = admm_result.adaptive_rho_state_updates;
    result->adaptive_rho_control_updates = admm_result.adaptive_rho_control_updates;
    result->numeric_clip_count = admm_result.numeric_clip_count;

#ifdef MPCC_DEBUG_PRINT
    if (status == MPCC_STATUS_SUCCESS || status == MPCC_STATUS_MAX_ITERATIONS)
        mpcc_debug_print_consistency(&qp_problem, &admm_result);
#endif

    {
        const int finite_control = isfinite(admm_result.u_opt[0][MPCC_IDX_DELTA]) &&
                       isfinite(admm_result.u_opt[0][MPCC_IDX_AX]) &&
                       isfinite(admm_result.u_opt[0][MPCC_IDX_VTHETA]);
        const int finite_residuals = isfinite((double)(admm_result.primal_residual)) &&
                       isfinite((double)(admm_result.dual_residual));
        const float warm_start_track_tolerance = 0.01f;
        float max_track_violation = 0.0f;
        float min_track_slack = FLT_MAX;
        uint16_t max_track_violation_stage = 0u;
        uint16_t min_track_slack_stage = 0u;
        uint8_t keep_warm_start = 0u;
        if (finite_residuals)
        {
            mpcc_solution_track_metrics(&qp_problem, &admm_result,
                                        &max_track_violation, &max_track_violation_stage,
                                        &min_track_slack, &min_track_slack_stage);
        }

        result->max_track_violation = max_track_violation;
        result->min_track_slack = min_track_slack;
        result->max_track_violation_stage = max_track_violation_stage;
        result->min_track_slack_stage = min_track_slack_stage;

        const int max_iter_acceptable =
            (status == MPCC_STATUS_MAX_ITERATIONS) &&
            config.accept_max_iterations &&
            finite_residuals &&
            (admm_result.primal_residual <= config.max_iter_primal_tolerance) &&
            (admm_result.dual_residual <= config.max_iter_dual_tolerance) &&
            (max_track_violation <= config.max_iter_track_violation_tolerance);

        const int usable_status = (status == MPCC_STATUS_SUCCESS) ||
                       max_iter_acceptable;
        const int rejected_max_iter =
            (status == MPCC_STATUS_MAX_ITERATIONS) && !max_iter_acceptable;
        const int hard_failure = (status == MPCC_STATUS_INFEASIBLE) ||
                       (status == MPCC_STATUS_ERROR) ||
                       rejected_max_iter ||
                       !finite_residuals;

        if (!finite_residuals)
            result->reject_reason_flags |= MPCC_REJECT_NONFINITE_RESIDUAL;
        if (!finite_control)
            result->reject_reason_flags |= MPCC_REJECT_NONFINITE_CONTROL;
        if (status == MPCC_STATUS_INFEASIBLE || status == MPCC_STATUS_ERROR)
            result->reject_reason_flags |= MPCC_REJECT_SOLVER_STATUS;
        if (rejected_max_iter)
        {
            if (admm_result.primal_residual > config.max_iter_primal_tolerance)
                result->reject_reason_flags |= MPCC_REJECT_PRIMAL_TOL;
            if (admm_result.dual_residual > config.max_iter_dual_tolerance)
                result->reject_reason_flags |= MPCC_REJECT_DUAL_TOL;
            if (max_track_violation > config.max_iter_track_violation_tolerance)
                result->reject_reason_flags |= MPCC_REJECT_TRACK_TOL;
        }

        if (rejected_max_iter)
        {
            return_status = MPCC_STATUS_ERROR;
            result->status = return_status;
        }

        if (usable_status && finite_control && finite_residuals) {
        /* Use the solver iterate for control.
         * MAX_ITERATIONS is accepted only when explicitly enabled and residuals
         * plus predicted hard-corridor violation are small. */
        result->optimal_control.delta = admm_result.u_opt[0][MPCC_IDX_DELTA];
        result->optimal_control.a_x = admm_result.u_opt[0][MPCC_IDX_AX];
        result->optimal_control.v_theta = admm_result.u_opt[0][MPCC_IDX_VTHETA];

        uint16_t N = config.horizon_steps;
        if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;
        for (uint16_t k = 0; k <= N; k++)
            array_to_state(admm_result.x_opt[k], &result->predicted_states[k]);
        for (uint16_t k = 0; k < N; k++)
        {
            result->predicted_controls[k].delta = admm_result.u_opt[k][MPCC_IDX_DELTA];
            result->predicted_controls[k].a_x = admm_result.u_opt[k][MPCC_IDX_AX];
            result->predicted_controls[k].v_theta = admm_result.u_opt[k][MPCC_IDX_VTHETA];
        }

        result->predicted_states[0] = *current_state;
        result->predicted_controls[0] = result->optimal_control;

        if (max_track_violation <= warm_start_track_tolerance) {
            for (uint16_t k = 0; k <= N; k++)
                prev_predicted_states[k] = result->predicted_states[k];
            for (uint16_t k = 0; k < N; k++)
                prev_predicted_controls[k] = result->predicted_controls[k];
            keep_warm_start = 1u;
        }

        prev_control = result->optimal_control;
        prev_control_available = 1;
        } else {
        /* Unconverged / error: fall back to previous good control. */
        mpcc_clamp_fallback_vtheta(&fallback_control, current_state);
        result->optimal_control = fallback_control;
        result->used_fallback_control = 1u;

        /* Safe-braking fallback (solver-robustness).
         *
         * A rejected solve at speed almost always means the predicted
         * trajectory leaves the corridor — the car is over-committed for the
         * geometry ahead and the QP has become effectively infeasible (the
         * dynamics-feasible path must go wide, the projection forces it back,
         * ADMM diverges).  The previous control was tuned for a less-critical
         * state; repeating it drives the car into the wall.  Instead command
         * braking proportional to the predicted hard-corridor violation, so an
         * over-commit sheds speed until the corner becomes feasible again and
         * the solver recovers.  Benign rejects (tiny violation) are left
         * essentially untouched, preserving normal driving; only genuine
         * over-commits brake.  We only ever brake HARDER, never reduce it. */
        {
            const float TRACK_VIOL_FULL_BRAKE = 0.40f;  /* m -> full braking */
            float brake_frac = max_track_violation / TRACK_VIOL_FULL_BRAKE;
            if (hard_failure && brake_frac < 0.5f)
                brake_frac = 0.5f;  /* solver gave up entirely: brake firmly */
            if (brake_frac < 0.0f) brake_frac = 0.0f;
            if (brake_frac > 1.0f) brake_frac = 1.0f;
            float a_brake = config.ax_min * brake_frac;
            if (result->optimal_control.a_x > a_brake)
                result->optimal_control.a_x = a_brake;
        }

        /* Copy solver predicted trajectory for diagnostics */
        uint16_t N = config.horizon_steps;
        if (N > MPCC_MAX_HORIZON) N = MPCC_MAX_HORIZON;
        for (uint16_t k = 0; k <= N; k++)
            array_to_state(admm_result.x_opt[k], &result->predicted_states[k]);
        for (uint16_t k = 0; k < N; k++)
        {
            result->predicted_controls[k].delta = admm_result.u_opt[k][MPCC_IDX_DELTA];
            result->predicted_controls[k].a_x = admm_result.u_opt[k][MPCC_IDX_AX];
            result->predicted_controls[k].v_theta = admm_result.u_opt[k][MPCC_IDX_VTHETA];
        }

        /* Only update warm-start if residual is moderate (partially converged).
         * For badly diverged solves, keep the previous warm-start to avoid
         * contaminating the next solve with garbage. */
        if (finite_residuals &&
            max_track_violation <= warm_start_track_tolerance &&
            admm_result.primal_residual < 50.0f * config.admm_tolerance) {
            for (uint16_t k = 0; k <= N; k++)
                prev_predicted_states[k] = result->predicted_states[k];
            for (uint16_t k = 0; k < N; k++)
                prev_predicted_controls[k] = result->predicted_controls[k];
            keep_warm_start = 1u;
        }
        /* else: keep prev_predicted_states/controls from last good solve */

        }

        /* Numerical breakdown contaminates ADMM workspace (z/w/lambda) and
         * can poison subsequent warm-starts. Reset workspace only on hard failure. */
        if (hard_failure)
        {
#ifndef USE_OSQP
            admm_solver_initialize(&admm_workspace);
#endif
            warm_start_available = 0;
        }
        else
        {
            warm_start_available = keep_warm_start;
        }

        if (result->used_fallback_control || result->reject_reason_flags != 0u)
            mpcc_debug_print_rejected_horizon(&qp_problem, &admm_result,
                                              current_state, result);
    }

#ifdef MPCC_DEBUG_PRINT
        printf("[MPCC] status=%d  iter=%u  prim=%.4f  dual=%.4f  "
            "rho=%.3f  rho_u=%.3f  rho_upd=%u  rho_x_upd=%u  rho_u_upd=%u  clip=%u  "
            "diag=0x%X hmin=%.2e tw=%.3f dw=%.3f axlim=%.3f vxm=%.3f  "
            "delta=%.3f  a_x=%.3f  v_theta=%.3f  s=%.2f  vx=%.3f\n",
           return_status, admm_result.iterations,
           (double)(admm_result.primal_residual),
           (double)(admm_result.dual_residual),
            (double)(admm_result.rho_final),
            (double)(admm_result.rho_u_final),
            admm_result.adaptive_rho_updates,
            admm_result.adaptive_rho_state_updates,
            admm_result.adaptive_rho_control_updates,
            admm_result.numeric_clip_count,
            (unsigned)result->qp_diagnostic_flags,
            (double)result->qp_min_hessian_eigenvalue,
            (double)result->qp_min_track_width,
            (double)result->qp_min_delta_width,
            (double)result->qp_min_ax_limit,
            (double)result->qp_min_vx_margin,
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

    return return_status;
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
    admm_config.rho = config.admm_rho;
    admm_config.max_iterations = config.admm_max_iterations;
    admm_config.eps_primal = config.admm_tolerance;
    admm_config.eps_dual = config.admm_tolerance;
    admm_config.rho_u = config.admm_rho_u;
    admm_config.adaptive_rho = config.admm_adaptive_rho;
    admm_config.alpha_relax = config.admm_alpha_relax;

    /* Recompute cross-call scaling from runtime control period unless an
     * explicit MPCC_CROSS_CALL_SCALE is supplied. */
    {
        const char *env_scale = getenv("MPCC_CROSS_CALL_SCALE");
        float scale = 0.0f;

        if (env_scale)
            scale = (float)atof(env_scale);

        if (scale > 0.0f && isfinite((double)scale))
        {
            config.cross_call_rate_scale = scale;
        }
        else
        {
            const char *env_ms = getenv("MPCC_CONTROL_PERIOD_MS");
            float control_period_sec = 0.0f;
            if (env_ms) {
                float ms = (float)atof(env_ms);
                if (ms > 0.0f) control_period_sec = ms / 1000.0f;
            }
            if (control_period_sec <= 0.0f) {
                control_period_sec = 1.0f / MPCC_CONTROL_RATE_HZ;
            }
            config.cross_call_rate_scale = control_period_sec / config.dt;
        }
    }

    if (config.cross_call_rate_scale <= 0.0f ||
        !isfinite((double)config.cross_call_rate_scale))
        config.cross_call_rate_scale = MPCC_DEFAULT_CROSS_CALL_SCALE;
}

void mpcc_reset(void)
{
    warm_start_available = 0;
    last_closest_idx = 0; /* Reset path tracking */
    unwrapped_heading_available = 0;
    memset(&prev_control, 0, sizeof(prev_control));
    prev_control_available = 0;
#ifndef USE_OSQP
    admm_solver_initialize(&admm_workspace);
#else
    memset(&admm_workspace, 0, sizeof(admm_workspace));
#endif
}
