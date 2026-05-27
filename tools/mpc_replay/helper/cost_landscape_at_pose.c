/* cost_landscape_at_pose.c
 *
 * At a chosen snapshot from a state CSV, sweep the velocity-tracking weight
 * (MPC_W_VELOCITY) over a range. For each weight, cold-start the MPC, solve
 * on the snapshot, and record:
 *   - weight value used
 *   - planned velocity at horizon checkpoints
 *   - cost breakdown evaluated under the ORIGINAL (default) weights
 *
 * Used by Test 2 (Map 3) to show that lowering the velocity-tracking weight
 * pulls the optimal plan toward low velocity at a critical pose.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mpc.h"
#include "riccati_solver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HORIZON 20
#include "mpc_fpga_constants.h"
#define SCALE_QP MPC_FPGA_QP_SCALE_F32

typedef struct {
    uint64_t idx;
    int64_t stamp_ns;
    int32_t x_fp;
    int32_t y_fp;
    int32_t theta_fp;
    int32_t velocity_fp;
    int32_t vy_fp;
    int32_t omega_fp;
    int32_t steering_angle_fp;
    uint32_t horizon_length_msg;
    int32_t ref_ey_fp[HORIZON];
    int32_t ref_epsi_fp[HORIZON];
    int32_t ref_x_fp[HORIZON];
    int32_t ref_y_fp[HORIZON];
    int32_t ref_psi_fp[HORIZON];
    int32_t ref_vx_fp[HORIZON];
    int32_t ref_vy_fp[HORIZON];
    int32_t ref_omega_ref_fp[HORIZON];
    int32_t ref_kappa_fp[HORIZON];
    int32_t ref_left_bound_fp[HORIZON];
    int32_t ref_right_bound_fp[HORIZON];
    int32_t input_e_y_fp;
    int32_t input_epsi_fp;
    int has_input_frenet;
} ReplayRow;

static float fp_to_float(int32_t v) { return ((float)v) / SCALE_QP; }
static int32_t float_to_fp(float v) {
    return (int32_t)(v >= 0.0f ? (v * SCALE_QP + 0.5f) : (v * SCALE_QP - 0.5f));
}
static float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}
static float sq(float a) { return a * a; }

static void compute_frenet_errors_fp(const ReplayRow *r, int32_t *ey_fp, int32_t *epsi_fp) {
    const float x = fp_to_float(r->x_fp);
    const float y = fp_to_float(r->y_fp);
    const float theta = fp_to_float(r->theta_fp);
    const float ax = fp_to_float(r->ref_x_fp[0]);
    const float ay = fp_to_float(r->ref_y_fp[0]);
    const float bx = fp_to_float(r->ref_x_fp[1]);
    const float by = fp_to_float(r->ref_y_fp[1]);
    const float h0 = fp_to_float(r->ref_psi_fp[0]);
    const float h1 = fp_to_float(r->ref_psi_fp[1]);
    const float abx = bx - ax, aby = by - ay;
    const float apx = x - ax, apy = y - ay;
    const float ab_len2 = abx * abx + aby * aby;
    float t = 0.0f;
    if (ab_len2 > 1e-12f) t = (apx * abx + apy * aby) / ab_len2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const float wx = ax + t * abx;
    const float wy = ay + t * aby;
    const float wpsi = h0 + t * wrap_pi(h1 - h0);
    const float dx = x - wx, dy = y - wy;
    *ey_fp = float_to_fp(-sinf(wpsi) * dx + cosf(wpsi) * dy);
    *epsi_fp = float_to_fp(wrap_pi(theta - wpsi));
}

static int next_long(char **ctx, long *out) {
    char *tok = strtok_r(NULL, ",", ctx);
    char *end = NULL;
    if (!tok) return 0;
    *out = strtol(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}
static int next_ll(char **ctx, long long *out) {
    char *tok = strtok_r(NULL, ",", ctx);
    char *end = NULL;
    if (!tok) return 0;
    *out = strtoll(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int parse_row(char *line, ReplayRow *r) {
    char *ctx = NULL;
    char *tok = strtok_r(line, ",", &ctx);
    long v = 0; long long vll = 0;
    if (!tok) return 0;
    r->idx = (uint64_t)strtoull(tok, NULL, 10);
    if (!next_long(&ctx, &v)) return 0;
    if (!next_long(&ctx, &v)) return 0;
    if (!next_ll(&ctx, &vll)) return 0; r->stamp_ns = (int64_t)vll;
    if (!next_long(&ctx, &v)) return 0; r->x_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->y_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->theta_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->velocity_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->vy_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->omega_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->steering_angle_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0; r->horizon_length_msg = (uint32_t)v;
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_ey_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_epsi_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_x_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_y_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_psi_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_vx_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_vy_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_omega_ref_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_kappa_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_left_bound_fp[i] = (int32_t)v; }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r->ref_right_bound_fp[i] = (int32_t)v; }
    r->has_input_frenet = 0;
    if (next_long(&ctx, &v)) {
        r->input_e_y_fp = (int32_t)v;
        if (!next_long(&ctx, &v)) return 0;
        r->input_epsi_fp = (int32_t)v;
        r->has_input_frenet = 1;
    }
    return 1;
}

typedef struct {
    float Q_lat, Q_heading, Q_vel, Q_lat_vel, Q_yaw_rate;
    float Q_delta_actual, Q_drate_prev, Q_accel_prev;
    float R_steer, R_accel;
} CostWeights_t;

static CostWeights_t build_cost_weights(const MpcConfiguration_t *c) {
    CostWeights_t w;
    w.Q_lat          = RICCATI_COST_FACTOR * c->weight_lateral_error;
    w.Q_heading      = RICCATI_COST_FACTOR * c->weight_heading_error;
    w.Q_vel          = RICCATI_COST_FACTOR * c->weight_velocity;
    w.Q_lat_vel      = RICCATI_COST_FACTOR * c->weight_lateral_velocity;
    w.Q_yaw_rate     = RICCATI_COST_FACTOR * c->weight_yaw_rate;
    w.Q_delta_actual = RICCATI_COST_FACTOR * c->weight_delta_actual;
    w.Q_drate_prev   = RICCATI_COST_FACTOR * c->weight_steering_rate;
    w.Q_accel_prev   = RICCATI_COST_FACTOR * c->weight_acceleration_rate;
    w.R_steer        = RICCATI_COST_FACTOR * (c->weight_steering_effort + c->weight_steering_rate);
    w.R_accel        = RICCATI_COST_FACTOR * (c->weight_acceleration_effort + c->weight_acceleration_rate);
    return w;
}

typedef struct {
    float J_lat, J_heading, J_vel, J_lat_vel, J_yaw_rate;
    float J_delta_actual, J_drate_prev, J_accel_prev;
    float J_steer_in, J_accel_in, J_total;
} CostBreakdown_t;

static CostBreakdown_t compute_cost_breakdown(
    const float x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX],
    const float u[PREDICTION_HORIZON][RICCATI_MAX_NU],
    const TrajectoryReferencePoint_t *ref,
    const CostWeights_t *w)
{
    CostBreakdown_t c = {0};
    for (int k = 0; k <= PREDICTION_HORIZON; k++) {
        const int kr = (k < PREDICTION_HORIZON) ? k : (PREDICTION_HORIZON - 1);
        float delta_ff = atanf(VP_WHEELBASE_M * ref[kr].path_curvature);
        if (delta_ff >  VP_MAX_STEERING_RAD) delta_ff =  VP_MAX_STEERING_RAD;
        if (delta_ff < -VP_MAX_STEERING_RAD) delta_ff = -VP_MAX_STEERING_RAD;
        c.J_lat          += 0.5f * w->Q_lat          * sq(x[k][0]                - ref[kr].reference_lateral_error);
        c.J_heading      += 0.5f * w->Q_heading      * sq(x[k][1]                - ref[kr].reference_heading_error);
        c.J_vel          += 0.5f * w->Q_vel          * sq(x[k][2]                - ref[kr].reference_velocity);
        c.J_lat_vel      += 0.5f * w->Q_lat_vel      * sq(x[k][3]                - ref[kr].reference_lateral_velocity);
        c.J_yaw_rate     += 0.5f * w->Q_yaw_rate     * sq(x[k][4]                - ref[kr].reference_yaw_rate);
        c.J_delta_actual += 0.5f * w->Q_delta_actual * sq(x[k][IDX_DELTA_ACTUAL] - delta_ff);
        c.J_drate_prev   += 0.5f * w->Q_drate_prev   * sq(x[k][IDX_DRATE_PREV]);
        c.J_accel_prev   += 0.5f * w->Q_accel_prev   * sq(x[k][IDX_ACCEL_PREV]);
    }
    for (int k = 0; k < PREDICTION_HORIZON; k++) {
        c.J_steer_in += 0.5f * w->R_steer * sq(u[k][0]);
        c.J_accel_in += 0.5f * w->R_accel * sq(u[k][1]);
    }
    c.J_total = c.J_lat + c.J_heading + c.J_vel + c.J_lat_vel + c.J_yaw_rate
              + c.J_delta_actual + c.J_drate_prev + c.J_accel_prev
              + c.J_steer_in + c.J_accel_in;
    return c;
}

static int load_snapshot(const char *csv_path, uint64_t target_idx, ReplayRow *out_row) {
    FILE *fp = fopen(csv_path, "r");
    if (!fp) { perror("open state CSV"); return 1; }
    char line[65536];
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return 2; }
    while (fgets(line, sizeof(line), fp)) {
        ReplayRow r;
        char tmp[65536];
        strncpy(tmp, line, sizeof(tmp));
        tmp[sizeof(tmp) - 1] = '\0';
        if (!parse_row(tmp, &r)) continue;
        if (r.idx == target_idx) {
            *out_row = r;
            fclose(fp);
            return 0;
        }
    }
    fclose(fp);
    return 3;
}

int main(int argc, char **argv) {
    if (argc < 5) {
        fprintf(stderr,
            "Usage: %s <state_csv> <snapshot_idx> <out_csv> "
            "<w_min> <w_max> <n_steps>\n", argv[0]);
        fprintf(stderr,
            "  Sweeps MPC_W_VELOCITY from w_min to w_max on log scale "
            "in n_steps points.\n");
        return 2;
    }
    const char *state_csv = argv[1];
    const uint64_t snapshot_idx = (uint64_t)strtoull(argv[2], NULL, 10);
    const char *out_csv = argv[3];
    const float w_min = strtof(argc > 4 ? argv[4] : "0.001", NULL);
    const float w_max = strtof(argc > 5 ? argv[5] : "500.0", NULL);
    const int n_steps = argc > 6 ? atoi(argv[6]) : 25;

    ReplayRow r;
    int rc = load_snapshot(state_csv, snapshot_idx, &r);
    if (rc != 0) {
        fprintf(stderr, "Failed to load snapshot idx=%llu (rc=%d)\n",
                (unsigned long long)snapshot_idx, rc);
        return 3;
    }

    FILE *out = fopen(out_csv, "w");
    if (!out) { perror("open out_csv"); return 4; }
    fprintf(out,
        "idx,w_vel_used,v_now,v_h5,v_h10,v_h_end,out_steer,out_accel,"
        "status,iters,"
        "J_lat,J_heading,J_vel,J_lat_vel,J_yaw_rate,J_delta_actual,"
        "J_drate_prev,J_accel_prev,J_steer_in,J_accel_in,J_total\n");

    /* Initialize once to learn default weights, then snapshot them as "original". */
    mpc_initialize();
    const MpcConfiguration_t default_cfg = mpc_get_configuration();
    const CostWeights_t original_weights = build_cost_weights(&default_cfg);

    /* Pre-compute ey/epsi and reference once. */
    int32_t ey_fp = 0, epsi_fp = 0;
    if (r.has_input_frenet) {
        ey_fp = r.input_e_y_fp;
        epsi_fp = r.input_epsi_fp;
    } else {
        compute_frenet_errors_fp(&r, &ey_fp, &epsi_fp);
    }
    FrenetState_t st;
    st.flat_error = fp_to_float(ey_fp);
    st.fhead_error = fp_to_float(epsi_fp);
    st.flong_vel = fp_to_float(r.velocity_fp);
    st.flat_vel = fp_to_float(r.vy_fp);
    st.fyaw_rate = fp_to_float(r.omega_fp);

    TrajectoryReferencePoint_t ref[HORIZON];
    for (int i = 0; i < HORIZON; i++) {
        ref[i].reference_lateral_error    = fp_to_float(r.ref_ey_fp[i]);
        ref[i].reference_heading_error    = fp_to_float(r.ref_epsi_fp[i]);
        ref[i].reference_velocity         = fp_to_float(r.ref_vx_fp[i]);
        ref[i].reference_lateral_velocity = fp_to_float(r.ref_vy_fp[i]);
        ref[i].reference_yaw_rate         = fp_to_float(r.ref_omega_ref_fp[i]);
        ref[i].path_curvature             = fp_to_float(r.ref_kappa_fp[i]);
        ref[i].left_wall_bound            = fp_to_float(r.ref_left_bound_fp[i]);
        ref[i].right_wall_bound           = fp_to_float(r.ref_right_bound_fp[i]);
    }

    ControlInput_t actual_prev;
    actual_prev.steer_ang = fp_to_float(r.steering_angle_fp);
    actual_prev.long_acc = 0.0f;

    const float log_min = logf(w_min);
    const float log_max = logf(w_max);

    for (int step = 0; step < n_steps; step++) {
        const float t = (n_steps == 1) ? 0.0f : ((float)step / (float)(n_steps - 1));
        const float w_vel = expf(log_min + t * (log_max - log_min));

        MpcConfiguration_t cfg = default_cfg;
        cfg.weight_velocity = w_vel;
        mpc_set_configuration(&cfg);
        mpc_reset();  /* cold start for fairness */
        mpc_set_actual_previous_control(&actual_prev);

        MpcSolverResult_t result;
        (void)mpc_compute_optimal_control(&st, ref, &result);

        float plan_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
        float plan_u[PREDICTION_HORIZON][RICCATI_MAX_NU];
        CostBreakdown_t c = {0};
        float v_now = 0, v_h5 = 0, v_h10 = 0, v_h_end = 0;
        if (mpc_debug_copy_last_plan(plan_x, plan_u)) {
            c = compute_cost_breakdown(plan_x, plan_u, ref, &original_weights);
            v_now   = plan_x[0][2];
            v_h5    = plan_x[5][2];
            v_h10   = plan_x[10][2];
            v_h_end = plan_x[PREDICTION_HORIZON][2];
        }

        fprintf(out,
            "%llu,%.6g,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%u,"
            "%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n",
            (unsigned long long)r.idx,
            (double)w_vel,
            v_now, v_h5, v_h10, v_h_end,
            result.optimal_control.steer_ang, result.optimal_control.long_acc,
            (int)result.solver_status, (unsigned int)result.iterations_used,
            (double)c.J_lat, (double)c.J_heading, (double)c.J_vel,
            (double)c.J_lat_vel, (double)c.J_yaw_rate, (double)c.J_delta_actual,
            (double)c.J_drate_prev, (double)c.J_accel_prev,
            (double)c.J_steer_in, (double)c.J_accel_in, (double)c.J_total);
    }

    fclose(out);
    return 0;
}
