#define _POSIX_C_SOURCE 200809L

/* replay_cpu_mpc_cost.c
 *
 * Cost-breakdown variant of replay_cpu_mpc. Drives the CPU MPC on a bag-derived
 * state CSV (identical input format to replay_cpu_mpc.c) and, for each
 * snapshot, dumps:
 *   - vehicle position + commanded controls
 *   - planned velocity at horizon checkpoints
 *   - per-term cost contribution summed over the horizon (lateral, heading,
 *     velocity, lateral_velocity, yaw_rate, effective_steering, drate_prev,
 *     accel_prev, steering input, acceleration input)
 *
 * Used by Test 2 (low-velocity-weight "stop around corner") to produce the
 * spatial cost-dominance map and the cost-landscape data.
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

static float fp_to_float(int32_t v) {
    return ((float)v) / SCALE_QP;
}

static int32_t float_to_fp(float v) {
    return (int32_t)(v >= 0.0f ? (v * SCALE_QP + 0.5f) : (v * SCALE_QP - 0.5f));
}

static float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

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

    const float abx = bx - ax;
    const float aby = by - ay;
    const float apx = x - ax;
    const float apy = y - ay;
    const float ab_len2 = abx * abx + aby * aby;
    float t = 0.0f;
    if (ab_len2 > 1e-12f) {
        t = (apx * abx + apy * aby) / ab_len2;
    }
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    const float wx = ax + t * abx;
    const float wy = ay + t * aby;
    const float wpsi = h0 + t * wrap_pi(h1 - h0);

    const float dx = x - wx;
    const float dy = y - wy;
    const float best_ey = -sinf(wpsi) * dx + cosf(wpsi) * dy;
    const float best_epsi = wrap_pi(theta - wpsi);

    *ey_fp = float_to_fp(best_ey);
    *epsi_fp = float_to_fp(best_epsi);
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
    long v = 0;
    long long vll = 0;

    if (!tok) return 0;
    r->idx = (uint64_t)strtoull(tok, NULL, 10);

    if (!next_long(&ctx, &v)) return 0; /* stamp_sec */
    if (!next_long(&ctx, &v)) return 0; /* stamp_nsec */
    if (!next_ll(&ctx, &vll)) return 0;
    r->stamp_ns = (int64_t)vll;
    if (!next_long(&ctx, &v)) return 0;
    r->x_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->y_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->theta_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->velocity_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->vy_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->omega_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->steering_angle_fp = (int32_t)v;
    if (!next_long(&ctx, &v)) return 0;
    r->horizon_length_msg = (uint32_t)v;

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

/* Per-stage Q weights consistent with mpc.c:585-596 (RICCATI_COST_FACTOR
 * is folded in so the values here match what the solver actually used). */
typedef struct {
    float Q_lat;
    float Q_heading;
    float Q_vel;
    float Q_lat_vel;
    float Q_yaw_rate;
    float Q_effective_steering;
    float Q_drate_prev;
    float Q_accel_prev;
    float R_steer;
    float R_accel;
} CostWeights_t;

static CostWeights_t build_cost_weights(const MpcConfiguration_t *cfg) {
    CostWeights_t w;
    w.Q_lat          = RICCATI_COST_FACTOR * cfg->weight_lateral_error;
    w.Q_heading      = RICCATI_COST_FACTOR * cfg->weight_heading_error;
    w.Q_vel          = RICCATI_COST_FACTOR * cfg->weight_velocity;
    w.Q_lat_vel      = RICCATI_COST_FACTOR * cfg->weight_lateral_velocity;
    w.Q_yaw_rate     = RICCATI_COST_FACTOR * cfg->weight_yaw_rate;
    w.Q_effective_steering =
        RICCATI_COST_FACTOR * cfg->weight_effective_steering;
    w.Q_drate_prev   = RICCATI_COST_FACTOR * cfg->weight_steering_rate;
    w.Q_accel_prev   = RICCATI_COST_FACTOR * cfg->weight_acceleration_rate;
    w.R_steer        = RICCATI_COST_FACTOR * (cfg->weight_steering_effort + cfg->weight_steering_rate);
    w.R_accel        = RICCATI_COST_FACTOR * (cfg->weight_acceleration_effort + cfg->weight_acceleration_rate);
    return w;
}

typedef struct {
    float J_lat;
    float J_heading;
    float J_vel;
    float J_lat_vel;
    float J_yaw_rate;
    float J_effective_steering;
    float J_drate_prev;
    float J_accel_prev;
    float J_steer_in;
    float J_accel_in;
    float J_total;
} CostBreakdown_t;

static float sq(float a) { return a * a; }

static CostBreakdown_t compute_cost_breakdown(
    float x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX],
    float u[PREDICTION_HORIZON][RICCATI_MAX_NU],
    const TrajectoryReferencePoint_t *ref,
    const CostWeights_t *w)
{
    CostBreakdown_t c = {0};

    /* Stage costs: 0.5 * Q * (x[k] - x_ref[k])^2 summed over k=0..H. */
    for (int k = 0; k <= PREDICTION_HORIZON; k++) {
        const int kr = (k < PREDICTION_HORIZON) ? k : (PREDICTION_HORIZON - 1);
        /* delta_ff matches mpc.c:656 (wheelbase * curvature, clamped to steering bound). */
        float delta_ff = atanf(VP_WHEELBASE_M * ref[kr].path_curvature);
        if (delta_ff >  VP_MAX_STEERING_RAD) delta_ff =  VP_MAX_STEERING_RAD;
        if (delta_ff < -VP_MAX_STEERING_RAD) delta_ff = -VP_MAX_STEERING_RAD;

        c.J_lat          += 0.5f * w->Q_lat          * sq(x[k][0]                - ref[kr].reference_lateral_error);
        c.J_heading      += 0.5f * w->Q_heading      * sq(x[k][1]                - ref[kr].reference_heading_error);
        c.J_vel          += 0.5f * w->Q_vel          * sq(x[k][2]                - ref[kr].reference_velocity);
        c.J_lat_vel      += 0.5f * w->Q_lat_vel      * sq(x[k][3]                - ref[kr].reference_lateral_velocity);
        c.J_yaw_rate     += 0.5f * w->Q_yaw_rate     * sq(x[k][4]                - ref[kr].reference_yaw_rate);
        c.J_effective_steering +=
            0.5f * w->Q_effective_steering *
            sq(x[k][IDX_DELTA_EFFECTIVE] - delta_ff);
        c.J_drate_prev   += 0.5f * w->Q_drate_prev   * sq(x[k][IDX_DRATE_PREV]);
        c.J_accel_prev   += 0.5f * w->Q_accel_prev   * sq(x[k][IDX_ACCEL_PREV]);
    }

    /* Input costs: 0.5 * R * u^2 summed over k=0..H-1. */
    for (int k = 0; k < PREDICTION_HORIZON; k++) {
        c.J_steer_in += 0.5f * w->R_steer * sq(u[k][0]);
        c.J_accel_in += 0.5f * w->R_accel * sq(u[k][1]);
    }

    c.J_total = c.J_lat + c.J_heading + c.J_vel + c.J_lat_vel + c.J_yaw_rate
              + c.J_effective_steering + c.J_drate_prev + c.J_accel_prev
              + c.J_steer_in + c.J_accel_in;
    return c;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <mpc_state_csv> <out_csv>\n", argv[0]);
        return 2;
    }

    FILE *in = fopen(argv[1], "r");
    if (!in) { perror("open input"); return 3; }
    FILE *out = fopen(argv[2], "w");
    if (!out) { perror("open output"); fclose(in); return 4; }

    char line[65536];
    if (!fgets(line, sizeof(line), in)) {
        fprintf(stderr, "Input CSV empty\n");
        fclose(in); fclose(out);
        return 5;
    }

    fprintf(out,
        "idx,stamp_ns,pos_x,pos_y,theta,v_now,v_h5,v_h10,v_h_end,"
        "v_ref0,out_steer,out_accel,ey_in,epsi_in,status,iters,"
        "J_lat,J_heading,J_vel,J_lat_vel,J_yaw_rate,J_effective_steering,"
        "J_drate_prev,J_accel_prev,J_steer_in,J_accel_in,J_total,"
        "w_lat,w_heading,w_vel\n");

    mpc_initialize();
    mpc_reset();

    const MpcConfiguration_t cfg_snapshot = mpc_get_configuration();
    const CostWeights_t w = build_cost_weights(&cfg_snapshot);

    int32_t prev_accel_fp = 0;
    int32_t last_accel_cmd_fp = 0;

    while (fgets(line, sizeof(line), in)) {
        ReplayRow r;
        int32_t ey_fp = 0, epsi_fp = 0;
        TrajectoryReferencePoint_t ref[HORIZON];
        FrenetState_t st;
        MpcSolverResult_t result;
        ControlInput_t previous_command;
        int32_t out_steer_fp = 0, out_accel_fp = 0;
        int32_t prev_accel_in_fp = prev_accel_fp;
        int32_t applied_accel_fp = 0;
        float plan_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
        float plan_u[PREDICTION_HORIZON][RICCATI_MAX_NU];
        CostBreakdown_t c = {0};

        if (!parse_row(line, &r)) {
            fprintf(stderr, "Skipping malformed row\n");
            continue;
        }

        if (r.has_input_frenet) {
            ey_fp = r.input_e_y_fp;
            epsi_fp = r.input_epsi_fp;
        } else {
            compute_frenet_errors_fp(&r, &ey_fp, &epsi_fp);
        }
        st.flat_error = fp_to_float(ey_fp);
        st.fhead_error = fp_to_float(epsi_fp);
        st.flong_vel = fp_to_float(r.velocity_fp);
        st.flat_vel = fp_to_float(r.vy_fp);
        st.fyaw_rate = fp_to_float(r.omega_fp);

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

        previous_command.steer_ang = fp_to_float(r.steering_angle_fp);
        previous_command.long_acc = fp_to_float(prev_accel_in_fp);
        mpc_set_previous_command(&previous_command);

        (void)mpc_compute_optimal_control(&st, ref, &result);

        out_steer_fp = float_to_fp(result.optimal_control.steer_ang);
        out_accel_fp = float_to_fp(result.optimal_control.long_acc);

        if (result.solver_status == MPC_STATUS_ERROR ||
            result.solver_status == MPC_STATUS_INFEASIBLE) {
            applied_accel_fp = last_accel_cmd_fp;
        } else {
            applied_accel_fp = out_accel_fp;
            last_accel_cmd_fp = out_accel_fp;
        }
        prev_accel_fp = applied_accel_fp;

        float v_now = 0.0f, v_h5 = 0.0f, v_h10 = 0.0f, v_h_end = 0.0f;
        if (mpc_debug_copy_last_plan(plan_x, plan_u)) {
            c = compute_cost_breakdown(plan_x, plan_u, ref, &w);
            v_now   = plan_x[0][2];
            v_h5    = plan_x[5][2];
            v_h10   = plan_x[10][2];
            v_h_end = plan_x[PREDICTION_HORIZON][2];
        }

        fprintf(out,
            "%llu,%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%d,%u,"
            "%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
            "%.6g,%.6g,%.6g,%.6g,%.6g,"
            "%.4f,%.4f,%.4f\n",
            (unsigned long long)r.idx,
            (long long)r.stamp_ns,
            fp_to_float(r.x_fp), fp_to_float(r.y_fp), fp_to_float(r.theta_fp),
            v_now, v_h5, v_h10, v_h_end,
            fp_to_float(r.ref_vx_fp[0]),
            fp_to_float(out_steer_fp), fp_to_float(out_accel_fp),
            fp_to_float(ey_fp), fp_to_float(epsi_fp),
            (int)result.solver_status, (unsigned int)result.iterations_used,
            (double)c.J_lat, (double)c.J_heading, (double)c.J_vel,
            (double)c.J_lat_vel, (double)c.J_yaw_rate,
            (double)c.J_effective_steering,
            (double)c.J_drate_prev, (double)c.J_accel_prev,
            (double)c.J_steer_in, (double)c.J_accel_in, (double)c.J_total,
            (double)cfg_snapshot.weight_lateral_error,
            (double)cfg_snapshot.weight_heading_error,
            (double)cfg_snapshot.weight_velocity);
    }

    fclose(in);
    fclose(out);
    return 0;
}
