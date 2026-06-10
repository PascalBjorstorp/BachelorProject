#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mpc.h"

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

static float fp_to_float(int32_t v)
{
    return ((float)v) / SCALE_QP;
}

static int32_t float_to_fp(float v)
{
    return (int32_t)(v >= 0.0f ? (v * SCALE_QP + 0.5f) : (v * SCALE_QP - 0.5f));
}

static float wrap_pi(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void compute_frenet_errors_fp(const ReplayRow *r, int32_t *ey_fp, int32_t *epsi_fp)
{
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

    {
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
}

static int next_long(char **ctx, long *out)
{
    char *tok = strtok_r(NULL, ",", ctx);
    char *end = NULL;
    if (!tok) return 0;
    *out = strtol(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int next_ll(char **ctx, long long *out)
{
    char *tok = strtok_r(NULL, ",", ctx);
    char *end = NULL;
    if (!tok) return 0;
    *out = strtoll(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int parse_row(char *line, ReplayRow *r)
{
    char *ctx = NULL;
    char *tok = strtok_r(line, ",", &ctx);
    long v = 0;
    long long vll = 0;

    if (!tok) return 0;
    r->idx = (uint64_t)strtoull(tok, NULL, 10);

    if (!next_long(&ctx, &v)) return 0;
    if (!next_long(&ctx, &v)) return 0;
    if (!next_ll(&ctx, &vll)) return 0;
    r->stamp_ns = (int64_t)vll;
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

static void dump_plan_csv(
    const char *path,
    const ReplayRow *row,
    const MpcSolverResult_t *result,
    const float x_plan[PREDICTION_HORIZON + 1][RICCATI_MAX_NX],
    const float u_plan[PREDICTION_HORIZON][RICCATI_MAX_NU])
{
    FILE *out = fopen(path, "w");
    VehicleState_t initial_state;
    VehicleState_t predicted[HORIZON + 1];
    ControlInput_t controls[HORIZON];
    const MpcConfiguration_t cfg = mpc_get_configuration();

    if (!out) {
        perror("open output");
        exit(5);
    }

    initial_state.pos_x = fp_to_float(row->x_fp);
    initial_state.pos_y = fp_to_float(row->y_fp);
    initial_state.heading = fp_to_float(row->theta_fp);
    initial_state.long_vel = fp_to_float(row->velocity_fp);
    initial_state.lat_vel = fp_to_float(row->vy_fp);
    initial_state.yaw_rate = fp_to_float(row->omega_fp);

    for (int k = 0; k < HORIZON; k++) {
        controls[k].steer_ang = x_plan[k][IDX_DELTA_ACTUAL];
        controls[k].long_acc = u_plan[k][1];
    }
    vehicle_model_predict_trajectory(&initial_state, controls, cfg.time_step, HORIZON, predicted);

    fprintf(out, "selected_idx,%llu\n", (unsigned long long)row->idx);
    fprintf(out, "stamp_ns,%lld\n", (long long)row->stamp_ns);
    fprintf(out, "solver_status,%d\n", (int)result->solver_status);
    fprintf(out, "iterations,%u\n", (unsigned int)result->iterations_used);
    fprintf(out, "prediction_dt_s,%.9g\n", (double)cfg.time_step);
    fprintf(out, "horizon_steps,%d\n", HORIZON);
    fprintf(out, "\n");
    fprintf(out,
            "step,is_terminal,ref_x,ref_y,ref_psi,ref_vx,ref_vy,ref_omega,ref_kappa,left_bound,right_bound,"
            "plan_ey,plan_epsi,plan_vx,plan_vy,plan_omega,plan_delta_actual,plan_prev_drate,plan_prev_accel,"
            "u_drate,u_accel,pred_x,pred_y,pred_psi,pred_vx,pred_vy,pred_omega\n");

    for (int k = 0; k <= HORIZON; k++) {
        const int ref_idx = (k < HORIZON) ? k : (HORIZON - 1);
        const int is_terminal = (k == HORIZON) ? 1 : 0;
        const float u_drate = (k < HORIZON) ? u_plan[k][0] : 0.0f;
        const float u_accel = (k < HORIZON) ? u_plan[k][1] : 0.0f;

        fprintf(out,
                "%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                k,
                is_terminal,
                (double)fp_to_float(row->ref_x_fp[ref_idx]),
                (double)fp_to_float(row->ref_y_fp[ref_idx]),
                (double)fp_to_float(row->ref_psi_fp[ref_idx]),
                (double)fp_to_float(row->ref_vx_fp[ref_idx]),
                (double)fp_to_float(row->ref_vy_fp[ref_idx]),
                (double)fp_to_float(row->ref_omega_ref_fp[ref_idx]),
                (double)fp_to_float(row->ref_kappa_fp[ref_idx]),
                (double)fp_to_float(row->ref_left_bound_fp[ref_idx]),
                (double)fp_to_float(row->ref_right_bound_fp[ref_idx]),
                (double)x_plan[k][IDX_EY],
                (double)x_plan[k][1],
                (double)x_plan[k][2],
                (double)x_plan[k][3],
                (double)x_plan[k][4],
                (double)x_plan[k][IDX_DELTA_ACTUAL],
                (double)x_plan[k][IDX_DRATE_PREV],
                (double)x_plan[k][IDX_ACCEL_PREV],
                (double)u_drate,
                (double)u_accel,
                (double)predicted[k].pos_x,
                (double)predicted[k].pos_y,
                (double)predicted[k].heading,
                (double)predicted[k].long_vel,
                (double)predicted[k].lat_vel,
                (double)predicted[k].yaw_rate);
    }

    fclose(out);
}

int main(int argc, char **argv)
{
    FILE *in = NULL;
    char line[65536];
    uint64_t selected_idx = 0;
    int found = 0;
    int32_t prev_accel_fp = 0;
    int32_t last_accel_cmd_fp = 0;

    if (argc != 4) {
        fprintf(stderr, "Usage: %s <state_replay.csv> <idx> <out.csv>\n", argv[0]);
        return 2;
    }

    selected_idx = (uint64_t)strtoull(argv[2], NULL, 10);
    in = fopen(argv[1], "r");
    if (!in) {
        perror("open input");
        return 3;
    }

    if (!fgets(line, sizeof(line), in)) {
        fprintf(stderr, "Input CSV empty\n");
        fclose(in);
        return 4;
    }

    mpc_initialize();
    mpc_reset();

    while (fgets(line, sizeof(line), in)) {
        ReplayRow r;
        int32_t ey_fp = 0;
        int32_t epsi_fp = 0;
        TrajectoryReferencePoint_t ref[HORIZON];
        FrenetState_t st;
        MpcSolverResult_t result;
        MpcSolverStatus_t status;
        ControlInput_t actual_prev;
        int32_t prev_accel_in_fp = prev_accel_fp;
        int32_t applied_accel_fp = 0;

        if (!parse_row(line, &r)) {
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
            ref[i].reference_lateral_error = fp_to_float(r.ref_ey_fp[i]);
            ref[i].reference_heading_error = fp_to_float(r.ref_epsi_fp[i]);
            ref[i].reference_velocity = fp_to_float(r.ref_vx_fp[i]);
            ref[i].reference_lateral_velocity = fp_to_float(r.ref_vy_fp[i]);
            ref[i].reference_yaw_rate = fp_to_float(r.ref_omega_ref_fp[i]);
            ref[i].path_curvature = fp_to_float(r.ref_kappa_fp[i]);
            ref[i].left_wall_bound = fp_to_float(r.ref_left_bound_fp[i]);
            ref[i].right_wall_bound = fp_to_float(r.ref_right_bound_fp[i]);
        }

        actual_prev.steer_ang = fp_to_float(r.steering_angle_fp);
        actual_prev.long_acc = fp_to_float(prev_accel_in_fp);
        mpc_set_actual_previous_control(&actual_prev);

        status = mpc_compute_optimal_control(&st, ref, &result);

        if (result.solver_status == MPC_STATUS_ERROR ||
            result.solver_status == MPC_STATUS_INFEASIBLE) {
            applied_accel_fp = last_accel_cmd_fp;
        } else {
            applied_accel_fp = float_to_fp(result.optimal_control.long_acc);
            last_accel_cmd_fp = applied_accel_fp;
        }
        prev_accel_fp = applied_accel_fp;

        if (r.idx == selected_idx) {
            float x_plan[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
            float u_plan[PREDICTION_HORIZON][RICCATI_MAX_NU];

            (void)status;
            if (!mpc_debug_copy_last_plan(x_plan, u_plan)) {
                fprintf(stderr, "No MPC plan available at idx=%llu\n",
                        (unsigned long long)selected_idx);
                fclose(in);
                return 6;
            }

            dump_plan_csv(argv[3], &r, &result, x_plan, u_plan);
            found = 1;
            break;
        }
    }

    fclose(in);

    if (!found) {
        fprintf(stderr, "Could not find idx=%llu in %s\n",
                (unsigned long long)selected_idx, argv[1]);
        return 7;
    }

    return 0;
}
