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
#define SCALE_Q16 65536.0f

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
    return ((float)v) / SCALE_Q16;
}

static int32_t float_to_fp(float v) {
    return (int32_t)(v >= 0.0f ? (v * SCALE_Q16 + 0.5f) : (v * SCALE_Q16 - 0.5f));
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

int main(int argc, char **argv) {
    uint64_t trace_idx = 0;
    const char *trace_out_path = NULL;

    if (argc < 3) {
        fprintf(stderr, "Usage: %s <mpc_state_csv> <out_csv> [--trace-idx N --trace-out path]\n", argv[0]);
        return 2;
    }
    for (int ai = 3; ai < argc; ai++) {
        if (strcmp(argv[ai], "--trace-idx") == 0 && (ai + 1) < argc) {
            trace_idx = (uint64_t)strtoull(argv[++ai], NULL, 10);
        } else if (strcmp(argv[ai], "--trace-out") == 0 && (ai + 1) < argc) {
            trace_out_path = argv[++ai];
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[ai]);
            return 2;
        }
    }

    FILE *in = fopen(argv[1], "r");
    if (!in) {
        perror("open input");
        return 3;
    }
    FILE *out = fopen(argv[2], "w");
    if (!out) {
        perror("open output");
        fclose(in);
        return 4;
    }

    char line[65536];
    if (!fgets(line, sizeof(line), in)) {
        fprintf(stderr, "Input CSV empty\n");
        fclose(in);
        fclose(out);
        return 5;
    }

    fprintf(out, "idx,stamp_ns,status,iters,out_steer_fp,out_accel_fp,ey_fp,epsi_fp,vx_fp,vy_fp,omega_fp,steer_meas_fp,prev_accel_in_fp\n");

    mpc_initialize();
    mpc_reset();

    int32_t prev_accel_fp = 0;
    int32_t last_accel_cmd_fp = 0;

    while (fgets(line, sizeof(line), in)) {
        ReplayRow r;
        int32_t ey_fp = 0;
        int32_t epsi_fp = 0;
        TrajectoryReferencePoint_t ref[HORIZON];
        FrenetState_t st;
        MpcSolverResult_t result;
        MpcSolverStatus_t status;
        ControlInput_t actual_prev;
        int32_t out_steer_fp = 0;
        int32_t out_accel_fp = 0;
        int32_t prev_accel_in_fp = prev_accel_fp;
        int32_t applied_accel_fp = 0;

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
        (void)status;

        out_steer_fp = float_to_fp(result.optimal_control.steer_ang);
        out_accel_fp = float_to_fp(result.optimal_control.long_acc);

        if (result.solver_status == MPC_STATUS_ERROR || result.solver_status == MPC_STATUS_INFEASIBLE) {
            applied_accel_fp = last_accel_cmd_fp;
        } else {
            applied_accel_fp = out_accel_fp;
            last_accel_cmd_fp = out_accel_fp;
        }
        prev_accel_fp = applied_accel_fp;

        if (trace_idx > 0 && trace_out_path != NULL && r.idx == trace_idx) {
            FILE *t = fopen(trace_out_path, "w");
            if (t != NULL) {
                fprintf(t,
                        "idx,iter,primal_residual,dual_residual,state_primal_residual,state_dual_residual,ctrl_primal_residual,ctrl_dual_residual,rho,rho_u,u0_steer,u0_accel,z0_steer,z0_accel,y0_steer,y0_accel,scale_rho,scale_rho_u\n");
                const int trace_n = riccati_debug_get_trace_count();
                for (int ti = 0; ti < trace_n; ti++) {
                    RiccatiDebugIterSample_t s;
                    if (riccati_debug_get_trace_sample(ti, &s) == 0) {
                        fprintf(t,
                                "%llu,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d\n",
                                (unsigned long long)r.idx,
                                s.iter,
                                (double)s.primal_residual,
                                (double)s.dual_residual,
                                (double)s.state_primal_residual,
                                (double)s.state_dual_residual,
                                (double)s.ctrl_primal_residual,
                                (double)s.ctrl_dual_residual,
                                (double)s.rho,
                                (double)s.rho_u,
                                (double)s.u0_steer,
                                (double)s.u0_accel,
                                (double)s.z0_steer,
                                (double)s.z0_accel,
                                (double)s.y0_steer,
                                (double)s.y0_accel,
                                s.scale_rho,
                                s.scale_rho_u);
                    }
                }
                fclose(t);
            } else {
                fprintf(stderr, "Failed to open trace output: %s\n", trace_out_path);
            }
        }

        fprintf(out,
                "%llu,%lld,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                (unsigned long long)r.idx,
                (long long)r.stamp_ns,
                (int)result.solver_status,
                (unsigned int)result.iterations_used,
                out_steer_fp,
                out_accel_fp,
                ey_fp,
                epsi_fp,
                r.velocity_fp,
                r.vy_fp,
                r.omega_fp,
                r.steering_angle_fp,
                prev_accel_in_fp);
    }

    fclose(in);
    fclose(out);
    return 0;
}
