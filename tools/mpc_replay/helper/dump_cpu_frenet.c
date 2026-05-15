#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mpc_types.h"
#include "vehicle_model.h"

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
} ReplayRow;

enum AccelSource {
    ACCEL_ZERO = 0,
    ACCEL_CPU_PREV = 1,
};

static float fp_to_float(int32_t v) { return ((float)v) / SCALE_Q16; }

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
    if (ab_len2 > 1e-12f) t = (apx * abx + apy * aby) / ab_len2;
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
    return 1;
}

static int parse_cpu_prev_accel(char *line, uint64_t *idx, int32_t *prev_accel_in_fp) {
    /* Accept both replay_cpu_mpc (13 cols) and replay_fpga_scalar (14 cols)
     * by reading idx from first token and prev_accel from the last token. */
    char *ctx = NULL;
    char *tok = strtok_r(line, ",", &ctx);
    char *last = tok;
    if (!tok) return 0;
    *idx = (uint64_t)strtoull(tok, NULL, 10);
    while (1) {
        char *next = strtok_r(NULL, ",", &ctx);
        if (!next) break;
        last = next;
    }
    if (!last) return 0;
    *prev_accel_in_fp = (int32_t)strtol(last, NULL, 10);
    return 1;
}

int main(int argc, char **argv) {
    enum AccelSource accel_source = ACCEL_ZERO;
    const char *cpu_replay_csv_path = NULL;

    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s <state_csv> <out_csv> [--accel-source zero|cpu-prev --cpu-replay-csv path]\n",
                argv[0]);
        return 2;
    }

    for (int ai = 3; ai < argc; ai++) {
        if (strcmp(argv[ai], "--accel-source") == 0 && (ai + 1) < argc) {
            const char *mode = argv[++ai];
            if (strcmp(mode, "zero") == 0) accel_source = ACCEL_ZERO;
            else if (strcmp(mode, "cpu-prev") == 0) accel_source = ACCEL_CPU_PREV;
            else {
                fprintf(stderr, "Unknown accel source: %s\n", mode);
                return 2;
            }
        } else if (strcmp(argv[ai], "--cpu-replay-csv") == 0 && (ai + 1) < argc) {
            cpu_replay_csv_path = argv[++ai];
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[ai]);
            return 2;
        }
    }

    if (accel_source == ACCEL_CPU_PREV && cpu_replay_csv_path == NULL) {
        fprintf(stderr, "--cpu-replay-csv is required when --accel-source cpu-prev\n");
        return 2;
    }

    FILE *in = fopen(argv[1], "r");
    if (!in) {
        perror("open state_csv");
        return 3;
    }

    FILE *out = fopen(argv[2], "w");
    if (!out) {
        perror("open out_csv");
        fclose(in);
        return 4;
    }

    FILE *cpu_in = NULL;
    char cpu_line[4096];
    if (accel_source == ACCEL_CPU_PREV) {
        cpu_in = fopen(cpu_replay_csv_path, "r");
        if (!cpu_in) {
            perror("open cpu_replay_csv");
            fclose(in);
            fclose(out);
            return 5;
        }
        if (!fgets(cpu_line, sizeof(cpu_line), cpu_in)) {
            fprintf(stderr, "CPU replay CSV is empty\n");
            fclose(cpu_in);
            fclose(in);
            fclose(out);
            return 6;
        }
    }

    char line[65536];
    if (!fgets(line, sizeof(line), in)) {
        fprintf(stderr, "state CSV empty\n");
        if (cpu_in) fclose(cpu_in);
        fclose(in);
        fclose(out);
        return 7;
    }

    fprintf(out,
            "idx,stamp_ns,ey,epsi,vx,vy,omega,delta,a_cmd,kappa,ref_vx");
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            fprintf(out, ",A%d%d", i, j);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 2; j++) {
            fprintf(out, ",B%d%d", i, j);
        }
    }
    fprintf(out, "\n");

    while (fgets(line, sizeof(line), in)) {
        ReplayRow r;
        int32_t ey_fp = 0;
        int32_t epsi_fp = 0;
        float A[5][5];
        float B[5][2];

        if (!parse_row(line, &r)) {
            fprintf(stderr, "Skipping malformed state row\n");
            continue;
        }

        compute_frenet_errors_fp(&r, &ey_fp, &epsi_fp);

        float a_cmd = 0.0f;
        if (accel_source == ACCEL_CPU_PREV) {
            if (!fgets(cpu_line, sizeof(cpu_line), cpu_in)) {
                fprintf(stderr, "CPU replay CSV ended before state CSV\n");
                break;
            }
            uint64_t cpu_idx = 0;
            int32_t prev_accel_fp = 0;
            if (!parse_cpu_prev_accel(cpu_line, &cpu_idx, &prev_accel_fp)) {
                fprintf(stderr, "Malformed CPU replay row\n");
                continue;
            }
            if (cpu_idx != r.idx) {
                fprintf(stderr,
                        "Index mismatch state/cpu-replay: state=%llu cpu=%llu\n",
                        (unsigned long long)r.idx,
                        (unsigned long long)cpu_idx);
                continue;
            }
            a_cmd = fp_to_float(prev_accel_fp);
        }

        FrenetState_t st;
        st.flat_error = fp_to_float(ey_fp);
        st.fhead_error = fp_to_float(epsi_fp);
        st.flong_vel = fp_to_float(r.velocity_fp);
        st.flat_vel = fp_to_float(r.vy_fp);
        st.fyaw_rate = fp_to_float(r.omega_fp);

        ControlInput_t u;
        u.steer_ang = fp_to_float(r.steering_angle_fp);
        u.long_acc = a_cmd;

        const float kappa = fp_to_float(r.ref_kappa_fp[0]);
        const float ref_vx = fp_to_float(r.ref_vx_fp[0]);

        vehicle_model_compute_frenet_linearization(
            &st, &u, PREDICTION_DT_SECONDS, kappa, ref_vx, A, B);

        fprintf(out,
                "%llu,%lld,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                (unsigned long long)r.idx,
                (long long)r.stamp_ns,
                (double)st.flat_error,
                (double)st.fhead_error,
                (double)st.flong_vel,
                (double)st.flat_vel,
                (double)st.fyaw_rate,
                (double)u.steer_ang,
                (double)u.long_acc,
                (double)kappa,
                (double)ref_vx);

        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                fprintf(out, ",%.9g", (double)A[i][j]);
            }
        }
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 2; j++) {
                fprintf(out, ",%.9g", (double)B[i][j]);
            }
        }
        fprintf(out, "\n");
    }

    if (cpu_in) fclose(cpu_in);
    fclose(in);
    fclose(out);
    return 0;
}
