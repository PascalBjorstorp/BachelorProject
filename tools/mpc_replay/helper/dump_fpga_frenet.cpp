#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "fp_math_hls.h"
#include "mpc_fpga_types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static constexpr int HORIZON = 20;
static constexpr float SCALE_Q16 = 65536.0f;

struct ReplayRow {
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
};

enum AccelSource {
    ACCEL_ZERO = 0,
    ACCEL_CPU_PREV = 1,
};

extern void compute_frenet_AB_and_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t reference_velocity,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
    fp_QP_t next_state[MPC_NX_FRENET]);

static inline float fp_to_float(int32_t v) { return static_cast<float>(v) / SCALE_Q16; }

static inline int32_t float_to_fp(float v) {
    return static_cast<int32_t>(v >= 0.0f ? (v * SCALE_Q16 + 0.5f) : (v * SCALE_Q16 - 0.5f));
}

static inline float wrap_pi(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

static void compute_frenet_errors_fp(const ReplayRow &r, int32_t &ey_fp, int32_t &epsi_fp) {
    const float x = fp_to_float(r.x_fp);
    const float y = fp_to_float(r.y_fp);
    const float theta = fp_to_float(r.theta_fp);

    const float ax = fp_to_float(r.ref_x_fp[0]);
    const float ay = fp_to_float(r.ref_y_fp[0]);
    const float bx = fp_to_float(r.ref_x_fp[1]);
    const float by = fp_to_float(r.ref_y_fp[1]);
    const float h0 = fp_to_float(r.ref_psi_fp[0]);
    const float h1 = fp_to_float(r.ref_psi_fp[1]);

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
    const float best_ey = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;
    const float best_epsi = wrap_pi(theta - wpsi);

    ey_fp = float_to_fp(best_ey);
    epsi_fp = float_to_fp(best_epsi);
}

static int next_long(char **ctx, long *out) {
    char *tok = ::strtok_r(nullptr, ",", ctx);
    char *end = nullptr;
    if (!tok) return 0;
    *out = std::strtol(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int next_ll(char **ctx, long long *out) {
    char *tok = ::strtok_r(nullptr, ",", ctx);
    char *end = nullptr;
    if (!tok) return 0;
    *out = std::strtoll(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int parse_row(char *line, ReplayRow &r) {
    char *ctx = nullptr;
    char *tok = ::strtok_r(line, ",", &ctx);
    long v = 0;
    long long vll = 0;

    if (!tok) return 0;
    r.idx = static_cast<uint64_t>(std::strtoull(tok, nullptr, 10));

    if (!next_long(&ctx, &v)) return 0; /* stamp_sec */
    if (!next_long(&ctx, &v)) return 0; /* stamp_nsec */
    if (!next_ll(&ctx, &vll)) return 0;
    r.stamp_ns = static_cast<int64_t>(vll);
    if (!next_long(&ctx, &v)) return 0; r.x_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.y_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.theta_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.velocity_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.vy_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.omega_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.steering_angle_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0; r.horizon_length_msg = static_cast<uint32_t>(v);

    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_ey_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_epsi_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_x_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_y_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_psi_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_vx_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_vy_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_omega_ref_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_kappa_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_left_bound_fp[i] = static_cast<int32_t>(v); }
    for (int i = 0; i < HORIZON; i++) { if (!next_long(&ctx, &v)) return 0; r.ref_right_bound_fp[i] = static_cast<int32_t>(v); }
    return 1;
}

static int parse_cpu_prev_accel(char *line, uint64_t *idx, int32_t *prev_accel_in_fp) {
    /* Accept both replay_cpu_mpc (13 cols) and replay_fpga_scalar (14 cols)
     * by reading idx from first token and prev_accel from the last token. */
    char *ctx = nullptr;
    char *tok = ::strtok_r(line, ",", &ctx);
    char *last = tok;
    if (!tok) return 0;
    *idx = static_cast<uint64_t>(std::strtoull(tok, nullptr, 10));
    while (1) {
        char *next = ::strtok_r(nullptr, ",", &ctx);
        if (!next) break;
        last = next;
    }
    if (!last) return 0;
    *prev_accel_in_fp = static_cast<int32_t>(std::strtol(last, nullptr, 10));
    return 1;
}

int main(int argc, char **argv) {
    enum AccelSource accel_source = ACCEL_ZERO;
    const char *cpu_replay_csv_path = nullptr;

    if (argc < 3) {
        std::fprintf(stderr,
                     "Usage: %s <state_csv> <out_csv> [--accel-source zero|cpu-prev --cpu-replay-csv path]\n",
                     argv[0]);
        return 2;
    }

    for (int ai = 3; ai < argc; ai++) {
        if (std::strcmp(argv[ai], "--accel-source") == 0 && (ai + 1) < argc) {
            const char *mode = argv[++ai];
            if (std::strcmp(mode, "zero") == 0) accel_source = ACCEL_ZERO;
            else if (std::strcmp(mode, "cpu-prev") == 0) accel_source = ACCEL_CPU_PREV;
            else {
                std::fprintf(stderr, "Unknown accel source: %s\n", mode);
                return 2;
            }
        } else if (std::strcmp(argv[ai], "--cpu-replay-csv") == 0 && (ai + 1) < argc) {
            cpu_replay_csv_path = argv[++ai];
        } else {
            std::fprintf(stderr, "Unknown arg: %s\n", argv[ai]);
            return 2;
        }
    }

    if (accel_source == ACCEL_CPU_PREV && cpu_replay_csv_path == nullptr) {
        std::fprintf(stderr, "--cpu-replay-csv is required when --accel-source cpu-prev\n");
        return 2;
    }

    std::FILE *in = std::fopen(argv[1], "r");
    if (!in) {
        std::perror("open state_csv");
        return 3;
    }

    std::FILE *out = std::fopen(argv[2], "w");
    if (!out) {
        std::perror("open out_csv");
        std::fclose(in);
        return 4;
    }

    std::FILE *cpu_in = nullptr;
    char cpu_line[4096];
    if (accel_source == ACCEL_CPU_PREV) {
        cpu_in = std::fopen(cpu_replay_csv_path, "r");
        if (!cpu_in) {
            std::perror("open cpu_replay_csv");
            std::fclose(in);
            std::fclose(out);
            return 5;
        }
        if (!std::fgets(cpu_line, sizeof(cpu_line), cpu_in)) {
            std::fprintf(stderr, "CPU replay CSV is empty\n");
            std::fclose(cpu_in);
            std::fclose(in);
            std::fclose(out);
            return 6;
        }
    }

    char line[65536];
    if (!std::fgets(line, sizeof(line), in)) {
        std::fprintf(stderr, "state CSV empty\n");
        if (cpu_in) std::fclose(cpu_in);
        std::fclose(in);
        std::fclose(out);
        return 7;
    }

    std::fprintf(out,
                 "idx,stamp_ns,ey,epsi,vx,vy,omega,delta,a_cmd,kappa,ref_vx");
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            std::fprintf(out, ",A%d%d", i, j);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 2; j++) {
            std::fprintf(out, ",B%d%d", i, j);
        }
    }
    for (int i = 0; i < 5; i++) {
        std::fprintf(out, ",next%d", i);
    }
    std::fprintf(out, "\n");

    while (std::fgets(line, sizeof(line), in)) {
        ReplayRow r{};
        int32_t ey_fp = 0;
        int32_t epsi_fp = 0;
        fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET];
        fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU];
        fp_QP_t next_state[MPC_NX_FRENET];

        if (!parse_row(line, r)) {
            std::fprintf(stderr, "Skipping malformed state row\n");
            continue;
        }

        compute_frenet_errors_fp(r, ey_fp, epsi_fp);

        float a_cmd_f = 0.0f;
        if (accel_source == ACCEL_CPU_PREV) {
            if (!std::fgets(cpu_line, sizeof(cpu_line), cpu_in)) {
                std::fprintf(stderr, "CPU replay CSV ended before state CSV\n");
                break;
            }
            uint64_t cpu_idx = 0;
            int32_t prev_accel_fp = 0;
            if (!parse_cpu_prev_accel(cpu_line, &cpu_idx, &prev_accel_fp)) {
                std::fprintf(stderr, "Malformed CPU replay row\n");
                continue;
            }
            if (cpu_idx != r.idx) {
                std::fprintf(stderr,
                             "Index mismatch state/cpu-replay: state=%llu cpu=%llu\n",
                             (unsigned long long)r.idx,
                             (unsigned long long)cpu_idx);
                continue;
            }
            a_cmd_f = fp_to_float(prev_accel_fp);
        }

        const fp_QP_t ey = fp_to_float(ey_fp);
        const fp_QP_t epsi = fp_to_float(epsi_fp);
        const fp_QP_t vx = fp_to_float(r.velocity_fp);
        const fp_QP_t vy = fp_to_float(r.vy_fp);
        const fp_QP_t omega = fp_to_float(r.omega_fp);
        const fp_QP_t delta = fp_to_float(r.steering_angle_fp);
        const fp_QP_t a_cmd = a_cmd_f;
        const fp_QP_t kappa = fp_to_float(r.ref_kappa_fp[0]);
        const fp_QP_t ref_vx = fp_to_float(r.ref_vx_fp[0]);

        compute_frenet_AB_and_next_hls(
            ey, epsi, vx, vy, omega, delta, a_cmd, kappa, ref_vx, A_fr, B_fr,
            next_state);

        std::fprintf(out,
                     "%llu,%lld,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                     static_cast<unsigned long long>(r.idx),
                     static_cast<long long>(r.stamp_ns),
                     static_cast<double>(ey),
                     static_cast<double>(epsi),
                     static_cast<double>(vx),
                     static_cast<double>(vy),
                     static_cast<double>(omega),
                     static_cast<double>(delta),
                     static_cast<double>(a_cmd),
                     static_cast<double>(kappa),
                     static_cast<double>(ref_vx));

        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                std::fprintf(out, ",%.9g", static_cast<double>(A_fr[i][j]));
            }
        }
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 2; j++) {
                std::fprintf(out, ",%.9g", static_cast<double>(B_fr[i][j]));
            }
        }
        for (int i = 0; i < 5; i++) {
            std::fprintf(out, ",%.9g", static_cast<double>(next_state[i]));
        }
        std::fprintf(out, "\n");
    }

    if (cpu_in) std::fclose(cpu_in);
    std::fclose(in);
    std::fclose(out);
    return 0;
}
