#include "../include/mpc_internal_diag.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static MpcInternalVarStat g_stats[MPC_DIAG_VAR_COUNT];

static const char *k_var_names[MPC_DIAG_VAR_COUNT] = {
    "recip_abs_x",
    "recip_est",
    "recip_prod",
    "vm_vx_safe",
    "vm_d_f",
    "vm_d_r",
    "vm_c_eff_f",
    "vm_c_eff_r",
    "assembly_q_vx",
    "assembly_q_delta",
    "assembly_dff",
    "riccati_p",
    "riccati_m",
    "riccati_s",
    "riccati_g",
    "riccati_k",
    "admm_primal",
    "admm_dual",
    "admm_rho",
    "admm_rho_u",
};

void mpc_internal_diag_reset(void)
{
    for (int i = 0; i < MPC_DIAG_VAR_COUNT; ++i) {
        g_stats[i].min_val = 0.0;
        g_stats[i].max_val = 0.0;
        g_stats[i].abs_max_val = 0.0;
        g_stats[i].min_nonzero_delta = 0.0;
        g_stats[i].last_val = 0.0;
        g_stats[i].has_last = 0;
        g_stats[i].samples = 0;
    }
}

void mpc_internal_diag_observe(int var_id, double value)
{
#if MPC_VAR_DIAG_ENABLE
    if (var_id < 0 || var_id >= MPC_DIAG_VAR_COUNT) {
        return;
    }

    MpcInternalVarStat *s = &g_stats[var_id];
    const double abs_v = fabs(value);

    if (s->samples == 0) {
        s->min_val = value;
        s->max_val = value;
        s->abs_max_val = abs_v;
        s->min_nonzero_delta = 0.0;
        s->last_val = value;
        s->has_last = 1;
    } else {
        if (value < s->min_val) s->min_val = value;
        if (value > s->max_val) s->max_val = value;
        if (abs_v > s->abs_max_val) s->abs_max_val = abs_v;

        if (s->has_last) {
            double delta = fabs(value - s->last_val);
            if (delta > 0.0) {
                if (s->min_nonzero_delta == 0.0 || delta < s->min_nonzero_delta) {
                    s->min_nonzero_delta = delta;
                }
            }
        }
        s->last_val = value;
    }

    s->samples++;
#else
    (void)var_id;
    (void)value;
#endif
}

static int estimate_integer_bits(double abs_max)
{
    if (!(abs_max > 0.0)) {
        return 1;
    }
    double m = abs_max + 1.0;
    int magnitude_bits = (int)ceil(log(m) / log(2.0));
    return magnitude_bits + 1; /* signed */
}

static int estimate_fraction_bits(double min_nonzero_delta)
{
    if (!(min_nonzero_delta > 0.0)) {
        return 0;
    }
    double bits = ceil(-log(min_nonzero_delta) / log(2.0));
    if (bits < 0.0) bits = 0.0;
    if (bits > 40.0) bits = 40.0;
    return (int)bits;
}

void mpc_internal_diag_dump_csv(const char *path)
{
#if MPC_VAR_DIAG_ENABLE
    const char *out_path = (path && path[0] != '\0') ? path : "internal_var_stats.csv";
    FILE *f = fopen(out_path, "w");
    if (!f) {
        return;
    }

    fprintf(
        f,
        "var_id,var_name,samples,min_val,max_val,abs_max_val,min_nonzero_delta,int_bits_est,frac_bits_est,total_bits_est\n");
    for (int i = 0; i < MPC_DIAG_VAR_COUNT; ++i) {
        int int_bits = estimate_integer_bits(g_stats[i].abs_max_val);
        int frac_bits = estimate_fraction_bits(g_stats[i].min_nonzero_delta);
        int total_bits = int_bits + frac_bits;
        fprintf(
            f,
            "%d,%s,%llu,%.12g,%.12g,%.12g,%.12g,%d,%d,%d\n",
            i,
            k_var_names[i],
            (unsigned long long)g_stats[i].samples,
            g_stats[i].min_val,
            g_stats[i].max_val,
            g_stats[i].abs_max_val,
            g_stats[i].min_nonzero_delta,
            int_bits,
            frac_bits,
            total_bits
        );
    }

    fclose(f);
#else
    (void)path;
#endif
}
