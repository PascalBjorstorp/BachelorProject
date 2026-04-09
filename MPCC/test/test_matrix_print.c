/*******************************************************************************
 * test_matrix_print.c — Print MPCC cost matrices (Q, R) for inspection
 *
 * Shows the stage cost Q (NX×NX) and R (NU×NU) matrices as they appear
 * in the QP, so you can see which entries are zero vs non-zero and how
 * significant the off-diagonal elements are.
 *
 * Build: gcc -I../include -I../../MPC/include test_matrix_print.c \
 *        ../src/qp_solver_mpcc.c ../src/mpcc.c ../../MPC/src/fp_math.c \
 *        ../../MPC/src/vehicle_model.c -lm -o test_matrix_print
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mpcc_types.h"
#include "qp_solver_mpcc.h"
#include "mpcc.h"

/* ── State/Control names for readable output ─────────────────────────── */

static const char *state_names[MPCC_NX] = {
    "s    ", "vx   ", "vy   ",
    "omega", "X    ", "Y    ", "psi  "
};

static const char *ctrl_names[MPCC_NU] = {
    "delta", "a_x  ", "v_th "
};

/* ── Print a NX×NX matrix with labels ─────────────────────────────────── */

static void print_Q_matrix(const char *title,
                           const float Q[MPCC_NX][MPCC_NX])
{
    printf("\n%s (NX=%d x NX=%d):\n", title, MPCC_NX, MPCC_NX);

    /* Header row */
    printf("         ");
    for (int j = 0; j < MPCC_NX; j++) {
        printf(" %9s", state_names[j]);
    }
    printf("\n");

    /* Separator */
    printf("         ");
    for (int j = 0; j < MPCC_NX; j++) {
        printf(" ---------");
    }
    printf("\n");

    /* Data rows */
    int off_diag_count = 0;
    float max_off_diag = 0.0f;

    for (int i = 0; i < MPCC_NX; i++) {
        printf("  %s |", state_names[i]);
        for (int j = 0; j < MPCC_NX; j++) {
            float val = Q[i][j];
            if (i != j && fabsf(val) > 1e-6f) {
                /* Highlight non-zero off-diagonal with asterisk */
                printf(" %8.4f*", val);
                off_diag_count++;
                if (fabsf(val) > max_off_diag)
                    max_off_diag = fabsf(val);
            } else if (i == j && fabsf(val) > 1e-6f) {
                /* Highlight diagonal */
                printf(" [%7.4f]", val);
            } else {
                printf(" %9.4f", val);
            }
        }
        printf("\n");
    }

    printf("\n  Summary: %d non-zero off-diagonal entries", off_diag_count);
    if (off_diag_count > 0) {
        printf(" (max magnitude: %.6f)", max_off_diag);
    }
    printf("\n  Matrix is %s\n",
           off_diag_count == 0 ? "DIAGONAL" : "DENSE (has off-diagonal terms)");
}

/* ── Print a NU×NU matrix with labels ─────────────────────────────────── */

static void print_R_matrix(const char *title,
                           const float R[MPCC_NU][MPCC_NU])
{
    printf("\n%s (NU=%d x NU=%d):\n", title, MPCC_NU, MPCC_NU);

    printf("         ");
    for (int j = 0; j < MPCC_NU; j++) {
        printf(" %9s", ctrl_names[j]);
    }
    printf("\n         ");
    for (int j = 0; j < MPCC_NU; j++) {
        printf(" ---------");
    }
    printf("\n");

    int off_diag_count = 0;
    for (int i = 0; i < MPCC_NU; i++) {
        printf("  %s |", ctrl_names[i]);
        for (int j = 0; j < MPCC_NU; j++) {
            float val = R[i][j];
            if (i != j && fabsf(val) > 1e-6f) {
                printf(" %8.4f*", val);
                off_diag_count++;
            } else if (i == j) {
                printf(" [%7.4f]", val);
            } else {
                printf(" %9.4f", val);
            }
        }
        printf("\n");
    }

    printf("\n  Matrix is %s\n",
           off_diag_count == 0 ? "DIAGONAL" : "DENSE (has off-diagonal terms)");
}

/* ── Print linear cost vectors ───────────────────────────────────────── */

static void print_q_vector(const char *title,
                           const float q[MPCC_NX])
{
    printf("\n%s (NX=%d):\n", title, MPCC_NX);
    for (int i = 0; i < MPCC_NX; i++) {
        float val = q[i];
        if (fabsf(val) > 1e-6f) {
            printf("  q[%s] = %10.6f  <-- active\n", state_names[i], val);
        } else {
            printf("  q[%s] = %10.6f\n", state_names[i], val);
        }
    }
}

static void print_r_vector(const char *title,
                           const float r[MPCC_NU])
{
    printf("\n%s (NU=%d):\n", title, MPCC_NU);
    for (int i = 0; i < MPCC_NU; i++) {
        float val = r[i];
        if (fabsf(val) > 1e-6f) {
            printf("  r[%s] = %10.6f  <-- active\n", ctrl_names[i], val);
        } else {
            printf("  r[%s] = %10.6f\n", ctrl_names[i], val);
        }
    }
}

/* ── Print S cross-term (NU×NX) ──────────────────────────────────────── */

static void print_S_matrix(const char *title,
                           const float S[MPCC_NU][MPCC_NX])
{
    printf("\n%s (NU=%d x NX=%d):\n", title, MPCC_NU, MPCC_NX);

    printf("         ");
    for (int j = 0; j < MPCC_NX; j++) {
        printf(" %9s", state_names[j]);
    }
    printf("\n         ");
    for (int j = 0; j < MPCC_NX; j++) {
        printf(" ---------");
    }
    printf("\n");

    int nonzero = 0;
    for (int i = 0; i < MPCC_NU; i++) {
        printf("  %s |", ctrl_names[i]);
        for (int j = 0; j < MPCC_NX; j++) {
            float val = S[i][j];
            if (fabsf(val) > 1e-6f) {
                printf(" %8.4f*", val);
                nonzero++;
            } else {
                printf(" %9.4f", val);
            }
        }
        printf("\n");
    }

    printf("\n  Cross-term has %d non-zero entries%s\n",
           nonzero, nonzero == 0 ? " (all zero)" : "");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Build stage costs the same way mpcc.c does (mimicking build_stage_cost)
 * ═══════════════════════════════════════════════════════════════════════ */

static void build_cost_from_defaults(MPCCStageCost_t *cost, int is_terminal)
{
    memset(cost, 0, sizeof(*cost));

    /* Quadratic state costs (diagonal) */
    if (is_terminal) {
        /* No n/alpha in 7-state model */
    } else {
        /* No n/alpha in 7-state model */
    }

    /* State regularization */
    cost->Q[MPCC_IDX_VY][MPCC_IDX_VY] = MPCC_DEFAULT_WEIGHT_VY;
    cost->Q[MPCC_IDX_OMEGA][MPCC_IDX_OMEGA] = MPCC_DEFAULT_WEIGHT_OMEGA;

    /* Velocity tracking */
    if (MPCC_DEFAULT_WEIGHT_VX > 0) {
        cost->Q[MPCC_IDX_VX][MPCC_IDX_VX] = MPCC_DEFAULT_WEIGHT_VX;
        cost->q[MPCC_IDX_VX] = -(2.0f * MPCC_DEFAULT_WEIGHT_VX * MPCC_DEFAULT_VX_REF);
    }

    /* Progress reward */
    float q_s = is_terminal ?
        MPCC_DEFAULT_WEIGHT_PROGRESS_TERMINAL : MPCC_DEFAULT_WEIGHT_PROGRESS;
    cost->q[MPCC_IDX_S] = (0 - q_s);

    /* Control costs */
    if (!is_terminal) {
        cost->R[MPCC_IDX_DELTA][MPCC_IDX_DELTA] =
            (MPCC_DEFAULT_WEIGHT_DELTA + MPCC_DEFAULT_WEIGHT_DELTA_RATE);
        cost->R[MPCC_IDX_AX][MPCC_IDX_AX] =
            (MPCC_DEFAULT_WEIGHT_AX + MPCC_DEFAULT_WEIGHT_AX_RATE);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("============================================================\n");
    printf("  MPCC Cost Matrix Inspector\n");
    printf("  NX=%d  NU=%d  (Global Frame: 7-state Liniger MPCC)\n", MPCC_NX, MPCC_NU);
    printf("============================================================\n");

    /* --- Stage cost (running cost at k=0..N-1) --- */
    printf("\n████████████████████████████████████████████████████████████\n");
    printf("  RUNNING STAGE COST (k = 0 .. N-1)\n");
    printf("████████████████████████████████████████████████████████████\n");

    MPCCStageCost_t stage;
    build_cost_from_defaults(&stage, 0);

    print_Q_matrix("Q (Quadratic State Cost)", stage.Q);
    print_q_vector("q (Linear State Cost)", stage.q);
    print_R_matrix("R (Quadratic Control Cost)", stage.R);
    print_r_vector("r (Linear Control Cost)", stage.r);
    print_S_matrix("S (State-Control Cross Term)", stage.S);

    /* --- Terminal cost (k = N) --- */
    printf("\n████████████████████████████████████████████████████████████\n");
    printf("  TERMINAL COST (k = N)\n");
    printf("████████████████████████████████████████████████████████████\n");

    MPCCStageCost_t terminal;
    build_cost_from_defaults(&terminal, 1);

    print_Q_matrix("Q_N (Terminal Quadratic State Cost)", terminal.Q);
    print_q_vector("q_N (Terminal Linear State Cost)", terminal.q);

    /* --- ADMM-augmented cost (what backward pass actually uses) --- */
    printf("\n████████████████████████████████████████████████████████████\n");
    printf("  ADMM-AUGMENTED COST (Q_tilde = Q + rho*I on constrained dims)\n");
    printf("████████████████████████████████████████████████████████████\n");

    /* Simulate selective augmentation */
    float rho_default = MPCC_DEFAULT_ADMM_RHO;
    float x_lower[MPCC_NX], x_upper[MPCC_NX];
    for (int i = 0; i < MPCC_NX; i++) {
        x_lower[i] = -1000.0f;
        x_upper[i] = 1000.0f;
    }
    /* Tight bounds on vx */
    x_upper[MPCC_IDX_VX] = 8.0f;
    x_lower[MPCC_IDX_VX] = 0.5f;

    printf("\n  Default rho = %.2f\n", rho_default);
    printf("  Global bounds (constrained if |bound| < 100):\n");
    for (int i = 0; i < MPCC_NX; i++) {
        float lb = x_lower[i];
        float ub = x_upper[i];
        int constrained = (ub < 100.0f || lb > -100.0f);
        printf("    %s:  [%8.2f, %8.2f]  %s\n",
               state_names[i], lb, ub,
               constrained ? "<-- CONSTRAINED (rho added)" : "unconstrained");
    }

    /* Build Q_tilde with selective augmentation */
    float Q_tilde[MPCC_NX][MPCC_NX];
    memcpy(Q_tilde, stage.Q, sizeof(Q_tilde));
    for (int i = 0; i < MPCC_NX; i++) {
        int constrained = (x_upper[i] < 100.0f ||
                           x_lower[i] > -100.0f);
        if (constrained) {
            Q_tilde[i][i] = (Q_tilde[i][i] + rho_default);
        }
    }

    print_Q_matrix("Q_tilde (Q + selective rho*I)", Q_tilde);

    /* --- Cost magnitude comparison --- */
    printf("\n████████████████████████████████████████████████████████████\n");
    printf("  DIAGONAL VALUE SUMMARY\n");
    printf("████████████████████████████████████████████████████████████\n\n");

    printf("  %-10s  %10s  %10s  %10s\n",
           "State", "Q[i][i]", "Q_tilde", "rho added?");
    printf("  %-10s  %10s  %10s  %10s\n",
           "----------", "----------", "----------", "----------");
    for (int i = 0; i < MPCC_NX; i++) {
        float q_val = stage.Q[i][i];
        float qt_val = Q_tilde[i][i];
        int constrained = (x_upper[i] < 100.0f ||
                           x_lower[i] > -100.0f);
        printf("  %-10s  %10.4f  %10.4f  %10s\n",
               state_names[i], q_val, qt_val,
               constrained ? "YES" : "no");
    }

    printf("\n  Control costs:\n");
    printf("  %-10s  %10s\n", "Control", "R[i][i]");
    printf("  %-10s  %10s\n", "----------", "----------");
    for (int i = 0; i < MPCC_NU; i++) {
        printf("  %-10s  %10.4f\n", ctrl_names[i], stage.R[i][i]);
    }

    printf("\n============================================================\n");
    printf("  Done. If off-diagonal count = 0, Q is purely diagonal\n");
    printf("  and the dense NX×NX storage has no overhead cost.\n");
    printf("============================================================\n");

    return 0;
}
