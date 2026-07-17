/**
 * @file test_frenet_linearization.c
 * @brief Finite-difference checks for the MPC Frenet kinematic Jacobian.
 *
 * Standalone build:
 *   cc -O2 -std=c99 -Wall -Wextra -Wpedantic -Iinclude \
 *      test/test_frenet_linearization.c src/vehicle_model.c src/util_math.c \
 *      -lm -o /tmp/test_frenet_linearization
 */

#include "vehicle_model.h"

#include <math.h>
#include <stdio.h>

static void predict_frenet_kinematics(
    const FrenetState_t *state,
    float dt,
    float curvature,
    float out[2])
{
    const float v_eff = (state->flong_vel > MIN_LINEARIZATION_VELOCITY)
                            ? state->flong_vel
                            : MIN_LINEARIZATION_VELOCITY;
    const float cp = cosf(state->fhead_error);
    const float sp = sinf(state->fhead_error);
    float denom = 1.0f - curvature * state->flat_error;
    if (fabsf(denom) < 1e-3f)
        denom = (denom >= 0.0f) ? 1e-3f : -1e-3f;

    const float ey_dot = v_eff * sp + state->flat_vel * cp;
    const float s_dot = v_eff * cp / denom;
    const float epsi_dot = state->fyaw_rate - curvature * s_dot;

    out[0] = state->flat_error + dt * ey_dot;
    out[1] = state->fhead_error + dt * epsi_dot;
}

static float *state_component(FrenetState_t *state, int index)
{
    switch (index) {
    case 0: return &state->flat_error;
    case 1: return &state->fhead_error;
    case 2: return &state->flong_vel;
    case 3: return &state->flat_vel;
    case 4: return &state->fyaw_rate;
    default: return NULL;
    }
}

static int check_case(
    const char *name,
    const FrenetState_t *state,
    float curvature,
    float tolerance)
{
    const float dt = 0.03f;
    const float epsilon = 1e-3f;
    const ControlInput_t control = {.steer_ang = 0.11f, .long_acc = 0.4f};
    float A[NX_FRENET][NX_FRENET];
    float B[NX_FRENET][2];
    float worst_error = 0.0f;
    int worst_row = -1;
    int worst_col = -1;

    vehicle_model_compute_frenet_linearization(
        state, &control, dt, curvature, state->flong_vel, A, B);

    for (int col = 0; col < NX_FRENET; ++col) {
        FrenetState_t plus = *state;
        FrenetState_t minus = *state;
        *state_component(&plus, col) += epsilon;
        *state_component(&minus, col) -= epsilon;

        float f_plus[2];
        float f_minus[2];
        predict_frenet_kinematics(&plus, dt, curvature, f_plus);
        predict_frenet_kinematics(&minus, dt, curvature, f_minus);

        for (int row = 0; row < 2; ++row) {
            const float numerical =
                (f_plus[row] - f_minus[row]) / (2.0f * epsilon);
            if (!isfinite(A[row][col]) || !isfinite(numerical)) {
                fprintf(stderr,
                        "[FAIL] %s: non-finite Jacobian value at A[%d][%d]\n",
                        name, row, col);
                return 0;
            }
            const float error = fabsf(A[row][col] - numerical);
            if (error > worst_error) {
                worst_error = error;
                worst_row = row;
                worst_col = col;
            }
        }
    }

    if (worst_error > tolerance) {
        fprintf(stderr,
                "[FAIL] %s: worst |analytic-numeric|=%g at A[%d][%d]\n",
                name, (double)worst_error, worst_row, worst_col);
        return 0;
    }

    printf("[PASS] %s: worst Jacobian error=%g\n", name, (double)worst_error);
    return 1;
}

int main(void)
{
    const FrenetState_t nominal = {
        .flat_error = 0.20f,
        .fhead_error = 0.25f,
        .flong_vel = 4.20f,
        .flat_vel = 0.60f,
        .fyaw_rate = 0.70f,
    };
    const FrenetState_t opposite_slip = {
        .flat_error = -0.35f,
        .fhead_error = -0.31f,
        .flong_vel = 7.50f,
        .flat_vel = -0.85f,
        .fyaw_rate = -1.10f,
    };
    int passed = 1;
    passed &= check_case("nominal", &nominal, 0.35f, 2.0e-4f);
    passed &= check_case("opposite slip", &opposite_slip, -0.28f, 2.0e-4f);

    return passed ? 0 : 1;
}
