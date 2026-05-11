/**
 * @file benchmark_vehicle_model_arm.c
 * @brief Standalone timing benchmark for the CPU vehicle model.
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "vehicle_model.h"

static int parse_positive_arg(const char *text, int fallback)
{
    if (!text || !text[0]) {
        return fallback;
    }

    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (end == text || value <= 0 || value > 100000000L) {
        return fallback;
    }
    return (int)value;
}

static double elapsed_ns(const struct timespec *start, const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec) * 1.0e9 +
           (double)(end->tv_nsec - start->tv_nsec);
}

static void initialize_inputs(FrenetState_t *frenet_state, ControlInput_t *control_input, VehicleState_t *vehicle_state)
{
    frenet_state->flat_error = 0.18f;
    frenet_state->fhead_error = 0.03f;
    frenet_state->flong_vel = 5.5f;
    frenet_state->flat_vel = 0.08f;
    frenet_state->fyaw_rate = 0.02f;

    control_input->steer_ang = 0.05f;
    control_input->long_acc = 0.75f;

    vehicle_state->pos_x = 1.0f;
    vehicle_state->pos_y = 0.5f;
    vehicle_state->heading = 0.02f;
    vehicle_state->long_vel = frenet_state->flong_vel;
    vehicle_state->lat_vel = frenet_state->flat_vel;
    vehicle_state->yaw_rate = frenet_state->fyaw_rate;
}

static double benchmark_frenet_linearization(
    const FrenetState_t *frenet_state,
    const ControlInput_t *control_input,
    float time_step,
    float path_curvature,
    float reference_velocity,
    int warmup_iterations,
    int timed_iterations,
    volatile float *checksum)
{
    float A[NX_FRENET][NX_FRENET];
    float B[NX_FRENET][2];

    for (int i = 0; i < warmup_iterations; ++i) {
        vehicle_model_compute_frenet_linearization(
            frenet_state, control_input, time_step, path_curvature,
            reference_velocity, A, B);
    }

    struct timespec start;
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (int i = 0; i < timed_iterations; ++i) {
        vehicle_model_compute_frenet_linearization(
            frenet_state, control_input, time_step, path_curvature,
            reference_velocity, A, B);
        *checksum += A[0][1] + A[1][0] + A[2][2] + A[3][3] + A[4][4] + B[2][0] + B[3][1];
    }
    clock_gettime(CLOCK_MONOTONIC, &end);

    return elapsed_ns(&start, &end) / (double)timed_iterations;
}

static double benchmark_predict_next_state(
    const VehicleState_t *vehicle_state,
    const ControlInput_t *control_input,
    float time_step,
    int warmup_iterations,
    int timed_iterations,
    volatile float *checksum)
{
    VehicleState_t next_state;

    for (int i = 0; i < warmup_iterations; ++i) {
        next_state = vehicle_model_predict_next_state(vehicle_state, control_input, time_step);
    }

    struct timespec start;
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (int i = 0; i < timed_iterations; ++i) {
        next_state = vehicle_model_predict_next_state(vehicle_state, control_input, time_step);
        *checksum += next_state.pos_x + next_state.pos_y + next_state.heading +
                     next_state.long_vel + next_state.lat_vel + next_state.yaw_rate;
    }
    clock_gettime(CLOCK_MONOTONIC, &end);

    return elapsed_ns(&start, &end) / (double)timed_iterations;
}

int main(int argc, char **argv)
{
    const int timed_iterations = parse_positive_arg((argc > 1) ? argv[1] : NULL, 20000);
    const int warmup_iterations = parse_positive_arg((argc > 2) ? argv[2] : NULL, 500);

    FrenetState_t frenet_state;
    ControlInput_t control_input;
    VehicleState_t vehicle_state;
    initialize_inputs(&frenet_state, &control_input, &vehicle_state);

    const float time_step = TIME_STEP_SECONDS;
    const float path_curvature = 0.015f;
    const float reference_velocity = 5.8f;

    volatile float checksum = 0.0f;

    double frenet_ns = benchmark_frenet_linearization(
        &frenet_state, &control_input, time_step, path_curvature,
        reference_velocity, warmup_iterations, timed_iterations, &checksum);
    double predict_ns = benchmark_predict_next_state(
        &vehicle_state, &control_input, time_step,
        warmup_iterations, timed_iterations, &checksum);

    printf("Standalone CPU vehicle model benchmark\n");
    printf("  warmup iterations: %d\n", warmup_iterations);
    printf("  timed iterations : %d\n", timed_iterations);
    printf("  vehicle_model_compute_frenet_linearization: %.2f ns/call (%.3f us/call)\n",
           frenet_ns, frenet_ns / 1000.0);
    printf("  vehicle_model_predict_next_state: %.2f ns/call (%.3f us/call)\n",
           predict_ns, predict_ns / 1000.0);
    printf("  checksum: %.6f\n", (double)checksum);
    return 0;
}
