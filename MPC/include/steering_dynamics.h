/**
 * @file steering_dynamics.h
 * @brief Exact coefficients for the CPU MPC effective-steering pole.
 */

#ifndef STEERING_DYNAMICS_H
#define STEERING_DYNAMICS_H

#include "mpc_types.h"

#include <math.h>

typedef struct
{
    float retention;
    float command_gain;
    float rate_gain_seconds;
    float average_effective_gain;
    float average_command_gain;
    float average_rate_gain_seconds;
} SteeringDynamicsCoefficients_t;

/**
 * @brief Build the exact zero-order-hold pole coefficients for one interval.
 * @details The commanded angle is a ramp with constant steering rate over the
 *          interval. The average coefficients let the vehicle dynamics use
 *          the steering that acts throughout the interval, avoiding an extra
 *          one-stage delay.
 */
static inline SteeringDynamicsCoefficients_t
steering_dynamics_coefficients(float dt_seconds)
{
    const float tau = STEERING_EFFECTIVE_TIME_CONSTANT_SECONDS;
    const float retention = expf(-dt_seconds / tau);
    const float command_gain = 1.0f - retention;
    SteeringDynamicsCoefficients_t coefficients;

    coefficients.retention = retention;
    coefficients.command_gain = command_gain;
    coefficients.rate_gain_seconds = dt_seconds - tau * command_gain;
    coefficients.average_effective_gain = tau * command_gain / dt_seconds;
    coefficients.average_command_gain =
        1.0f - coefficients.average_effective_gain;
    coefficients.average_rate_gain_seconds =
        0.5f * dt_seconds - tau +
        (tau * tau * command_gain / dt_seconds);

    return coefficients;
}

static inline float steering_dynamics_next_effective(
    float effective_angle,
    float commanded_angle,
    float steering_rate,
    const SteeringDynamicsCoefficients_t *coefficients)
{
    return coefficients->retention * effective_angle +
           coefficients->command_gain * commanded_angle +
           coefficients->rate_gain_seconds * steering_rate;
}

static inline float steering_dynamics_interval_average(
    float effective_angle,
    float commanded_angle,
    float steering_rate,
    const SteeringDynamicsCoefficients_t *coefficients)
{
    return coefficients->average_effective_gain * effective_angle +
           coefficients->average_command_gain * commanded_angle +
           coefficients->average_rate_gain_seconds * steering_rate;
}

#endif /* STEERING_DYNAMICS_H */
