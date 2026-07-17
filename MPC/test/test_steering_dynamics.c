#include "mpc.h"
#include "steering_dynamics.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static void check_close(float actual, float expected, float tolerance,
                        const char *message)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr,
                "FAIL: %s (actual=%.9g expected=%.9g tolerance=%.3g)\n",
                message, (double)actual, (double)expected,
                (double)tolerance);
        failures++;
    }
}

static float effective_derivative(float time_seconds, float effective,
                                  float command_initial, float steering_rate)
{
    const float command = command_initial + steering_rate * time_seconds;
    return (command - effective) /
           STEERING_EFFECTIVE_TIME_CONSTANT_SECONDS;
}

static void numerical_ramp_solution(float dt_seconds, float effective_initial,
                                    float command_initial, float steering_rate,
                                    float *effective_final,
                                    float *effective_average)
{
    const int integration_steps = 30000;
    const float h = dt_seconds / (float)integration_steps;
    float effective = effective_initial;
    double integral = 0.0;

    for (int step = 0; step < integration_steps; step++) {
        const float time = (float)step * h;
        const float before = effective;
        const float k1 = effective_derivative(
            time, effective, command_initial, steering_rate);
        const float k2 = effective_derivative(
            time + 0.5f * h, effective + 0.5f * h * k1,
            command_initial, steering_rate);
        const float k3 = effective_derivative(
            time + 0.5f * h, effective + 0.5f * h * k2,
            command_initial, steering_rate);
        const float k4 = effective_derivative(
            time + h, effective + h * k3,
            command_initial, steering_rate);
        effective += (h / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
        integral += 0.5 * (double)h * ((double)before + (double)effective);
    }

    *effective_final = effective;
    *effective_average = (float)(integral / (double)dt_seconds);
}

static void test_exact_prediction_coefficients(void)
{
    const SteeringDynamicsCoefficients_t coefficients =
        steering_dynamics_coefficients(PREDICTION_DT_SECONDS);

    check_close(coefficients.retention, 0.301194212f, 2e-7f,
                "30 ms pole retention");
    check_close(coefficients.command_gain, 0.698805788f, 2e-7f,
                "30 ms command gain");
    check_close(coefficients.rate_gain_seconds, 0.0125298553f, 2e-7f,
                "30 ms steering-rate gain");
    check_close(coefficients.average_effective_gain, 0.582338157f, 2e-7f,
                "interval-average effective-angle gain");
    check_close(coefficients.average_command_gain, 0.417661843f, 2e-7f,
                "interval-average command-angle gain");
    check_close(coefficients.average_rate_gain_seconds, 0.00455845392f,
                2e-7f, "interval-average steering-rate gain");

    {
        const float effective_initial = -0.12f;
        const float command_initial = 0.20f;
        const float steering_rate = -1.30f;
        float numerical_final;
        float numerical_average;
        numerical_ramp_solution(
            PREDICTION_DT_SECONDS, effective_initial, command_initial,
            steering_rate, &numerical_final, &numerical_average);

        check_close(
            steering_dynamics_next_effective(
                effective_initial, command_initial, steering_rate,
                &coefficients),
            numerical_final, 3e-6f,
            "exact ramp transition must match numerical integration");
        check_close(
            steering_dynamics_interval_average(
                effective_initial, command_initial, steering_rate,
                &coefficients),
            numerical_average, 3e-6f,
            "interval-average steering must match numerical integration");
    }
}

static void test_control_rate_estimator_response(void)
{
    const SteeringDynamicsCoefficients_t coefficients =
        steering_dynamics_coefficients(CONTROL_DT_SECONDS);
    float effective = 0.0f;

    effective = steering_dynamics_next_effective(
        effective, 1.0f, 0.0f, &coefficients);
    check_close(effective, 0.181269247f, 2e-7f,
                "one 5 ms update must reach 18.1269 percent");

    for (int update = 1; update < 5; update++) {
        effective = steering_dynamics_next_effective(
            effective, 1.0f, 0.0f, &coefficients);
    }
    check_close(effective, 0.632120559f, 5e-7f,
                "five 5 ms updates must reach 63.2121 percent");
}

static void build_straight_reference(
    TrajectoryReferencePoint_t reference[PREDICTION_HORIZON])
{
    memset(reference, 0,
           sizeof(TrajectoryReferencePoint_t) * PREDICTION_HORIZON);
    for (int k = 0; k < PREDICTION_HORIZON; k++) {
        reference[k].reference_velocity = 3.0f;
        reference[k].left_wall_bound = 5.0f;
        reference[k].right_wall_bound = 5.0f;
    }
}

static void test_mpc_estimator_initialization_and_reset(void)
{
    MpcConfiguration_t configuration = get_default_configuration();
    FrenetState_t state = {0};
    TrajectoryReferencePoint_t reference[PREDICTION_HORIZON];
    MpcSolverResult_t result;
    float x_plan[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
    float u_plan[PREDICTION_HORIZON][RICCATI_MAX_NU];
    ControlInput_t command = {.steer_ang = 0.20f, .long_acc = 0.0f};

    configuration.time_step = PREDICTION_DT_SECONDS;
    configuration.max_solver_iterations = 50;
    state.flong_vel = 3.0f;
    build_straight_reference(reference);
    mpc_initialize_with_configuration(&configuration);

    mpc_set_previous_command(&command);
    (void)mpc_compute_optimal_control(&state, reference, &result);
    check_true(mpc_debug_copy_last_plan(x_plan, u_plan),
               "first solve must expose a plan");
    check_close(x_plan[0][IDX_DELTA_COMMAND], 0.20f, 1e-6f,
                "first command sample initializes command state");
    check_close(x_plan[0][IDX_DELTA_EFFECTIVE], 0.20f, 1e-6f,
                "first command sample initializes effective state");

    command.steer_ang = -0.10f;
    mpc_set_previous_command(&command);
    (void)mpc_compute_optimal_control(&state, reference, &result);
    check_true(mpc_debug_copy_last_plan(x_plan, u_plan),
               "second solve must expose a plan");
    check_close(x_plan[0][IDX_DELTA_COMMAND], -0.10f, 1e-6f,
                "command state must use the latest target");
    check_close(x_plan[0][IDX_DELTA_EFFECTIVE], 0.145619228f, 2e-6f,
                "effective state advances once over the 5 ms interval");

    {
        const float expected_output = util_clamp(
            -0.10f + configuration.time_step * u_plan[0][0],
            -VP_MAX_STEERING_RAD, VP_MAX_STEERING_RAD);
        check_close(result.optimal_control.steer_ang, expected_output, 2e-5f,
                    "published target integrates from command, not effective angle");
    }

    mpc_reset();
    check_true(!mpc_debug_copy_last_plan(x_plan, u_plan),
               "reset must clear the warm-start plan");
    mpc_set_previous_command(&command);
    (void)mpc_compute_optimal_control(&state, reference, &result);
    check_true(mpc_debug_copy_last_plan(x_plan, u_plan),
               "post-reset solve must expose a plan");
    check_close(x_plan[0][IDX_DELTA_COMMAND], -0.10f, 1e-6f,
                "reset command state reinitializes from first sample");
    check_close(x_plan[0][IDX_DELTA_EFFECTIVE], -0.10f, 1e-6f,
                "reset effective state reinitializes from first sample");
}

static void test_command_callbacks_do_not_advance_the_estimator(void)
{
    MpcConfiguration_t configuration = get_default_configuration();
    FrenetState_t state = {0};
    TrajectoryReferencePoint_t reference[PREDICTION_HORIZON];
    MpcSolverResult_t result;
    float x_plan[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
    float u_plan[PREDICTION_HORIZON][RICCATI_MAX_NU];
    ControlInput_t command = {0};

    configuration.time_step = PREDICTION_DT_SECONDS;
    state.flong_vel = 3.0f;
    build_straight_reference(reference);
    mpc_initialize_with_configuration(&configuration);

    mpc_set_previous_command(&command);
    command.steer_ang = 0.05f;
    mpc_set_previous_command(&command);
    command.steer_ang = 0.10f;
    mpc_set_previous_command(&command);
    command.steer_ang = 0.20f;
    mpc_set_previous_command(&command);

    (void)mpc_compute_optimal_control(&state, reference, &result);
    check_true(mpc_debug_copy_last_plan(x_plan, u_plan),
               "callback-cadence solve must expose a plan");
    check_close(x_plan[0][IDX_DELTA_COMMAND], 0.20f, 1e-6f,
                "latest callback supplies the command state");
    check_close(x_plan[0][IDX_DELTA_EFFECTIVE], 0.0362538494f, 2e-6f,
                "multiple callbacks must still advance the pole only once");
}

int main(void)
{
    test_exact_prediction_coefficients();
    test_control_rate_estimator_response();
    test_mpc_estimator_initialization_and_reset();
    test_command_callbacks_do_not_advance_the_estimator();

    if (failures != 0) {
        fprintf(stderr, "%d steering-dynamics regression(s) failed\n",
                failures);
        return 1;
    }

    puts("Steering-dynamics regressions passed");
    return 0;
}
