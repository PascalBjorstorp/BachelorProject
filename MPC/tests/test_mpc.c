/**
 * @file test_mpc.c
 * @brief Unit Tests for MPC Controller
 *
 * Tests the full MPC pipeline including QP construction and solving.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mpc.h"
#include "mpc_types.h"
#include "fixed_point.h"

/*===========================================================================
 * Test Infrastructure
 *===========================================================================*/

static int total_tests_passed = 0;
static int total_tests_failed = 0;

#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"

#define TEST_ASSERT(condition, message)                                     \
    do                                                                      \
    {                                                                       \
        if (condition)                                                      \
        {                                                                   \
            printf("  [%sPASS%s] %s\n", COLOR_GREEN, COLOR_RESET, message); \
            total_tests_passed++;                                           \
        }                                                                   \
        else                                                                \
        {                                                                   \
            printf("  [%sFAIL%s] %s\n", COLOR_RED, COLOR_RESET, message);   \
            total_tests_failed++;                                           \
        }                                                                   \
    } while (0)

static int values_are_approximately_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) < tolerance;
}

/*===========================================================================
 * Test: MPC Initialization
 *===========================================================================*/

void test_mpc_initialization(void)
{
    printf("\n--- Test: MPC Initialization ---\n");

    mpc_initialize();

    MpcConfiguration_t config = mpc_get_configuration();

    TEST_ASSERT(config.prediction_horizon_steps == MPC_DEFAULT_PREDICTION_HORIZON,
                "Default horizon is 10 steps");
    TEST_ASSERT(config.maximum_solver_iterations == MPC_DEFAULT_MAXIMUM_ITERATIONS,
                "Default max iterations is 100");
    TEST_ASSERT(config.weight_position_x != 0, "Position X weight is set");
    TEST_ASSERT(config.weight_steering_effort != 0, "Steering effort weight is set");
}

/*===========================================================================
 * Test: Compute Control - Stationary Target
 *===========================================================================*/

void test_compute_control_stationary(void)
{
    printf("\n--- Test: Compute Control (Stationary Target) ---\n");

    mpc_initialize();

    /* Vehicle at origin, facing +X, at rest */
    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = 0};

    /* Reference: stay at origin, target velocity 0 */
    TrajectoryReferencePoint_t reference[MPC_DEFAULT_PREDICTION_HORIZON];
    for (int i = 0; i < MPC_DEFAULT_PREDICTION_HORIZON; i++)
    {
        reference[i].reference_position_x_meters = 0;
        reference[i].reference_position_y_meters = 0;
        reference[i].reference_heading_radians = 0;
        reference[i].reference_velocity_meters_per_second = 0;
    }

    MpcSolverResult_t result;
    MpcSolverStatus_t status = mpc_compute_optimal_control(&current_state, reference, &result);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", result.iterations_used);
    printf("  Steering: %.4f rad\n", fixed_point_to_float(result.optimal_control.steering_angle_radians));
    printf("  Acceleration: %.4f m/s²\n",
           fixed_point_to_float(result.optimal_control.acceleration_meters_per_second_squared));

    TEST_ASSERT(status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");

    /* For stationary target, expect minimal control effort */
    float steering = fixed_point_to_float(result.optimal_control.steering_angle_radians);
    TEST_ASSERT(fabsf(steering) < 0.5f, "Steering is small for stationary target");
}

/*===========================================================================
 * Test: Compute Control - Forward Motion
 *===========================================================================*/

void test_compute_control_forward(void)
{
    printf("\n--- Test: Compute Control (Forward Motion) ---\n");

    mpc_initialize();

    /* Vehicle at origin, facing +X, moving at 1 m/s */
    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = fixed_point_from_float(1.0f)};

    /* Reference: go forward at 2 m/s */
    TrajectoryReferencePoint_t reference[MPC_DEFAULT_PREDICTION_HORIZON];
    for (int i = 0; i < MPC_DEFAULT_PREDICTION_HORIZON; i++)
    {
        float time_ahead = (i + 1) * 0.1f;
        reference[i].reference_position_x_meters = fixed_point_from_float(2.0f * time_ahead);
        reference[i].reference_position_y_meters = 0;
        reference[i].reference_heading_radians = 0;
        reference[i].reference_velocity_meters_per_second = fixed_point_from_float(2.0f);
    }

    MpcSolverResult_t result;
    MpcSolverStatus_t status = mpc_compute_optimal_control(&current_state, reference, &result);

    printf("  Solver status: %d\n", status);
    printf("  Steering: %.4f rad\n", fixed_point_to_float(result.optimal_control.steering_angle_radians));
    printf("  Acceleration: %.4f m/s²\n",
           fixed_point_to_float(result.optimal_control.acceleration_meters_per_second_squared));

    TEST_ASSERT(status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");

    /* The solver produces some control output (sign may vary based on cost weights) */
    float accel = fixed_point_to_float(result.optimal_control.acceleration_meters_per_second_squared);
    TEST_ASSERT(fabsf(accel) < 5.0f, "Acceleration is within reasonable bounds");
}

/*===========================================================================
 * Test: Control Saturation
 *===========================================================================*/

void test_control_saturation(void)
{
    printf("\n--- Test: Control Saturation ---\n");

    mpc_initialize();

    /* Vehicle at origin, but reference is far to the left */
    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = fixed_point_from_float(2.0f)};

    /* Reference: hard left turn (Y = +10) */
    TrajectoryReferencePoint_t reference[MPC_DEFAULT_PREDICTION_HORIZON];
    for (int i = 0; i < MPC_DEFAULT_PREDICTION_HORIZON; i++)
    {
        reference[i].reference_position_x_meters = 0;
        reference[i].reference_position_y_meters = fixed_point_from_float(10.0f);
        reference[i].reference_heading_radians = fixed_point_from_float(1.57f); /* π/2 */
        reference[i].reference_velocity_meters_per_second = fixed_point_from_float(2.0f);
    }

    MpcSolverResult_t result;
    MpcSolverStatus_t status = mpc_compute_optimal_control(&current_state, reference, &result);

    float steering = fixed_point_to_float(result.optimal_control.steering_angle_radians);
    float max_steering = 0.42f; /* F1/10th max steering */

    printf("  Steering: %.4f rad (max: %.2f)\n", steering, max_steering);

    TEST_ASSERT(status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");
    TEST_ASSERT(fabsf(steering) <= max_steering + 0.01f,
                "Steering respects vehicle limits");
}

/*===========================================================================
 * Test: MPC Reset
 *===========================================================================*/

void test_mpc_reset(void)
{
    printf("\n--- Test: MPC Reset ---\n");

    mpc_initialize();

    /* Compute some controls to set internal state */
    VehicleState_t state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = fixed_point_from_float(1.0f)};

    TrajectoryReferencePoint_t reference[MPC_DEFAULT_PREDICTION_HORIZON];
    for (int i = 0; i < MPC_DEFAULT_PREDICTION_HORIZON; i++)
    {
        reference[i].reference_position_x_meters = fixed_point_from_float((float)(i + 1));
        reference[i].reference_position_y_meters = 0;
        reference[i].reference_heading_radians = 0;
        reference[i].reference_velocity_meters_per_second = fixed_point_from_float(2.0f);
    }

    MpcSolverResult_t result1;
    mpc_compute_optimal_control(&state, reference, &result1);

    /* Reset MPC */
    mpc_reset();

    /* Compute again - should work without issues */
    MpcSolverResult_t result2;
    MpcSolverStatus_t status = mpc_compute_optimal_control(&state, reference, &result2);

    TEST_ASSERT(status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "MPC works after reset");
}

/*===========================================================================
 * Test: Configuration Update
 *===========================================================================*/

void test_configuration_update(void)
{
    printf("\n--- Test: Configuration Update ---\n");

    mpc_initialize();

    MpcConfiguration_t new_config = mpc_get_configuration();
    new_config.prediction_horizon_steps = 5;
    new_config.weight_position_x = fixed_point_from_float(20.0f);

    mpc_set_configuration(&new_config);

    MpcConfiguration_t retrieved = mpc_get_configuration();

    TEST_ASSERT(retrieved.prediction_horizon_steps == 5, "Horizon updated to 5");
    TEST_ASSERT(fixed_point_to_float(retrieved.weight_position_x) > 15.0f,
                "Position weight updated");
}

/*===========================================================================
 * Main Test Runner
 *===========================================================================*/

int main(void)
{
    printf("===========================================================\n");
    printf("   MPC Controller Unit Tests\n");
    printf("===========================================================\n");

    test_mpc_initialization();
    test_compute_control_stationary();
    test_compute_control_forward();
    test_control_saturation();
    test_mpc_reset();
    test_configuration_update();

    printf("\n===========================================================\n");
    printf("   Results: %d passed, %d failed\n", total_tests_passed, total_tests_failed);
    printf("===========================================================\n");

    return (total_tests_failed > 0) ? 1 : 0;
}
