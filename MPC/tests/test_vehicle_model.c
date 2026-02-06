/**
 * @file test_vehicle_model.c
 * @brief Unit Tests for Kinematic Bicycle Model
 *
 * Tests vehicle model predictions for correctness.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vehicle_model.h"
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
 * Test: Model Initialization
 *===========================================================================*/

void test_model_initialization(void)
{
    printf("\n--- Test: Model Initialization ---\n");

    vehicle_model_initialize();
    VehicleParameters_t parameters = vehicle_model_get_parameters();

    float wheelbase = (parameters.wheelbase_meters);
    float max_steering = (parameters.maximum_steering_angle_radians);
    float max_velocity = (parameters.maximum_velocity_meters_per_second);

    printf("  Wheelbase: %.3f m\n", wheelbase);
    printf("  Max steering: %.3f rad (%.1f°)\n", max_steering, max_steering * 180.0f / 3.14159f);
    printf("  Max velocity: %.1f m/s\n", max_velocity);

    TEST_ASSERT(values_are_approximately_equal(wheelbase, 0.32f, 0.01f),
                "Wheelbase = 0.32 m (F1/10th default)");
    TEST_ASSERT(values_are_approximately_equal(max_steering, 0.42f, 0.02f),
                "Max steering ≈ 0.42 rad (24°)");
    TEST_ASSERT(values_are_approximately_equal(max_velocity, 6.0f, 0.1f),
                "Max velocity = 6.0 m/s");
}

/*===========================================================================
 * Test: Straight Line Motion
 *===========================================================================*/

void test_straight_line_motion(void)
{
    printf("\n--- Test: Straight Line Motion ---\n");

    vehicle_model_initialize();

    /* Initial state: at origin, heading along +X axis, velocity 1 m/s */
    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = (1.0f)};

    /* Control: no steering, no acceleration */
    ControlInput_t control_input = {
        .steering_angle_radians = 0,
        .acceleration_meters_per_second_squared = 0};

    /* Simulate 1 second with 0.1s time steps (10 steps) */
    fixed_point_t time_step = (0.1f);

    for (int step = 0; step < 10; step++)
    {
        current_state = vehicle_model_predict_next_state(&current_state, &control_input, time_step);
    }

    float final_x = (current_state.position_x_meters);
    float final_y = (current_state.position_y_meters);
    float final_heading = (current_state.heading_angle_radians);

    printf("  After 1s at 1 m/s:\n");
    printf("    Position: (%.3f, %.3f) m\n", final_x, final_y);
    printf("    Heading: %.3f rad\n", final_heading);

    TEST_ASSERT(values_are_approximately_equal(final_x, 1.0f, 0.1f),
                "X position ≈ 1.0 m (traveled 1 m/s × 1 s)");
    TEST_ASSERT(values_are_approximately_equal(final_y, 0.0f, 0.05f),
                "Y position ≈ 0 m (straight line)");
    TEST_ASSERT(values_are_approximately_equal(final_heading, 0.0f, 0.01f),
                "Heading unchanged (no steering)");
}

/*===========================================================================
 * Test: Turning Left
 *===========================================================================*/

void test_turning_left(void)
{
    printf("\n--- Test: Turning Left ---\n");

    vehicle_model_initialize();

    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = (2.0f)};

    /* Positive steering angle = turn left (counterclockwise) */
    ControlInput_t control_input = {
        .steering_angle_radians = (0.3f), /* ~17° */
        .acceleration_meters_per_second_squared = 0};

    fixed_point_t time_step = (0.1f);

    for (int step = 0; step < 10; step++)
    {
        current_state = vehicle_model_predict_next_state(&current_state, &control_input, time_step);
    }

    float final_heading = (current_state.heading_angle_radians);
    float final_y = (current_state.position_y_meters);

    printf("  After 1s turning left:\n");
    printf("    Heading: %.3f rad (%.1f°)\n", final_heading, final_heading * 180.0f / 3.14159f);
    printf("    Y position: %.3f m\n", final_y);

    TEST_ASSERT(final_heading > 0.1f, "Heading increased (turned left)");
    TEST_ASSERT(final_y > 0.1f, "Moved in +Y direction (left turn)");
}

/*===========================================================================
 * Test: Acceleration
 *===========================================================================*/

void test_acceleration(void)
{
    printf("\n--- Test: Acceleration ---\n");

    vehicle_model_initialize();

    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = (1.0f) /* Start at 1 m/s */
    };

    /* Accelerate at 2 m/s² */
    ControlInput_t control_input = {
        .steering_angle_radians = 0,
        .acceleration_meters_per_second_squared = (2.0f)};

    fixed_point_t time_step = (0.1f);

    /* After 1 second: v = v0 + a×t = 1 + 2×1 = 3 m/s */
    for (int step = 0; step < 10; step++)
    {
        current_state = vehicle_model_predict_next_state(&current_state, &control_input, time_step);
    }

    float final_velocity = (current_state.velocity_meters_per_second);

    printf("  After 1s accelerating at 2 m/s²:\n");
    printf("    Velocity: %.3f m/s (expected 3.0)\n", final_velocity);

    TEST_ASSERT(values_are_approximately_equal(final_velocity, 3.0f, 0.2f),
                "Velocity ≈ 3.0 m/s (1 + 2×1)");
}

/*===========================================================================
 * Test: Control Saturation
 *===========================================================================*/

void test_control_saturation(void)
{
    printf("\n--- Test: Control Saturation ---\n");

    vehicle_model_initialize();
    VehicleParameters_t parameters = vehicle_model_get_parameters();

    float max_steering = (parameters.maximum_steering_angle_radians);
    float max_accel = (parameters.maximum_acceleration_meters_per_second_squared);

    /* Try to exceed limits */
    ControlInput_t excessive_control = {
        .steering_angle_radians = (1.0f),                 /* Way over limit */
        .acceleration_meters_per_second_squared = (10.0f) /* Way over limit */
    };

    ControlInput_t saturated = vehicle_model_saturate_control(&excessive_control);

    float saturated_steering = (saturated.steering_angle_radians);
    float saturated_accel = (saturated.acceleration_meters_per_second_squared);

    printf("  Input: steering=1.0 rad, accel=10.0 m/s²\n");
    printf("  Saturated: steering=%.3f rad, accel=%.1f m/s²\n",
           saturated_steering, saturated_accel);

    TEST_ASSERT(values_are_approximately_equal(saturated_steering, max_steering, 0.01f),
                "Steering clamped to maximum");
    TEST_ASSERT(values_are_approximately_equal(saturated_accel, max_accel, 0.1f),
                "Acceleration clamped to maximum");
}

/*===========================================================================
 * Test: Trajectory Prediction
 *===========================================================================*/

void test_trajectory_prediction(void)
{
    printf("\n--- Test: Trajectory Prediction ---\n");

    vehicle_model_initialize();

    VehicleState_t initial_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = 0,
        .velocity_meters_per_second = (1.0f)};

    /* Constant control for 5 steps */
    ControlInput_t control_sequence[5];
    for (int i = 0; i < 5; i++)
    {
        control_sequence[i].steering_angle_radians = 0;
        control_sequence[i].acceleration_meters_per_second_squared = 0;
    }

    VehicleState_t predicted_trajectory[6]; /* 5 steps + initial */
    fixed_point_t time_step = (0.1f);

    vehicle_model_predict_trajectory(&initial_state, control_sequence, time_step, 5, predicted_trajectory);

    printf("  Trajectory X positions: ");
    for (int i = 0; i <= 5; i++)
    {
        printf("%.2f ", (predicted_trajectory[i].position_x_meters));
    }
    printf("\n");

    /* First state should match initial */
    TEST_ASSERT(predicted_trajectory[0].position_x_meters == initial_state.position_x_meters,
                "First trajectory state matches initial");

    /* X positions should be monotonically increasing */
    int positions_increasing = 1;
    for (int i = 1; i <= 5; i++)
    {
        if (predicted_trajectory[i].position_x_meters <= predicted_trajectory[i - 1].position_x_meters)
        {
            positions_increasing = 0;
            break;
        }
    }
    TEST_ASSERT(positions_increasing, "X positions monotonically increasing");
}

/*===========================================================================
 * Test: Heading Normalization
 *===========================================================================*/

void test_heading_normalization(void)
{
    printf("\n--- Test: Heading Normalization ---\n");

    vehicle_model_initialize();

    /* Start with heading close to π */
    VehicleState_t current_state = {
        .position_x_meters = 0,
        .position_y_meters = 0,
        .heading_angle_radians = (3.0f), /* Close to π */
        .velocity_meters_per_second = (1.0f)};

    /* Turn more to exceed π */
    ControlInput_t control_input = {
        .steering_angle_radians = (0.4f), /* Strong left turn */
        .acceleration_meters_per_second_squared = 0};

    fixed_point_t time_step = (0.1f);

    for (int step = 0; step < 20; step++)
    {
        current_state = vehicle_model_predict_next_state(&current_state, &control_input, time_step);
    }

    float final_heading = (current_state.heading_angle_radians);

    printf("  Final heading: %.3f rad\n", final_heading);

    TEST_ASSERT(final_heading >= -3.15f && final_heading <= 3.15f,
                "Heading normalized to [-π, +π]");
}

/*===========================================================================
 * Main Test Runner
 *===========================================================================*/

int main(void)
{
    printf("===========================================================\n");
    printf("   Vehicle Model Unit Tests\n");
    printf("===========================================================\n");

    test_model_initialization();
    test_straight_line_motion();
    test_turning_left();
    test_acceleration();
    test_control_saturation();
    test_trajectory_prediction();
    test_heading_normalization();

    printf("\n===========================================================\n");
    printf("   Results: %d passed, %d failed\n", total_tests_passed, total_tests_failed);
    printf("===========================================================\n");

    return (total_tests_failed > 0) ? 1 : 0;
}
