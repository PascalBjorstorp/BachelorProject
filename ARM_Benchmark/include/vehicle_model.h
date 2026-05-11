/**
 * @file vehicle_model.h
 * @brief Standalone CPU vehicle model used by the ARM benchmark project.
 */

#ifndef ARM_BENCHMARK_VEHICLE_MODEL_H
#define ARM_BENCHMARK_VEHICLE_MODEL_H

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NX_FRENET 5

#define TIME_STEP_SECONDS 0.03f

#define VP_MAX_STEERING_RAD 0.39f
#define VP_MAX_VELOCITY_MPS 20.0f
#define VP_MIN_VELOCITY_MPS 0.5f
#define VP_CG_TO_FRONT_AXLE_M 0.166f
#define VP_CG_TO_REAR_AXLE_M 0.16f
#define VP_WHEELBASE_M (VP_CG_TO_FRONT_AXLE_M + VP_CG_TO_REAR_AXLE_M)
#define VP_MASS_KG 3.314f
#define VP_YAW_INERTIA_KGM2 0.035f
#define VP_CG_HEIGHT_M 0.0703f
#define VP_FRICTION_COEFF 0.72f
#define GRAVITY_MPS2 9.81f
#define VP_MASS_TIMES_GRAVITY_N (VP_MASS_KG * GRAVITY_MPS2)
#define VP_INV_MASS_1_PER_KG (1.0f / VP_MASS_KG)
#define VP_INV_YAW_INERTIA_1_PER_KGM2 (1.0f / VP_YAW_INERTIA_KGM2)
#define VP_MAX_ACCEL_MPS2 (VP_FRICTION_COEFF * GRAVITY_MPS2)
#define VP_MIN_ACCEL_MPS2 (-VP_MAX_ACCEL_MPS2)
#define VP_C_ALPHA_F 51.40f
#define VP_C_ALPHA_R 43.10f
#define VP_NORM_LOAD_F (VP_MASS_TIMES_GRAVITY_N * VP_CG_TO_REAR_AXLE_M / VP_WHEELBASE_M)
#define VP_NORM_LOAD_R (VP_MASS_TIMES_GRAVITY_N * VP_CG_TO_FRONT_AXLE_M / VP_WHEELBASE_M)
#define VP_D_FRONT (VP_FRICTION_COEFF * VP_NORM_LOAD_F)
#define VP_D_REAR (VP_FRICTION_COEFF * VP_NORM_LOAD_R)
#define VP_C_SHAPE 1.9f
#define VP_INV_C_SHAPE (1.0f / VP_C_SHAPE)
#define MIN_STIFF_SCALE 0.1f
#define VP_FRONT_CORNERING_STIFFNESS (VP_C_ALPHA_F / VP_D_FRONT)
#define VP_REAR_CORNERING_STIFFNESS (VP_C_ALPHA_R / VP_D_REAR)
#define MIN_SLIP_VELOCITY 0.5f

typedef struct {
    float pos_x;
    float pos_y;
    float heading;
    float long_vel;
    float lat_vel;
    float yaw_rate;
} VehicleState_t;

typedef struct {
    float flat_error;
    float fhead_error;
    float flong_vel;
    float flat_vel;
    float fyaw_rate;
} FrenetState_t;

typedef struct {
    float steer_ang;
    float long_acc;
} ControlInput_t;

typedef struct {
    float vx_safe;
    float inv_vx_safe;
    float front_num;
    float rear_num;
    float front_ratio;
    float rear_ratio;
    float alpha_front;
    float alpha_rear;
} SlipTerms_t;

static inline float util_div(float a, float b)
{
    return (b != 0.0f) ? a / b : 0.0f;
}

static inline float util_clamp(float val, float lo, float hi)
{
    return fminf(fmaxf(val, lo), hi);
}

static inline float util_normalize_angle(float angle)
{
    angle = fmodf(angle + (float)M_PI, 2.0f * (float)M_PI);
    if (angle < 0.0f) {
        angle += 2.0f * (float)M_PI;
    }
    return angle - (float)M_PI;
}

static inline float util_recip(float x)
{
    return (x != 0.0f) ? 1.0f / x : 0.0f;
}

void vehicle_model_compute_slip_terms(
    float vx,
    float vy,
    float omega,
    float delta,
    SlipTerms_t *slip_terms);

void vehicle_model_compute_normal_loads(
    float longitudinal_force,
    float *front_normal_load,
    float *rear_normal_load);

void vehicle_model_compute_effective_lateral_stiffness(
    uint8_t use_front_axle,
    float normal_load,
    float slip_angle,
    float *effective_stiffness,
    float *lateral_force);

ControlInput_t vehicle_model_saturate_control(const ControlInput_t *raw_control);

VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    float time_step);

void vehicle_model_predict_trajectory(
    const VehicleState_t *initial_state,
    const ControlInput_t *control_sequence,
    float time_step,
    uint16_t step_count,
    VehicleState_t *predicted_trajectory);

void vehicle_model_compute_frenet_linearization(
    const FrenetState_t *frenet_state,
    const ControlInput_t *operating_control,
    float time_step,
    float path_curvature,
    float reference_velocity,
    float state_matrix_A[NX_FRENET][NX_FRENET],
    float input_matrix_B[NX_FRENET][2]);

#endif
