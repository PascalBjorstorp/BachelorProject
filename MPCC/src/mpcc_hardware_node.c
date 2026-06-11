#define _POSIX_C_SOURCE 199309L

/*******************************************************************************
 * mpcc_hardware_node.c -- ROS2 Node for Lifted ODE MPCC on Real Hardware
 *
 * Frenet primary (s, n, alpha) + Cartesian redundant (X, Y, psi) formulation.
 * Subscribes to hardware odometry/localization and publishes Ackermann commands.
 *
 * State [9]: s, n, alpha, vx, vy, omega, X, Y, psi
 * Control [3]: delta, a_x, v_theta
 ******************************************************************************/

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <std_msgs/msg/float64.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <rcutils/allocator.h>

#include "mpcc_types.h"
#include "mpcc.h"
#include "mpcc_vehicle_model.h"

/* -------------------------------------------------------------------------- */
/* Runtime Configuration                                                       */
/* -------------------------------------------------------------------------- */

#define MPCC_DEFAULT_CONTROL_PERIOD_MS 50U
#define MPCC_EXECUTOR_HANDLES 6
#define MPCC_ODOM_LOG_THROTTLE_SEC 1.0
#define MPCC_ODOM_MAX_ABS_VX_MPS 20.0
#define MPCC_ODOM_MAX_ABS_VY_MPS 20.0
#define MPCC_ODOM_MAX_ABS_YAW_RATE_RADPS 20.0
#define MPCC_STALE_SOLVE_GAP_RESET_FACTOR 4.0
#define MPCC_STALE_SOLVE_GAP_MIN_SEC 0.10

static const char *g_odom_topic = "/ego_racecar/odom";
static const char *g_pose_topic = "/ekf_pose";
static const char *g_imu_topic = "/imu/filtered_angular_velocity";
static const char *g_servo_topic = "/sensors/servo_position_command";
static const char *g_drive_topic = "/drive";

static const char *g_trajectory_file = NULL;

static int g_verbose = 0;

/* Solver-derived values used to map MPCC acceleration to a velocity command. */
static double g_solver_dt_sec = MPCC_DEFAULT_DT;
static double g_vx_max_mps = 8.0;
static double g_vx_min_cmd = 0.1;  /* Minimum velocity command [m/s] */

/* Hardware safety: clamp acceleration to the measured braking capability. */
static double g_ax_min_hardware = -6.0;
static double g_delta_rate_limit = MPCC_DEFAULT_DELTA_RATE_MAX;

/* Odometry watchdog and twist sanity thresholds for real-car inputs. */
static double g_watchdog_timeout_sec = 0.2;

/* Nominal control cadence used for rate penalties when not adapting online. */
static double g_nominal_control_dt_sec = 1.0 / MPCC_CONTROL_RATE_HZ;
static int g_adapt_cross_call_scale = 1;
static int g_control_period_explicit = 0;
static int g_adapt_nominal_dt_bootstrapped = 0;
static int g_cross_call_upper_clamp_logged = 0;
static int g_cross_call_motion_gate_logged = 0;

/* -------------------------------------------------------------------------- */
/* VESC Servo Conversion Parameters                                            */
/* -------------------------------------------------------------------------- */
/* Forward (angle -> servo): servo_val = gain * corrected + offset
 * where corrected = c2*|d|^2 + c1*|d| + c0 (sign-preserved)
 * Inverse: solve quadratic to recover d from servo_val */
#define STEERING_TO_SERVO_GAIN   (-0.7284)
#define STEERING_TO_SERVO_OFFSET 0.55
#define STEERING_CORRECTION_C2   0.589566
#define STEERING_CORRECTION_C1   0.918061
#define STEERING_CORRECTION_C0   0.001490
#define STEERING_RATE_LIMIT      2.849

/* -------------------------------------------------------------------------- */
/* Global State                                                                */
/* -------------------------------------------------------------------------- */

static VehicleState_t g_vehicle_state;
static float g_current_s = 0;
static MPCCReferencePath_t g_reference_path;

static int g_have_reference = 0;
static int g_have_odom = 0;
static int g_have_pose = 0;
static int g_using_map_pose = 0;

static double g_latest_vx_mps = 0.0;
static double g_latest_vy_mps = 0.0;
static double g_latest_omega = 0.0;
static struct timespec g_last_odom_time = {0, 0};
static struct timespec g_last_imu_time  = {0, 0};
static struct timespec g_last_odom_watchdog_log_time = {0, 0};
static struct timespec g_last_odom_sanity_log_time = {0, 0};
static uint32_t g_solve_count = 0;
static uint32_t g_odom_update_count = 0;
static uint32_t g_last_solve_odom_update_count = 0;
static int g_odom_watchdog_active = 0;

/* Measured solve cadence used for control gating and cross-call scaling. */
static struct timespec g_prev_solve_time = {0, 0};
static double g_control_dt_filtered = 0.005;  /* Initial guess: 200 Hz */

/* Servo feedback tracking */
static double g_actual_steering_angle = 0.0;
static int g_use_steering_feedback = 0;

/* Previous control command (kept on solver failure) */
static float g_prev_delta_cmd = 0.0f;
static float g_prev_speed_cmd = 0.0f;
static float g_prev_ax_cmd = 0.0f;
static int g_have_published_drive_cmd = 0;
static int g_publish_speed_command = 0;
static int g_use_local_raceline = 0;
static int g_raceline_sub_ok = 0;

/* ROS entities */
static rcl_subscription_t g_odom_sub;
static rcl_subscription_t g_pose_sub;
static rcl_subscription_t g_imu_sub;
static rcl_subscription_t g_servo_sub;

static rcl_publisher_t g_drive_pub;
static rcl_publisher_t g_predicted_path_pub;
static rcl_publisher_t g_raceline_pub;

/* Message buffers */
static nav_msgs__msg__Odometry g_odom_msg;
static geometry_msgs__msg__PoseWithCovarianceStamped g_pose_msg;
static std_msgs__msg__Float64 g_imu_msg;
static std_msgs__msg__Float64 g_servo_msg;
static ackermann_msgs__msg__AckermannDriveStamped g_drive_msg;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static double quat_to_yaw(double qx, double qy, double qz, double qw)
{
    const double siny = 2.0 * (qw * qz + qx * qy);
    const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    return atan2(siny, cosy);
}

static double timespec_diff_sec(const struct timespec *start,
                                const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec)
         + (double)(end->tv_nsec - start->tv_nsec) * 1e-9;
}

static int timespec_is_set(const struct timespec *stamp)
{
    return stamp->tv_sec != 0 || stamp->tv_nsec != 0;
}

static void log_path_alignment_debug(uint32_t solve_count, float s_state)
{
    if (!g_have_reference || g_reference_path.num_points < 2)
    {
        return;
    }

    const float s_closest = mpcc_find_closest_s(
        &g_reference_path,
        g_vehicle_state.pos_x,
        g_vehicle_state.pos_y);

    MPCCPathPoint_t path_pt;
    mpcc_path_interpolate(&g_reference_path, s_closest, &path_pt);

    const float dX = g_vehicle_state.pos_x - path_pt.x_ref;
    const float dY = g_vehicle_state.pos_y - path_pt.y_ref;
    const float sin_phi = sinf(path_pt.phi_ref);
    const float cos_phi = cosf(path_pt.phi_ref);
    const float e_c = (sin_phi * dX) - (cos_phi * dY);
    const float e_l = -((cos_phi * dX) + (sin_phi * dY));
    float dpsi = g_vehicle_state.heading - path_pt.phi_ref;
    const float dist = sqrtf((dX * dX) + (dY * dY));
    const float left_slack = path_pt.left_bound - e_c;
    const float right_slack = path_pt.right_bound + e_c;

    while (dpsi > (float)M_PI) dpsi -= 2.0f * (float)M_PI;
    while (dpsi < -(float)M_PI) dpsi += 2.0f * (float)M_PI;

    fprintf(stderr,
            "[MPCC %3u] path debug: s_state=%.2f s_closest=%.2f dist=%.3f ec=%.3f el=%.3f dpsi=%.3f bounds(L/R)=%.3f/%.3f slack(L/R)=%.3f/%.3f ref=(%.2f,%.2f) pose=(%.2f,%.2f)\n",
            solve_count,
            s_state,
            s_closest,
            dist,
            e_c,
            e_l,
            dpsi,
            path_pt.left_bound,
            path_pt.right_bound,
            left_slack,
            right_slack,
            path_pt.x_ref,
            path_pt.y_ref,
            g_vehicle_state.pos_x,
            g_vehicle_state.pos_y);
}

static void log_solve_metrics(uint32_t solve_count,
                              MPCCStatus_t status,
                              const MPCCState_t *state,
                              const MPCCResult_t *result,
                              float delta_cmd,
                              float a_x_cmd,
                              float v_theta_cmd,
                              float v_cmd,
                              double solve_gap_sec,
                              double target_dt_sec)
{
    const double solve_gap_ms = (solve_gap_sec > 0.0) ? (solve_gap_sec * 1000.0) : 0.0;
    const double solve_rate_hz = (solve_gap_sec > 1e-9) ? (1.0 / solve_gap_sec) : 0.0;

    fprintf(stderr,
            "[MPCC] solve=%u status=%d iter=%u prim=%.4f dual=%.4f rho=%.3f rho_u=%.3f rho_upd=%u clip=%u rho_x_upd=%u rho_u_upd=%u diag=0x%X hmin=%.2e tw=%.3f dw=%.3f axlim=%.3f vxm=%.3f s=%.2f x=%.2f y=%.2f psi=%.2f vx=%.2f delta=%.4f a_x=%.3f v_theta=%.3f v_cmd=%.2f solve_gap_ms=%.1f solve_rate_hz=%.2f target_ms=%.1f\n",
            solve_count,
            (int)status,
            result->admm_iterations,
            result->primal_residual,
            result->dual_residual,
            result->rho_final,
            result->rho_u_final,
            (unsigned)result->adaptive_rho_updates,
            (unsigned)result->numeric_clip_count,
            (unsigned)result->adaptive_rho_state_updates,
            (unsigned)result->adaptive_rho_control_updates,
            (unsigned)result->qp_diagnostic_flags,
            result->qp_min_hessian_eigenvalue,
            result->qp_min_track_width,
            result->qp_min_delta_width,
            result->qp_min_ax_limit,
            result->qp_min_vx_margin,
            state->s,
            state->X,
            state->Y,
            state->psi,
            state->vx,
            delta_cmd,
            a_x_cmd,
            v_theta_cmd,
            v_cmd,
            solve_gap_ms,
            solve_rate_hz,
            target_dt_sec * 1000.0);
    fflush(stderr);
}

static int should_log_throttled(struct timespec *last_log_time,
                                const struct timespec *now,
                                double interval_sec)
{
    if (!timespec_is_set(last_log_time)
        || timespec_diff_sec(last_log_time, now) >= interval_sec)
    {
        *last_log_time = *now;
        return 1;
    }

    return 0;
}

static int odom_twist_is_sane(double vx,
                              double vy,
                              double omega,
                              const struct timespec *now)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(omega))
    {
        if (should_log_throttled(&g_last_odom_sanity_log_time,
                                 now,
                                 MPCC_ODOM_LOG_THROTTLE_SEC))
        {
            fprintf(stderr,
                    "[MPCC] WARNING: rejecting odometry twist with NaN/Inf "
                    "(vx=%.3f vy=%.3f w=%.3f)\n",
                    vx,
                    vy,
                    omega);
        }
        return 0;
    }

    if (fabs(vx) > MPCC_ODOM_MAX_ABS_VX_MPS
        || fabs(vy) > MPCC_ODOM_MAX_ABS_VY_MPS
        || fabs(omega) > MPCC_ODOM_MAX_ABS_YAW_RATE_RADPS)
    {
        if (should_log_throttled(&g_last_odom_sanity_log_time,
                                 now,
                                 MPCC_ODOM_LOG_THROTTLE_SEC))
        {
            fprintf(stderr,
                    "[MPCC] WARNING: rejecting odometry twist outside sanity "
                    "limits (vx=%.3f vy=%.3f w=%.3f)\n",
                    vx,
                    vy,
                    omega);
        }
        return 0;
    }

    return 1;
}

static double current_control_dt_sec(void)
{
    if (g_adapt_cross_call_scale && g_control_dt_filtered > 0.0)
    {
        return g_control_dt_filtered;
    }
    if (g_nominal_control_dt_sec > 0.0)
    {
        return g_nominal_control_dt_sec;
    }
    if (g_solver_dt_sec > 0.0)
    {
        return g_solver_dt_sec;
    }
    return 0.005;
}

static void reset_mpcc_after_stale_solve_gap(double solve_gap_sec)
{
    double target_dt_sec = g_nominal_control_dt_sec;
    double stale_gap_sec;

    if (!isfinite(target_dt_sec) || target_dt_sec <= 0.0)
    {
        target_dt_sec = current_control_dt_sec();
    }

    stale_gap_sec = MPCC_STALE_SOLVE_GAP_RESET_FACTOR * target_dt_sec;
    if (stale_gap_sec < MPCC_STALE_SOLVE_GAP_MIN_SEC)
    {
        stale_gap_sec = MPCC_STALE_SOLVE_GAP_MIN_SEC;
    }

    if (!isfinite(solve_gap_sec) || solve_gap_sec <= stale_gap_sec)
    {
        return;
    }

    if (g_have_reference && g_reference_path.num_points >= 2)
    {
        g_current_s = mpcc_find_closest_s(
            &g_reference_path,
            g_vehicle_state.pos_x,
            g_vehicle_state.pos_y);
    }

    mpcc_reset();

    fprintf(stderr,
            "[MPCC] WARNING: solve gap %.1f ms exceeded stale threshold %.1f ms; "
            "reset warm-start and re-anchored s=%.2f\n",
            solve_gap_sec * 1000.0,
            stale_gap_sec * 1000.0,
            g_current_s);
}

static float limit_steering_delta(float requested_delta, double control_dt_sec)
{
    float limited_delta = requested_delta;

    if (limited_delta > F110_DEFAULT_MAXIMUM_STEERING_RADIANS)
        limited_delta = F110_DEFAULT_MAXIMUM_STEERING_RADIANS;
    if (limited_delta < -F110_DEFAULT_MAXIMUM_STEERING_RADIANS)
        limited_delta = -F110_DEFAULT_MAXIMUM_STEERING_RADIANS;

    if (control_dt_sec > 0.0)
    {
        const float max_delta_change = (float)(g_delta_rate_limit * control_dt_sec);
        float delta_change = limited_delta - g_prev_delta_cmd;

        if (delta_change > max_delta_change)
            limited_delta = g_prev_delta_cmd + max_delta_change;
        else if (delta_change < -max_delta_change)
            limited_delta = g_prev_delta_cmd - max_delta_change;
    }

    return limited_delta;
}

static void republish_last_drive_command(const char *reason)
{
    if (!g_have_published_drive_cmd)
    {
        return;
    }

    if (g_verbose)
    {
        printf("[MPCC] Republishing last drive command (%s)\n", reason);
    }

    {
        const rcl_ret_t rc = rcl_publish(&g_drive_pub, &g_drive_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr,
                    "[MPCC] WARNING: failed to republish drive command (%s): %s\n",
                    reason,
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
}

static void align_first_prediction_step(
    MPCCResult_t *result,
    const MPCCState_t *current_state,
    float delta_cmd,
    float a_x_cmd,
    float v_theta_cmd,
    float v_cmd)
{
    MPCCConfiguration_t cfg = mpcc_get_configuration();
    MPCCControl_t applied_control;

    applied_control.delta = delta_cmd;
    applied_control.a_x = a_x_cmd;
    applied_control.v_theta = v_theta_cmd;

    result->predicted_states[0] = *current_state;
    result->predicted_controls[0] = applied_control;

    if (cfg.horizon_steps > 0)
    {
        MPCCLinearSystem_t dyn;
        float x_arr[MPCC_NX] = {
            current_state->s,
            current_state->vx,
            current_state->vy,
            current_state->omega,
            current_state->X,
            current_state->Y,
            current_state->psi
        };
        float u_arr[MPCC_NU] = {
            delta_cmd,
            a_x_cmd,
            v_theta_cmd
        };
        float x_next[MPCC_NX];

        mpcc_linearize_dynamics(current_state, &applied_control, cfg.dt, &cfg, &dyn);

        for (int i = 0; i < MPCC_NX; ++i)
        {
            x_next[i] = dyn.d[i];
            for (int j = 0; j < MPCC_NX; ++j)
                x_next[i] += dyn.A[i][j] * x_arr[j];
            for (int j = 0; j < MPCC_NU; ++j)
                x_next[i] += dyn.B[i][j] * u_arr[j];
        }

        x_next[MPCC_IDX_VX] = v_cmd;

        result->predicted_states[1].s = x_next[MPCC_IDX_S];
        result->predicted_states[1].vx = x_next[MPCC_IDX_VX];
        result->predicted_states[1].vy = x_next[MPCC_IDX_VY];
        result->predicted_states[1].omega = x_next[MPCC_IDX_OMEGA];
        result->predicted_states[1].X = x_next[MPCC_IDX_X];
        result->predicted_states[1].Y = x_next[MPCC_IDX_Y];
        result->predicted_states[1].psi = x_next[MPCC_IDX_PSI];
    }
}

/* -------------------------------------------------------------------------- */
/* Trajectory Loading                                                          */
/* -------------------------------------------------------------------------- */

static int load_trajectory_csv(const char *file_path, MPCCReferencePath_t *path)
{
    FILE *file = fopen(file_path, "r");
    if (file == NULL)
    {
        fprintf(stderr, "[MPCC] ERROR: cannot open trajectory file: %s\n", file_path);
        return 0;
    }

    char line[512];
    path->num_points = 0;

    while (fgets(line, sizeof(line), file) != NULL)
    {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
        {
            continue;
        }

        if (path->num_points >= MPCC_MAX_PATH_POINTS)
        {
            printf("[MPCC] WARNING: trajectory truncated at %d points\n",
                   MPCC_MAX_PATH_POINTS);
            break;
        }

        double s, x, y, psi, kappa, vx, ax;
        double d_left = 0.5;
        double d_right = 0.5;

        const int parsed = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                  &s, &x, &y, &psi, &kappa, &vx, &ax,
                                  &d_left, &d_right);

        if (parsed < 5)
        {
            continue;
        }

        MPCCPathPoint_t *pt = &path->points[path->num_points];
        pt->s_ref = (float)s;
        pt->x_ref = (float)x;
        pt->y_ref = (float)y;
        pt->phi_ref = (float)psi;
        pt->kappa_ref = (float)kappa;
        pt->vx_ref = (parsed >= 6) ? (float)vx : 3.0f;

        {
            const float car_half_width = 0.155f;
            if (parsed >= 9)
            {
                float left_bound = (float)d_left - car_half_width;
                float right_bound = (float)d_right - car_half_width;
                if ((left_bound + right_bound) < 0.0f)
                {
                    float lower = -left_bound;
                    float upper = right_bound;
                    float center = 0.5f * (lower + upper);
                    left_bound = -center;
                    right_bound = center;
                }
                pt->left_bound = left_bound;
                pt->right_bound = right_bound;
            }
            else
            {
                pt->left_bound = 0.5f;
                pt->right_bound = 0.5f;
            }
        }

        (void)ax;
        path->num_points++;
    }

    fclose(file);

    if (path->num_points < 2)
    {
        fprintf(stderr, "[MPCC] ERROR: only %d points loaded from %s\n",
                path->num_points, file_path);
        return 0;
    }

    path->total_length = path->points[path->num_points - 1].s_ref;
    path->is_closed = 1;

    {
        float min_corridor = path->points[0].left_bound + path->points[0].right_bound;
        for (uint16_t i = 1; i < path->num_points; ++i)
        {
            const float corridor = path->points[i].left_bound + path->points[i].right_bound;
            if (corridor < min_corridor)
            {
                min_corridor = corridor;
            }
        }

        if (min_corridor < 0.35f)
        {
            fprintf(stderr,
                    "[MPCC] WARNING: very tight CSV corridor detected (min width %.2f m)\n",
                    min_corridor);
        }
    }

    printf("[MPCC] Loaded %d points from %s (track length %.1f m)\n",
           path->num_points, file_path, path->total_length);

    return 1;
}

static void publish_raceline(const MPCCReferencePath_t *path)
{
    nav_msgs__msg__Path path_msg;
    nav_msgs__msg__Path__init(&path_msg);

    rcutils_allocator_t alloc = rcutils_get_default_allocator();

    char *frame = (char *)alloc.allocate(8, alloc.state);
    if (frame != NULL)
    {
        memcpy(frame, "map", 4);
        path_msg.header.frame_id.data = frame;
        path_msg.header.frame_id.size = 3;
        path_msg.header.frame_id.capacity = 8;
    }

    const uint16_t n = path->num_points;
    geometry_msgs__msg__PoseStamped *poses =
        (geometry_msgs__msg__PoseStamped *)alloc.allocate(
            (size_t)n * sizeof(geometry_msgs__msg__PoseStamped), alloc.state);

    if (poses == NULL)
    {
        fprintf(stderr, "[MPCC] WARNING: failed to allocate raceline poses\n");
        if (frame != NULL)
        {
            alloc.deallocate(frame, alloc.state);
        }
        return;
    }

    for (uint16_t i = 0; i < n; ++i)
    {
        geometry_msgs__msg__PoseStamped__init(&poses[i]);

        char *pose_frame = (char *)alloc.allocate(8, alloc.state);
        if (pose_frame != NULL)
        {
            memcpy(pose_frame, "map", 4);
            poses[i].header.frame_id.data = pose_frame;
            poses[i].header.frame_id.size = 3;
            poses[i].header.frame_id.capacity = 8;
        }

        const float x = path->points[i].x_ref;
        const float y = path->points[i].y_ref;
        const float psi = path->points[i].phi_ref;

        poses[i].pose.position.x = x;
        poses[i].pose.position.y = y;
        poses[i].pose.position.z = 0.0;
        poses[i].pose.orientation.x = 0.0;
        poses[i].pose.orientation.y = 0.0;
        poses[i].pose.orientation.z = sin(0.5f * psi);
        poses[i].pose.orientation.w = cos(0.5f * psi);
    }

    path_msg.poses.data = poses;
    path_msg.poses.size = n;
    path_msg.poses.capacity = n;

    {
        const rcl_ret_t rc = rcl_publish(&g_raceline_pub, &path_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: failed to publish /mpcc/raceline: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }

    for (uint16_t i = 0; i < n; ++i)
    {
        if (poses[i].header.frame_id.data != NULL)
        {
            alloc.deallocate(poses[i].header.frame_id.data, alloc.state);
        }
    }
    alloc.deallocate(poses, alloc.state);

    if (frame != NULL)
    {
        alloc.deallocate(frame, alloc.state);
    }
}

/* -------------------------------------------------------------------------- */
/* Predicted Path Publisher                                                     */
/* -------------------------------------------------------------------------- */

static geometry_msgs__msg__PoseArray g_predicted_path_msg;
static int g_predicted_path_msg_inited = 0;

static void publish_predicted_path(const MPCCResult_t *result)
{
    MPCCConfiguration_t cfg = mpcc_get_configuration();
    uint16_t n = cfg.horizon_steps + 1;
    if (n > MPCC_MAX_HORIZON + 1) n = MPCC_MAX_HORIZON + 1;

    if (!g_predicted_path_msg_inited) {
        geometry_msgs__msg__PoseArray__init(&g_predicted_path_msg);
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        char *fid = (char *)alloc.allocate(8, alloc.state);
        if (fid) {
            memcpy(fid, "map", 4);
            g_predicted_path_msg.header.frame_id.data = fid;
            g_predicted_path_msg.header.frame_id.size = 3;
            g_predicted_path_msg.header.frame_id.capacity = 8;
        }
        geometry_msgs__msg__Pose *poses =
            (geometry_msgs__msg__Pose *)alloc.allocate(
                (MPCC_MAX_HORIZON + 1) * sizeof(geometry_msgs__msg__Pose),
                alloc.state);
        if (!poses) return;
        g_predicted_path_msg.poses.data = poses;
        g_predicted_path_msg.poses.capacity = MPCC_MAX_HORIZON + 1;
        g_predicted_path_msg_inited = 1;
    }

    g_predicted_path_msg.poses.size = n;
    g_predicted_path_msg.header.stamp = g_pose_msg.header.stamp;
    for (uint16_t i = 0; i < n; i++) {
        const MPCCState_t *st = &result->predicted_states[i];
        g_predicted_path_msg.poses.data[i].position.x = st->X;
        g_predicted_path_msg.poses.data[i].position.y = st->Y;
        g_predicted_path_msg.poses.data[i].position.z = 0.08;
        g_predicted_path_msg.poses.data[i].orientation.x = 0.0;
        g_predicted_path_msg.poses.data[i].orientation.y = 0.0;
        g_predicted_path_msg.poses.data[i].orientation.z = sin(st->psi * 0.5f);
        g_predicted_path_msg.poses.data[i].orientation.w = cos(st->psi * 0.5f);
    }

    { rcl_ret_t rc_ = rcl_publish(&g_predicted_path_pub, &g_predicted_path_msg, NULL); (void)rc_; }
}

/* -------------------------------------------------------------------------- */
/* Raceline Subscriber                                                         */
/* -------------------------------------------------------------------------- */

static rcl_subscription_t g_raceline_sub;
static nav_msgs__msg__Path g_raceline_msg;

static void raceline_callback(const void *msg_in)
{
    const nav_msgs__msg__Path *msg = (const nav_msgs__msg__Path *)msg_in;

    if (msg->poses.size < 2) {
        fprintf(stderr, "[MPCC] WARNING: raceline with %zu points (need >= 2)\n",
                msg->poses.size);
        return;
    }

    MPCCReferencePath_t ref_path;
    uint16_t n = (msg->poses.size > MPCC_MAX_PATH_POINTS)
                  ? MPCC_MAX_PATH_POINTS : (uint16_t)msg->poses.size;
    ref_path.num_points = n;

    for (uint16_t i = 0; i < n; i++) {
        const geometry_msgs__msg__PoseStamped *ps = &msg->poses.data[i];
        ref_path.points[i].x_ref  = (float)ps->pose.position.x;
        ref_path.points[i].y_ref  = (float)ps->pose.position.y;
        ref_path.points[i].vx_ref = (float)ps->pose.position.z; /* velocity in z */
        ref_path.points[i].phi_ref = (float)quat_to_yaw(
            ps->pose.orientation.x, ps->pose.orientation.y,
            ps->pose.orientation.z, ps->pose.orientation.w);
    }

    /* Build arc-length parameterization */
    ref_path.points[0].s_ref = 0.0f;
    for (uint16_t i = 1; i < n; i++) {
        float dx = ref_path.points[i].x_ref - ref_path.points[i - 1].x_ref;
        float dy = ref_path.points[i].y_ref - ref_path.points[i - 1].y_ref;
        ref_path.points[i].s_ref = ref_path.points[i - 1].s_ref
                                   + sqrtf(dx * dx + dy * dy);
    }

    /* Build curvature from heading differences */
    for (uint16_t i = 0; i < n; i++) {
        uint16_t prev = (i == 0) ? 0 : i - 1;
        uint16_t next = (i == n - 1) ? n - 1 : i + 1;
        float ds = ref_path.points[next].s_ref - ref_path.points[prev].s_ref;
        if (ds > 1e-6f) {
            float dpsi = ref_path.points[next].phi_ref
                       - ref_path.points[prev].phi_ref;
            while (dpsi >  (float)M_PI) dpsi -= 2.0f * (float)M_PI;
            while (dpsi < -(float)M_PI) dpsi += 2.0f * (float)M_PI;
            ref_path.points[i].kappa_ref = dpsi / ds;
        } else {
            ref_path.points[i].kappa_ref = 0.0f;
        }
    }

    ref_path.total_length = ref_path.points[n - 1].s_ref;
    /* /local_raceline is a rolling lookahead segment, not a closed lap. */
    ref_path.is_closed = 1;

    /* ---------------------------------------------------------------
     * Track bounds: look up from the CSV-loaded path (g_reference_path)
     * by finding the nearest point on the old path for each new point.
     *
     * nav_msgs/Path with PoseStamped has no field for track bounds —
     * position.z is already used for velocity. The CSV path has measured
     * bounds. Since both describe the same physical track, we match by
     * nearest Cartesian distance and copy bounds across.
     *
     * Fallback: if no CSV path is loaded yet, use a conservative 0.35 m.
     * --------------------------------------------------------------- */
    if (g_reference_path.num_points >= 2)
    {
        /* Transfer bounds from the CSV reference path point-by-point.
         * The incoming topic path may use a different discretization or
         * progression profile, so a single global arc-length ratio can
         * attach the wrong corridor farther along the horizon. */
        for (uint16_t i = 0; i < n; i++)
        {
            MPCCPathPoint_t bounds_pt;
            float s_query = mpcc_find_closest_s(
                &g_reference_path,
                ref_path.points[i].x_ref,
                ref_path.points[i].y_ref);
            float dx;
            float dy;
            float dist_sq;

            mpcc_path_interpolate(&g_reference_path, s_query, &bounds_pt);

            ref_path.points[i].left_bound = bounds_pt.left_bound;
            ref_path.points[i].right_bound = bounds_pt.right_bound;

            /* If the nearest CSV sample is still materially offset in
             * Cartesian space, the paths do not describe the same corridor.
             * Fall back to a conservative fixed-width lane instead. */
            dx = ref_path.points[i].x_ref - bounds_pt.x_ref;
            dy = ref_path.points[i].y_ref - bounds_pt.y_ref;
            dist_sq = dx * dx + dy * dy;
            if (dist_sq > 0.25f)
            {
                ref_path.points[i].left_bound = 0.35f;
                ref_path.points[i].right_bound = 0.35f;
            }
        }
    }
    else
    {
        /* No CSV path loaded yet — use conservative fixed bounds */
        for (uint16_t i = 0; i < n; i++)
        {
            ref_path.points[i].left_bound  = 0.35f;
            ref_path.points[i].right_bound = 0.35f;
        }
    }

    mpcc_set_reference_path(&ref_path);

    if (g_have_pose || g_have_odom)
    {
        /* Re-anchor progress onto the new local segment instead of
         * carrying a global-lap s hint into a short open path. */
        g_current_s = mpcc_find_closest_s(
            &ref_path,
            g_vehicle_state.pos_x,
            g_vehicle_state.pos_y);
    }
    else
    {
        g_current_s = 0.0f;
    }

    g_have_reference = 1;
    printf("[MPCC] Updated open raceline segment from topic (%d points, %.1f m) "
           "— bounds from %s, re-anchored s=%.2f\n",
           n,
           ref_path.total_length,
           (g_reference_path.num_points >= 2) ? "CSV lookup" : "fallback",
           g_current_s);

    publish_raceline(&ref_path);
}

/* -------------------------------------------------------------------------- */
/* ROS Callbacks                                                               */
/* -------------------------------------------------------------------------- */

static void odom_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const nav_msgs__msg__Odometry *msg = (const nav_msgs__msg__Odometry *)msg_in;

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double heading = quat_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double omega = msg->twist.twist.angular.z;
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);

    if (!odom_twist_is_sane(vx, vy, omega, &now))
    {
        return;
    }

    if (!g_using_map_pose)
    {
        g_vehicle_state.pos_x = (float)x;
        g_vehicle_state.pos_y = (float)y;
        g_vehicle_state.heading = (float)heading;
        g_have_pose = 1;
    }

    g_vehicle_state.long_vel = (float)vx;
    g_vehicle_state.lat_vel = (float)vy;
    g_vehicle_state.yaw_rate = (float)omega;

    g_latest_vx_mps = vx;
    g_latest_vy_mps = vy;
    g_latest_omega = omega;
    g_last_odom_time = now;
    g_have_odom = 1;
    g_odom_update_count++;

    if (g_odom_watchdog_active)
    {
        fprintf(stderr,
                "[MPCC] WATCHDOG: odometry recovered (vx=%.2f vy=%.2f w=%.2f)\n",
                vx,
                vy,
                omega);
        g_odom_watchdog_active = 0;
    }
}

static void pose_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const geometry_msgs__msg__PoseWithCovarianceStamped *msg =
        (const geometry_msgs__msg__PoseWithCovarianceStamped *)msg_in;

    const double heading = quat_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    g_vehicle_state.pos_x = (float)msg->pose.pose.position.x;
    g_vehicle_state.pos_y = (float)msg->pose.pose.position.y;
    g_vehicle_state.heading = (float)heading;

    if (!g_using_map_pose)
    {
        printf("[MPCC] Map-frame pose received on %s; switching to map pose\n",
               g_pose_topic);
        g_using_map_pose = 1;
    }

    g_have_pose = 1;

    /* ------- EKF-driven control: solve on new pose when the nominal
     * control period has elapsed and a fresh odom sample is available. */

    if (!g_have_reference || !g_have_odom)
    {
        republish_last_drive_command("waiting_for_reference_or_odom");
        return;
    }

    {
        struct timespec now;

        clock_gettime(CLOCK_MONOTONIC, &now);
        if (g_watchdog_timeout_sec > 0.0
            && timespec_is_set(&g_last_odom_time))
        {
            const double odom_age = timespec_diff_sec(&g_last_odom_time, &now);

            if (odom_age > g_watchdog_timeout_sec)
            {
                g_odom_watchdog_active = 1;
                if (should_log_throttled(&g_last_odom_watchdog_log_time,
                                         &now,
                                         MPCC_ODOM_LOG_THROTTLE_SEC))
                {
                    fprintf(stderr,
                            "[MPCC] WATCHDOG: odometry stale (%.1f ms > %.1f ms), "
                            "skipping solve\n",
                            odom_age * 1000.0,
                            g_watchdog_timeout_sec * 1000.0);
                }
                republish_last_drive_command("stale_odom_watchdog");
                return;
            }
        }
    }

    /* Only solve when this fresh EKF pose is paired with a newer odom sample. */
    if (g_odom_update_count == g_last_solve_odom_update_count)
    {
        if (g_verbose)
        {
            printf("[MPCC] Waiting for fresh odometry to pair with EKF pose; skipping solve\n");
        }
        republish_last_drive_command("waiting_for_fresh_odom");
        return;
    }

    double solve_gap_sec = 0.0;

    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        if ((g_last_imu_time.tv_sec != 0 || g_last_imu_time.tv_nsec != 0)
            && timespec_diff_sec(&g_last_imu_time, &now) > 0.05)
        {
            if (g_verbose)
            {
                printf("[MPCC] WARNING: IMU stale (%.0f ms), "
                       "falling back to odometry yaw rate\n",
                       timespec_diff_sec(&g_last_imu_time, &now) * 1000.0);
            }
            g_vehicle_state.yaw_rate = (float)g_latest_omega;
        }
    }

    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        if ((g_prev_solve_time.tv_sec != 0 || g_prev_solve_time.tv_nsec != 0)
            && g_nominal_control_dt_sec > 0.0)
        {
            solve_gap_sec = timespec_diff_sec(&g_prev_solve_time, &now);

            if (solve_gap_sec + 1e-9 < g_nominal_control_dt_sec)
            {
                if (g_verbose)
                {
                    printf("[MPCC] Solve gate active (%.1f ms < %.1f ms); keeping previous command\n",
                           solve_gap_sec * 1000.0,
                           g_nominal_control_dt_sec * 1000.0);
                }
                republish_last_drive_command("solve_rate_gate");
                return;
            }
        }
    }

    reset_mpcc_after_stale_solve_gap(solve_gap_sec);

    g_last_solve_odom_update_count = g_odom_update_count;

    MPCCState_t mpcc_state = mpcc_state_from_vehicle_state(&g_vehicle_state, g_current_s);
    {
        struct timespec solve_now;
        clock_gettime(CLOCK_MONOTONIC, &solve_now);

        if (g_adapt_cross_call_scale)
        {
            if (g_prev_solve_time.tv_sec != 0 || g_prev_solve_time.tv_nsec != 0)
            {
                double dt_actual = timespec_diff_sec(&g_prev_solve_time, &solve_now);
                if (dt_actual > 0.0005 && dt_actual < 0.5)  /* sanity: 2 Hz–2 kHz */
                {
                    const double speed_for_adapt = fabs(g_latest_vx_mps);
                    if (speed_for_adapt < 0.5)
                    {
                        if (!g_cross_call_motion_gate_logged)
                        {
                            fprintf(stderr,
                                    "[MPCC] INFO: holding adaptive cross-call at baseline until |vx| >= 0.50 m/s (current %.2f m/s)\n",
                                    speed_for_adapt);
                            g_cross_call_motion_gate_logged = 1;
                        }
                    }
                    else
                    {
                        if (!g_control_period_explicit && !g_adapt_nominal_dt_bootstrapped)
                        {
                            g_nominal_control_dt_sec = dt_actual;
                            g_control_dt_filtered = dt_actual;
                            g_adapt_nominal_dt_bootstrapped = 1;

                            fprintf(stderr,
                                    "[MPCC] INFO: adaptive cross-call nominal bootstrapped to %.1f ms from measured solve cadence\n",
                                    dt_actual * 1000.0);
                        }

                        const double alpha =
                            (fabs(dt_actual - g_control_dt_filtered) > 0.02) ? 0.3 : 0.1;
                        const double min_dt = 0.5 * g_nominal_control_dt_sec;
                        const double max_dt = 2.0 * g_nominal_control_dt_sec;

                        g_control_dt_filtered =
                            (1.0 - alpha) * g_control_dt_filtered + alpha * dt_actual;
                        if (g_control_dt_filtered < min_dt) g_control_dt_filtered = min_dt;
                        if (g_control_dt_filtered > max_dt) g_control_dt_filtered = max_dt;

                        double prediction_dt = (double)g_solver_dt_sec;
                        if (prediction_dt > 0.0)
                        {
                            double scale = g_control_dt_filtered / prediction_dt;

                            if (scale > 1.0)
                            {
                                if (!g_cross_call_upper_clamp_logged)
                                {
                                    fprintf(stderr,
                                            "[MPCC] WARNING: adaptive cross-call measured %.1f ms solve cadence vs %.1f ms prediction step; clamping scale %.3f -> 1.000 to avoid multi-stage warm-start jumps\n",
                                            g_control_dt_filtered * 1000.0,
                                            prediction_dt * 1000.0,
                                            scale);
                                    g_cross_call_upper_clamp_logged = 1;
                                }
                                scale = 1.0;
                            }

                            MPCCConfiguration_t cfg = mpcc_get_configuration();
                            cfg.cross_call_rate_scale = (float)scale;
                            mpcc_set_configuration(&cfg);
                        }
                    }
                }
            }
        }

        g_prev_solve_time = solve_now;
    }

    MPCCResult_t result;
    const MPCCStatus_t status = mpcc_compute_control(&mpcc_state, &result);

    g_solve_count++;

    if (status != MPCC_STATUS_SUCCESS && status != MPCC_STATUS_MAX_ITERATIONS)
    {
        {
            const double control_dt_sec = current_control_dt_sec();
            float delta_safe = 0.5f * g_prev_delta_cmd;
            float a_x_safe = g_prev_ax_cmd;
            double v_safe;
            float steering_velocity_cmd;

            delta_safe = limit_steering_delta(delta_safe, control_dt_sec);
            steering_velocity_cmd = (control_dt_sec > 0.0)
                ? (float)((delta_safe - g_prev_delta_cmd) / control_dt_sec)
                : 0.0f;

            if (a_x_safe > -1.0f)
            {
                a_x_safe = -1.0f;
            }

            v_safe = g_latest_vx_mps + (double)a_x_safe * g_solver_dt_sec;
            if (v_safe < 0.0)
            {
                v_safe = 0.0;
            }
            if (v_safe > g_vx_max_mps)
            {
                v_safe = g_vx_max_mps;
            }

            g_drive_msg.drive.steering_angle = delta_safe;
            g_drive_msg.drive.steering_angle_velocity = steering_velocity_cmd;
            g_drive_msg.drive.speed = g_publish_speed_command ? (float)v_safe : 0.0f;
            g_drive_msg.drive.acceleration = a_x_safe;

            g_prev_delta_cmd = delta_safe;
            g_prev_speed_cmd = (float)v_safe;
            g_prev_ax_cmd = a_x_safe;

            log_solve_metrics(g_solve_count,
                              status,
                              &mpcc_state,
                              &result,
                              delta_safe,
                              a_x_safe,
                              0.0f,
                              (float)v_safe,
                              solve_gap_sec,
                              g_nominal_control_dt_sec);

            if (g_verbose || g_solve_count <= 20 || (g_solve_count % 10U) == 0U)
            {
                log_path_alignment_debug(g_solve_count, mpcc_state.s);
            }
        }
        {
            rcl_ret_t rc_ = rcl_publish(&g_drive_pub, &g_drive_msg, NULL);
            if (rc_ == RCL_RET_OK)
            {
                g_have_published_drive_cmd = 1;
            }
        }
        return;
    }

    float a_x_cmd = result.optimal_control.a_x;
    float delta_cmd = result.optimal_control.delta;
    const float v_theta_cmd = result.optimal_control.v_theta;
    const double control_dt_sec = current_control_dt_sec();
    float steering_velocity_cmd;

    /* Clamp acceleration for hardware safety */
    if (a_x_cmd < (float)g_ax_min_hardware)
    {
        a_x_cmd = (float)g_ax_min_hardware;
    }

    /* Mirror the simulator's constant-power acceleration limit so the
     * published command matches what the drivetrain can achieve at speed. */
    if (a_x_cmd > 0.0f)
    {
        const float accel_switch_speed = 7.319f;
        const float accel_limit = 7.31f;
        const float vx_measured = (float)g_latest_vx_mps;

        if (vx_measured > accel_switch_speed)
        {
            float a_x_power_max = accel_limit * accel_switch_speed / vx_measured;
            if (a_x_cmd > a_x_power_max)
                a_x_cmd = a_x_power_max;
        }
    }

    delta_cmd = limit_steering_delta(delta_cmd, control_dt_sec);
    steering_velocity_cmd = (control_dt_sec > 0.0)
        ? (float)((delta_cmd - g_prev_delta_cmd) / control_dt_sec)
        : 0.0f;


    double v_cmd;
    float vx_predicted = result.predicted_states[1].vx;
    double v_euler = g_latest_vx_mps + (double)a_x_cmd * g_solver_dt_sec;
    const float v_pred_min = 0.1f;
    const float v_pred_max = (float)g_vx_max_mps * 1.2f;
    const double v_pred_err = fabs((double)vx_predicted - v_euler);

    if (status == MPCC_STATUS_SUCCESS
        && isfinite(vx_predicted)
        && vx_predicted > v_pred_min
        && vx_predicted < v_pred_max
        && v_pred_err < 0.5)
    {
        /* Solver prediction is sane and consistent with the measured state. */
        v_cmd = (double)vx_predicted;
    }
    else
    {
        /* Safer fallback: simple Euler step from measured vx and commanded accel. */
        v_cmd = v_euler;
    }

    /* Apply minimum velocity floor only when the solver wants to accelerate,
     * so the car can still brake when the solver commands a_x < 0. */
    if (a_x_cmd >= 0.0f && v_cmd < g_vx_min_cmd)
    {
        v_cmd = g_vx_min_cmd;
    }
    if (v_cmd > g_vx_max_mps)
    {
        v_cmd = g_vx_max_mps;
    }

    /* Store for next iteration (keep-prev on failure) */
    g_prev_delta_cmd = delta_cmd;
    g_prev_speed_cmd = (float)v_cmd;
    g_prev_ax_cmd = a_x_cmd;

    g_drive_msg.header.stamp.sec = 0;
    g_drive_msg.header.stamp.nanosec = 0;
    g_drive_msg.drive.steering_angle = delta_cmd;
    g_drive_msg.drive.steering_angle_velocity = steering_velocity_cmd;
    g_drive_msg.drive.speed = g_publish_speed_command ? (float)v_cmd : 0.0f;
    g_drive_msg.drive.acceleration = a_x_cmd;
    g_drive_msg.drive.jerk = 0.0f;

    align_first_prediction_step(
        &result,
        &mpcc_state,
        delta_cmd,
        a_x_cmd,
        v_theta_cmd,
        (float)v_cmd);

    {
        const rcl_ret_t rc = rcl_publish(&g_drive_pub, &g_drive_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: failed to publish drive command: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
        else
        {
            g_have_published_drive_cmd = 1;
        }
    }

    log_solve_metrics(g_solve_count,
                      status,
                      &mpcc_state,
                      &result,
                      delta_cmd,
                      a_x_cmd,
                      v_theta_cmd,
                      (float)v_cmd,
                      solve_gap_sec,
                      g_nominal_control_dt_sec);

    if (g_solve_count <= 20 || (g_solve_count % 10U) == 0U)
    {
        fprintf(stderr,
                "[MPCC %3u] s=%.2f x=%.2f y=%.2f psi=%.2f vx=%.2f | d=%.4f ax=%.3f vt=%.3f vcmd=%.2f | st=%d it=%u\n",
                g_solve_count,
                mpcc_state.s,
                mpcc_state.X,
                mpcc_state.Y,
                mpcc_state.psi,
                mpcc_state.vx,
                delta_cmd,
                a_x_cmd,
                v_theta_cmd,
                (float)v_cmd,
                (int)status,
                result.admm_iterations);
        fflush(stderr);
    }

    /* Publish predicted horizon path for visualization */
    publish_predicted_path(&result);
}

/* -------------------------------------------------------------------------- */
/* IMU & Servo Callbacks                                                       */
/* -------------------------------------------------------------------------- */

static void imu_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    g_vehicle_state.yaw_rate = (float)msg->data;
    clock_gettime(CLOCK_MONOTONIC, &g_last_imu_time);
}

static void servo_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    const double servo_val = msg->data;

    /* Invert the forward mapping: servo_val = GAIN * corrected_angle + OFFSET
     * => corrected_angle = (servo_val - OFFSET) / GAIN */
    const double corrected = (servo_val - STEERING_TO_SERVO_OFFSET) / STEERING_TO_SERVO_GAIN;

    /* Invert the polynomial correction: corrected = C2*|d|^2 + C1*|d| + C0
     * Solve quadratic: C2*a^2 + C1*a + (C0 - |corrected|) = 0
     * where a = |d|  */
    const double abs_corrected = fabs(corrected);
    const double discriminant =
        STEERING_CORRECTION_C1 * STEERING_CORRECTION_C1
        - 4.0 * STEERING_CORRECTION_C2 * (STEERING_CORRECTION_C0 - abs_corrected);

    if (discriminant >= 0.0)
    {
        const double abs_delta =
            (-STEERING_CORRECTION_C1 + sqrt(discriminant)) / (2.0 * STEERING_CORRECTION_C2);
        g_actual_steering_angle = (corrected >= 0.0) ? abs_delta : -abs_delta;
        g_use_steering_feedback = 1;
    }
}

/* -------------------------------------------------------------------------- */
/* Runtime Setup                                                               */
/* -------------------------------------------------------------------------- */

static void read_runtime_environment(void)
{
    const char *value;

    if ((value = getenv("MPCC_ODOM_TOPIC")) != NULL && value[0] != '\0')
    {
        g_odom_topic = value;
    }
    if ((value = getenv("MPCC_DRIVE_TOPIC")) != NULL && value[0] != '\0')
    {
        g_drive_topic = value;
    }
    if ((value = getenv("MPCC_IMU_TOPIC")) != NULL && value[0] != '\0')
    {
        g_imu_topic = value;
    }
    if ((value = getenv("MPCC_POSE_TOPIC")) != NULL && value[0] != '\0')
    {
        g_pose_topic = value;
    }
    else if ((value = getenv("MPCC_AMCL_TOPIC")) != NULL && value[0] != '\0')
    {
        /* Backward-compatible alias for map-frame pose topic. */
        g_pose_topic = value;
    }

    if ((value = getenv("MPCC_SERVO_TOPIC")) != NULL && value[0] != '\0')
    {
        g_servo_topic = value;
    }

    if ((value = getenv("MPCC_AX_MIN_HARDWARE")) != NULL)
    {
        const double val = atof(value);
        if (val < 0.0 && val >= -20.0)
        {
            g_ax_min_hardware = val;
        }
    }

    if ((value = getenv("MPCC_CONTROL_PERIOD_MS")) != NULL)
    {
        const double period_ms = atof(value);
        if (period_ms > 0.0 && period_ms <= 1000.0)
        {
            g_nominal_control_dt_sec = period_ms * 1e-3;
            g_control_period_explicit = 1;
        }
        else if (period_ms == 0.0)
        {
            g_nominal_control_dt_sec = 1.0 / MPCC_CONTROL_RATE_HZ;
            g_control_period_explicit = 0;
        }
    }

    if ((value = getenv("MPCC_WATCHDOG_TIMEOUT")) != NULL)
    {
        const double timeout = atof(value);
        if (timeout >= 0.0 && timeout <= 5.0)
        {
            g_watchdog_timeout_sec = timeout;
        }
    }

    if ((value = getenv("MPCC_VERBOSE")) != NULL)
    {
        g_verbose = atoi(value);
    }

    if ((value = getenv("MPCC_TRAJECTORY_FILE")) != NULL && value[0] != '\0')
    {
        g_trajectory_file = value;
    }

    if ((value = getenv("MPCC_VX_MIN_CMD")) != NULL)
    {
        const double val = atof(value);
        if (val >= 0.0 && val <= 5.0)
        {
            g_vx_min_cmd = val;
        }
    }

    if ((value = getenv("MPCC_ADAPT_CROSS_CALL_SCALE")) != NULL)
    {
        g_adapt_cross_call_scale = (atoi(value) != 0);
    }

    if ((value = getenv("MPCC_PUBLISH_SPEED_COMMAND")) != NULL)
    {
        g_publish_speed_command = (atoi(value) != 0);
    }

    if ((value = getenv("MPCC_USE_LOCAL_RACELINE")) != NULL)
    {
        g_use_local_raceline = (atoi(value) != 0);
    }
}

static void configure_mpcc_from_environment(void)
{
    mpcc_initialize();
    if (g_ax_min_hardware > 0.0)
    {
        g_ax_min_hardware = 0.0;
    }

    MPCCConfiguration_t cfg = mpcc_get_configuration();

    /* Each parameter with an alias: canonical name takes priority.
     * Warn loudly if both are set so the user knows which one wins. */
    {
        const char *v_canon = getenv("Q_CONTOURING");
        const char *v_alias = getenv("Q_N");
        if (v_canon && v_alias)
            fprintf(stderr, "[MPCC] WARNING: both Q_CONTOURING and Q_N are set — "
                            "Q_CONTOURING takes priority (%.1f)\n", atof(v_canon));
        if (v_canon)      cfg.weight_contouring = (float)atof(v_canon);
        else if (v_alias) cfg.weight_contouring = (float)atof(v_alias);
    }
    {
        const char *v_canon = getenv("Q_LAG");
        const char *v_alias = getenv("Q_ALPHA");
        if (v_canon && v_alias)
            fprintf(stderr, "[MPCC] WARNING: both Q_LAG and Q_ALPHA are set — "
                            "Q_LAG takes priority (%.1f)\n", atof(v_canon));
        if (v_canon)      cfg.weight_lag = (float)atof(v_canon);
        else if (v_alias) cfg.weight_lag = (float)atof(v_alias);
    }
    {
        const char *v_canon = getenv("Q_WALL_CLEARANCE");
        const char *v_alias = getenv("Q_WALL");
        if (v_canon && v_alias)
            fprintf(stderr, "[MPCC] WARNING: both Q_WALL_CLEARANCE and Q_WALL are set — "
                            "Q_WALL_CLEARANCE takes priority (%.1f)\n", atof(v_canon));
        if (v_canon)      cfg.weight_wall_clearance = (float)atof(v_canon);
        else if (v_alias) cfg.weight_wall_clearance = (float)atof(v_alias);
    }
    {
        const char *v_canon = getenv("Q_CONTOURING_TERM");
        const char *v_alias = getenv("Q_N_TERM");
        if (v_canon && v_alias)
            fprintf(stderr, "[MPCC] WARNING: both Q_CONTOURING_TERM and Q_N_TERM are set — "
                            "Q_CONTOURING_TERM takes priority (%.1f)\n", atof(v_canon));
        if (v_canon)      cfg.weight_contouring_terminal = (float)atof(v_canon);
        else if (v_alias) cfg.weight_contouring_terminal = (float)atof(v_alias);
    }
    {
        const char *v_canon = getenv("Q_LAG_TERM");
        const char *v_alias = getenv("Q_ALPHA_TERM");
        if (v_canon && v_alias)
            fprintf(stderr, "[MPCC] WARNING: both Q_LAG_TERM and Q_ALPHA_TERM are set — "
                            "Q_LAG_TERM takes priority (%.1f)\n", atof(v_canon));
        if (v_canon)      cfg.weight_lag_terminal = (float)atof(v_canon);
        else if (v_alias) cfg.weight_lag_terminal = (float)atof(v_alias);
    }

    /* Single-name parameters — no alias conflict possible */
    const char *v;
    int requested_horizon = -1;
    if ((v = getenv("Q_HEADING")) != NULL)     cfg.weight_heading           = (float)atof(v);
    if ((v = getenv("Q_HEADING_TERM")) != NULL) cfg.weight_heading_terminal = (float)atof(v);
    if ((v = getenv("WALL_CLEARANCE_MARGIN")) != NULL) cfg.wall_clearance_margin = (float)atof(v);
    if ((v = getenv("MPCC_TRACK_BUFFER")) != NULL) cfg.track_safety_buffer = (float)atof(v);
    if ((v = getenv("Q_PROGRESS")) != NULL)    cfg.weight_progress          = (float)atof(v);
    if ((v = getenv("Q_PHYSICAL_PROGRESS")) != NULL) cfg.weight_physical_progress = (float)atof(v);
    if ((v = getenv("MPCC_S_QP_WINDOW")) != NULL) cfg.s_qp_window           = (float)atof(v);
    if ((v = getenv("Q_VX")) != NULL)          cfg.weight_vx                = (float)atof(v);
    if ((v = getenv("VX_REF")) != NULL)        cfg.vx_ref                   = (float)atof(v);
    if ((v = getenv("MPCC_USE_RACELINE_VX_REF")) != NULL)
        cfg.use_raceline_vx_ref = (uint8_t)(atoi(v) != 0);
    if ((v = getenv("MPCC_USE_RACELINE_VX_LIMIT")) != NULL)
        cfg.use_raceline_vx_limit = (uint8_t)(atoi(v) != 0);
    if ((v = getenv("MPCC_RACELINE_VX_LIMIT_SCALE")) != NULL)
        cfg.raceline_vx_limit_scale = (float)atof(v);
    if ((v = getenv("Q_VY")) != NULL)          cfg.weight_vy                = (float)atof(v);
    if ((v = getenv("Q_OMEGA")) != NULL)       cfg.weight_omega             = (float)atof(v);
    if ((v = getenv("R_DELTA")) != NULL)       cfg.weight_delta             = (float)atof(v);
    if ((v = getenv("R_AX")) != NULL)          cfg.weight_ax                = (float)atof(v);
    if ((v = getenv("R_VTHETA")) != NULL)      cfg.weight_v_theta           = (float)atof(v);
    if ((v = getenv("W_VTHETA_PHYSICAL")) != NULL) cfg.weight_vtheta_physical = (float)atof(v);
    if ((v = getenv("W_DELTA_RATE")) != NULL)  cfg.weight_delta_rate        = (float)atof(v);
    if ((v = getenv("W_AX_RATE")) != NULL)     cfg.weight_ax_rate           = (float)atof(v);
    if ((v = getenv("W_VTHETA_RATE")) != NULL) cfg.weight_v_theta_rate      = (float)atof(v);
    if ((v = getenv("Q_PROGRESS_TERM")) != NULL) cfg.weight_progress_terminal = (float)atof(v);
#ifndef USE_OSQP
    if ((v = getenv("ADMM_RHO")) != NULL)      cfg.admm_rho                 = (float)atof(v);
    if ((v = getenv("ADMM_MAX_ITER")) != NULL) cfg.admm_max_iterations      = (uint16_t)atoi(v);
    if ((v = getenv("ADMM_TOL")) != NULL)      cfg.admm_tolerance           = (float)atof(v);
    if ((v = getenv("ADMM_RHO_U")) != NULL)    cfg.admm_rho_u               = (float)atof(v);
    if ((v = getenv("ADMM_ADAPTIVE_RHO")) != NULL)
        cfg.admm_adaptive_rho = (uint8_t)(atoi(v) != 0);
    if ((v = getenv("ADMM_ALPHA_RELAX")) != NULL) cfg.admm_alpha_relax      = (float)atof(v);
#endif
    if ((v = getenv("MPCC_ACCEPT_MAX_ITER")) != NULL)
        cfg.accept_max_iterations = (uint8_t)(atoi(v) != 0);
    if ((v = getenv("MPCC_MAX_ITER_PRIMAL_TOL")) != NULL)
        cfg.max_iter_primal_tolerance = (float)atof(v);
    if ((v = getenv("MPCC_MAX_ITER_DUAL_TOL")) != NULL)
        cfg.max_iter_dual_tolerance = (float)atof(v);
    if ((v = getenv("MPCC_MAX_ITER_TRACK_TOL")) != NULL)
        cfg.max_iter_track_violation_tolerance = (float)atof(v);
    if ((v = getenv("MPCC_WARM_START_MAX_S_ERROR")) != NULL)
        cfg.warm_start_max_s_error = (float)atof(v);
    if ((v = getenv("HORIZON")) != NULL)
    {
        requested_horizon = atoi(v);
        cfg.horizon_steps = (uint16_t)requested_horizon;
    }
    if ((v = getenv("DT")) != NULL)            cfg.dt                       = (float)atof(v);
    if ((v = getenv("V_THETA_MAX")) != NULL)   cfg.v_theta_max              = (float)atof(v);
    if ((v = getenv("V_THETA_MIN")) != NULL)   cfg.v_theta_min              = (float)atof(v);
    if ((v = getenv("MU")) != NULL)            cfg.mu                       = (float)atof(v);
    if ((v = getenv("C_SF")) != NULL)          cfg.C_Sf                     = (float)atof(v);
    if ((v = getenv("C_SR")) != NULL)          cfg.C_Sr                     = (float)atof(v);
    if ((v = getenv("AX_MAX")) != NULL)        cfg.ax_max                   = (float)atof(v);
    if ((v = getenv("AX_MIN")) != NULL)        cfg.ax_min                   = (float)atof(v);
    if ((v = getenv("DELTA_RATE_MAX")) != NULL) cfg.delta_rate_max          = (float)atof(v);

    if ((float)g_ax_min_hardware > cfg.ax_min)
        cfg.ax_min = (float)g_ax_min_hardware;

    if ((v = getenv("MPCC_CROSS_CALL_SCALE")) != NULL)
        cfg.cross_call_rate_scale = (float)atof(v);

    mpcc_set_configuration(&cfg);

    if (requested_horizon > MPCC_MAX_HORIZON)
    {
        fprintf(stderr,
                "[MPCC] WARNING: requested HORIZON=%d exceeds compile-time MPCC_MAX_HORIZON=%d; clamping to %d\n",
                requested_horizon,
                MPCC_MAX_HORIZON,
                MPCC_MAX_HORIZON);
    }

    /* If the user hasn't explicitly set the cross-call scale, auto-compute
     * from control rate and the (possibly overridden) prediction dt. */
    if (getenv("MPCC_CROSS_CALL_SCALE") == NULL)
    {
        float dt = cfg.dt;
        if (dt > 0.0f)
            cfg.cross_call_rate_scale =
                (float)(g_nominal_control_dt_sec / (double)dt);
        mpcc_set_configuration(&cfg);
    }

    cfg = mpcc_get_configuration();
    g_delta_rate_limit = cfg.delta_rate_max;

    g_solver_dt_sec = cfg.dt;
    if (g_solver_dt_sec <= 0.0)
    {
        g_solver_dt_sec = MPCC_DEFAULT_DT;
    }

    g_vx_max_mps = cfg.vx_max;
    if (g_vx_max_mps <= 0.0)
    {
        g_vx_max_mps = 8.0;
    }

    g_control_dt_filtered = cfg.cross_call_rate_scale * (double)cfg.dt;
    if (g_control_dt_filtered <= 0.0)
    {
        g_control_dt_filtered = g_nominal_control_dt_sec;
    }
    if (g_control_dt_filtered <= 0.0)
    {
        g_control_dt_filtered = 1.0 / MPCC_CONTROL_RATE_HZ;
    }

        printf("[MPCC] Config: solver=%s N=%d dt=%.3f Q_c=%.1f Q_l=%.1f Q_head=%.1f Q_wall=%.1f wall_margin=%.3f track_buffer=%.3f s_window=%.2f Q_prog=%.1f Q_phys_prog=%.1f Q_vx=%.1f use_csv_vx_ref=%u use_csv_vx_limit=%u R_delta=%.2f R_vtheta=%.2f W_vtheta_phys=%.2f W_vtheta_rate=%.2f warm_s_err=%.2f ax_min_hw=%.1f delta_rate=%.3f cross_call=%.4f adapt_cross_call=%d accept_max_iter=%u vx_min_cmd=%.2f rho=%.3f rho_u=%.3f adaptive_rho=%u max_iter=%u tol=%.4f\n",
#ifdef USE_OSQP
           "OSQP",
#else
           "ADMM+Riccati",
#endif
           cfg.horizon_steps,
           cfg.dt,
           cfg.weight_contouring,
           cfg.weight_lag,
           cfg.weight_heading,
            cfg.weight_wall_clearance,
            cfg.wall_clearance_margin,
           cfg.track_safety_buffer,
           cfg.s_qp_window,
           cfg.weight_progress,
           cfg.weight_physical_progress,
           cfg.weight_vx,
           (unsigned)cfg.use_raceline_vx_ref,
           (unsigned)cfg.use_raceline_vx_limit,
           cfg.weight_delta,
           cfg.weight_v_theta,
           cfg.weight_vtheta_physical,
           cfg.weight_v_theta_rate,
           cfg.warm_start_max_s_error,
           g_ax_min_hardware,
           cfg.delta_rate_max,
           cfg.cross_call_rate_scale,
           g_adapt_cross_call_scale,
           (unsigned)cfg.accept_max_iterations,
           g_vx_min_cmd,
           cfg.admm_rho,
           cfg.admm_rho_u > 0.0f ? cfg.admm_rho_u : cfg.admm_rho,
           (unsigned)cfg.admm_adaptive_rho,
           (unsigned)cfg.admm_max_iterations,
           cfg.admm_tolerance);
}

static const char *autodetect_trajectory_file(void)
{
    static const char *candidates[] = {
        "hardware_centerline.csv",
        "my_track_centerline.csv",
        "f1tenth_planning/trajectories/hardware_centerline.csv",
        "f1tenth_planning/trajectories/my_track_centerline.csv",
        "../f1tenth_planning/trajectories/hardware_centerline.csv",
        "../f1tenth_planning/trajectories/my_track_centerline.csv",
        "/ros2_ws/src/f1tenth_planning/trajectories/hardware_centerline.csv",
        "/ros2_ws/src/f1tenth_planning/trajectories/my_track_centerline.csv",
        NULL
    };

    for (int i = 0; candidates[i] != NULL; ++i)
    {
        FILE *probe = fopen(candidates[i], "r");
        if (probe != NULL)
        {
            fclose(probe);
            return candidates[i];
        }
    }

    return NULL;
}

/* -------------------------------------------------------------------------- */
/* Main                                                                        */
/* -------------------------------------------------------------------------- */

int main(int argc, const char *argv[])
{
    read_runtime_environment();
    configure_mpcc_from_environment();

    g_current_s = 0;

    if (argc >= 2 && argv[1] != NULL && argv[1][0] != '\0')
    {
        g_trajectory_file = argv[1];
    }
    if (g_trajectory_file == NULL)
    {
        g_trajectory_file = autodetect_trajectory_file();
    }

    if (g_trajectory_file == NULL)
    {
        fprintf(stderr,
                "[MPCC] FATAL: no trajectory file provided (arg or MPCC_TRAJECTORY_FILE)\n");
        return 1;
    }

    if (!load_trajectory_csv(g_trajectory_file, &g_reference_path))
    {
        fprintf(stderr, "[MPCC] FATAL: could not load reference path\n");
        return 1;
    }

    mpcc_set_reference_path(&g_reference_path);
    g_have_reference = 1;

    printf("[MPCC] Hardware topics: odom=%s pose=%s imu=%s servo=%s drive=%s\n",
           g_odom_topic, g_pose_topic, g_imu_topic, g_servo_topic, g_drive_topic);
        if (g_watchdog_timeout_sec > 0.0)
        {
         printf("[MPCC] Odom watchdog: %.0f ms | twist sanity: |vx|<=%.1f |vy|<=%.1f |w|<=%.1f\n",
             g_watchdog_timeout_sec * 1000.0,
             MPCC_ODOM_MAX_ABS_VX_MPS,
             MPCC_ODOM_MAX_ABS_VY_MPS,
             MPCC_ODOM_MAX_ABS_YAW_RATE_RADPS);
        }
        else
        {
         printf("[MPCC] Odom watchdog: disabled | twist sanity: |vx|<=%.1f |vy|<=%.1f |w|<=%.1f\n",
             MPCC_ODOM_MAX_ABS_VX_MPS,
             MPCC_ODOM_MAX_ABS_VY_MPS,
             MPCC_ODOM_MAX_ABS_YAW_RATE_RADPS);
        }
        printf("[MPCC] EKF-driven mode | solve gate: fresh pose + fresh odom + nominal_dt | nominal_dt: %.1f ms | cross_call: %s | trajectory: %s | local_raceline: %s\n",
            g_nominal_control_dt_sec * 1000.0,
            g_adapt_cross_call_scale ? "adaptive" : "fixed",
            g_trajectory_file,
            g_use_local_raceline ? "enabled" : "disabled");

    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;
    if (rclc_support_init(&support, argc, argv, &allocator) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: rclc_support_init failed\n");
        return 1;
    }

    rcl_node_t node;
    if (rclc_node_init_default(&node, "mpcc_hardware_node", "", &support) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: rclc_node_init_default failed\n");
        return 1;
    }

    /* Raceline publisher with transient-local durability for rviz. */
    {
        rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
        pub_opts.qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        pub_opts.qos.depth = 1;
        pub_opts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

        const rcl_ret_t rc = rcl_publisher_init(
            &g_raceline_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
            "/mpcc/raceline",
            &pub_opts);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: failed to init /mpcc/raceline: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
        else
        {
            publish_raceline(&g_reference_path);
            printf("[MPCC] Published raceline on /mpcc/raceline\n");
        }
    }

    if (rclc_subscription_init_default(
            &g_odom_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            g_odom_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: odom subscription init failed\n");
        return 1;
    }

    if (rclc_subscription_init_default(
            &g_pose_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
            g_pose_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: pose subscription init failed\n");
        return 1;
    }

    if (rclc_subscription_init_default(
            &g_imu_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            g_imu_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: imu subscription init failed\n");
        return 1;
    }

    int g_servo_sub_ok = 0;
    if (rclc_subscription_init_default(
            &g_servo_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            g_servo_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] WARNING: servo subscription init failed (steering feedback disabled)\n");
        rcl_reset_error();
    }
    else
    {
        g_servo_sub_ok = 1;
    }

    if (g_use_local_raceline)
    {
        /* Optional local segment override for the CSV reference path. */
        if (rclc_subscription_init_default(
                &g_raceline_sub,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
                "/local_raceline") != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: raceline subscription init failed\n");
            rcl_reset_error();
        }
        else
        {
            g_raceline_sub_ok = 1;
        }
    }
    else
    {
        printf("[MPCC] Using CSV trajectory only; ignoring /local_raceline updates\n");
    }

    if (rclc_publisher_init_default(
            &g_drive_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped),
            g_drive_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: drive publisher init failed\n");
        return 1;
    }

    if (rclc_publisher_init_default(
            &g_predicted_path_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
            "/mpcc/predicted_path") != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] WARNING: predicted path publisher init failed\n");
        rcl_reset_error();
    }

    /* Initialize message buffers used by the executor. */
    nav_msgs__msg__Odometry__init(&g_odom_msg);
    geometry_msgs__msg__PoseWithCovarianceStamped__init(&g_pose_msg);
    std_msgs__msg__Float64__init(&g_imu_msg);
    std_msgs__msg__Float64__init(&g_servo_msg);
    ackermann_msgs__msg__AckermannDriveStamped__init(&g_drive_msg);

    {
        rcutils_allocator_t alloc = rcutils_get_default_allocator();

        char *drive_frame = (char *)alloc.allocate(16, alloc.state);
        if (drive_frame != NULL)
        {
            memcpy(drive_frame, "base_link", 10);
            g_drive_msg.header.frame_id.data = drive_frame;
            g_drive_msg.header.frame_id.size = 9;
            g_drive_msg.header.frame_id.capacity = 16;
        }

        char *odom_header = (char *)alloc.allocate(64, alloc.state);
        if (odom_header != NULL)
        {
            odom_header[0] = '\0';
            g_odom_msg.header.frame_id.data = odom_header;
            g_odom_msg.header.frame_id.size = 0;
            g_odom_msg.header.frame_id.capacity = 64;
        }

        char *odom_child = (char *)alloc.allocate(64, alloc.state);
        if (odom_child != NULL)
        {
            odom_child[0] = '\0';
            g_odom_msg.child_frame_id.data = odom_child;
            g_odom_msg.child_frame_id.size = 0;
            g_odom_msg.child_frame_id.capacity = 64;
        }

        char *pose_header = (char *)alloc.allocate(64, alloc.state);
        if (pose_header != NULL)
        {
            pose_header[0] = '\0';
            g_pose_msg.header.frame_id.data = pose_header;
            g_pose_msg.header.frame_id.size = 0;
            g_pose_msg.header.frame_id.capacity = 64;
        }
    }

    /* Pre-allocate raceline message for rclc deserialization */
    nav_msgs__msg__Path__init(&g_raceline_msg);
    {
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        char *fid = (char *)alloc.allocate(64, alloc.state);
        if (fid) { fid[0] = '\0'; g_raceline_msg.header.frame_id.data = fid;
                    g_raceline_msg.header.frame_id.size = 0;
                    g_raceline_msg.header.frame_id.capacity = 64; }
        geometry_msgs__msg__PoseStamped__Sequence__init(
            &g_raceline_msg.poses, MPCC_MAX_PATH_POINTS);
    }

    rclc_executor_t executor;
    if (rclc_executor_init(&executor,
                           &support.context,
                           MPCC_EXECUTOR_HANDLES,
                           &allocator) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: executor init failed\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &g_odom_sub,
            &g_odom_msg,
            &odom_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: add odom subscription failed\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &g_pose_sub,
            &g_pose_msg,
            &pose_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: add pose subscription failed\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &g_imu_sub,
            &g_imu_msg,
            &imu_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: add imu subscription failed\n");
        return 1;
    }

    if (g_servo_sub_ok)
    {
        if (rclc_executor_add_subscription(
                &executor,
                &g_servo_sub,
                &g_servo_msg,
                &servo_callback,
                ON_NEW_DATA) != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: add servo subscription failed (steering feedback disabled)\n");
            rcl_reset_error();
            g_servo_sub_ok = 0;
        }
    }

    if (g_raceline_sub_ok &&
        rclc_executor_add_subscription(
            &executor,
            &g_raceline_sub,
            &g_raceline_msg,
            &raceline_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] WARNING: add raceline subscription failed\n");
        rcl_reset_error();
        g_raceline_sub_ok = 0;
    }

    printf("[MPCC] Spinning hardware node...\n");
    rclc_executor_spin(&executor);

    /* Cleanup */
    {
        rcl_ret_t rc_cleanup;
        rc_cleanup = rcl_subscription_fini(&g_odom_sub, &node);
        rc_cleanup = rcl_subscription_fini(&g_pose_sub, &node);
        rc_cleanup = rcl_subscription_fini(&g_imu_sub, &node);
        rc_cleanup = rcl_subscription_fini(&g_servo_sub, &node);
        if (g_raceline_sub_ok)
        {
            rc_cleanup = rcl_subscription_fini(&g_raceline_sub, &node);
        }
        rc_cleanup = rcl_publisher_fini(&g_drive_pub, &node);
        rc_cleanup = rcl_publisher_fini(&g_predicted_path_pub, &node);
        rc_cleanup = rcl_publisher_fini(&g_raceline_pub, &node);
        (void)rc_cleanup;
    }

    rclc_executor_fini(&executor);
    {
        rcl_ret_t rc_cleanup = rcl_node_fini(&node);
        (void)rc_cleanup;
    }
    (void)rclc_support_fini(&support);

    return 0;
}
