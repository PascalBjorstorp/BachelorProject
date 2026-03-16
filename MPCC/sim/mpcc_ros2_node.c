/*******************************************************************************
 * mpcc_ros2_node.c — ROS2 Node for Lifted ODE MPCC
 *
 * Frenet primary (s,n,α) + Cartesian redundant (X,Y,ψ) formulation.
 * Subscribes to odometry, publishes drive commands.
 *
 * State [9]: s, n, alpha, vx, vy, omega, X, Y, psi
 * Control [3]: delta, a_x, v_theta
 ******************************************************************************/

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/pose_array.h>

#include <stdio.h>
#include <math.h>
#include <rcutils/allocator.h>

#include "mpcc_types.h"
#include "mpcc.h"
#include "mpc_types.h"        /* VehicleState_t */

/* ── Globals ─────────────────────────────────────────────────────────────── */
static VehicleState_t   current_vehicle_state;
static fixed_point_t    current_s;          /* arc-length hint for warm-start */
static int              state_valid = 0;

/* ROS objects */
static rcl_subscription_t   odom_sub;
static rcl_publisher_t      drive_pub;
static rcl_publisher_t      predicted_path_pub;
static rcl_publisher_t      raceline_pub;
static rcl_timer_t          control_timer;

static nav_msgs__msg__Odometry                          odom_msg;
static ackermann_msgs__msg__AckermannDriveStamped       drive_msg;
static geometry_msgs__msg__PoseArray                     pred_msg;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* Extract yaw from quaternion (z-up) */
static double quat_to_yaw(double qx, double qy, double qz, double qw)
{
    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    return atan2(siny, cosy);
}

/* ── Trajectory Loading ──────────────────────────────────────────────────── */

/**
 * Load reference path from CSV file into MPCCReferencePath_t.
 *
 * CSV format (TUM compatible):
 *   # s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2[,d_left_m,d_right_m]
 *
 * If bounds columns are missing, defaults to 0.5 m (narrow track).
 *
 * @param file_path  Path to the CSV trajectory file
 * @param path       Output: populated reference path
 * @return 1 on success, 0 on failure
 */
static int load_trajectory_csv(const char *file_path, MPCCReferencePath_t *path)
{
    FILE *f = fopen(file_path, "r");
    if (!f)
    {
        fprintf(stderr, "[MPCC] ERROR: Cannot open trajectory file: %s\n", file_path);
        return 0;
    }

    char line[512];
    path->num_points = 0;

    while (fgets(line, sizeof(line), f))
    {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        if (path->num_points >= MPCC_MAX_PATH_POINTS)
        {
            printf("[MPCC] WARNING: Path truncated at %d points\n",
                   MPCC_MAX_PATH_POINTS);
            break;
        }

        double s, x, y, psi, kappa, vx, ax;
        double d_left = 0.5, d_right = 0.5;
        int n = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &s, &x, &y, &psi, &kappa, &vx, &ax,
                        &d_left, &d_right);

        if (n < 5) continue;

        MPCCPathPoint_t *pt = &path->points[path->num_points];
        pt->s_ref       = float_to_fp((float)s);
        pt->x_ref       = float_to_fp((float)x);
        pt->y_ref       = float_to_fp((float)y);
        pt->phi_ref     = float_to_fp((float)psi);
        pt->kappa_ref   = float_to_fp((float)kappa);
        pt->vx_ref      = (n >= 6) ? float_to_fp((float)vx) : FP_CONST(3.0);
        /* Subtract car half-width so n bounds keep the body inside the track */
        #define CAR_HALF_WIDTH 0.155f  /* F1/10th ~0.31m wide */
        if (n >= 9) {
            float lb = (float)d_left  - CAR_HALF_WIDTH;
            float rb = (float)d_right - CAR_HALF_WIDTH;
            if (lb < 0.05f) lb = 0.05f; /* minimum 5cm */
            if (rb < 0.05f) rb = 0.05f;
            pt->left_bound  = float_to_fp(lb);
            pt->right_bound = float_to_fp(rb);
        } else {
            pt->left_bound  = FP_CONST(0.5);
            pt->right_bound = FP_CONST(0.5);
        }

        path->num_points++;
    }

    fclose(f);

    if (path->num_points < 2)
    {
        fprintf(stderr, "[MPCC] ERROR: Only %d points loaded from %s\n",
                path->num_points, file_path);
        return 0;
    }

    /* Set total length from last waypoint's s */
    path->total_length = path->points[path->num_points - 1].s_ref;
    path->is_closed = 1;

    printf("[MPCC] Loaded %d path points from %s (total length: %.1f m)\n",
           path->num_points, file_path, fp_to_float(path->total_length));

    return 1;
}

/* ── Publish Racing Line to rviz ──────────────────────────────────────── */
static void publish_raceline(const MPCCReferencePath_t *path)
{
    nav_msgs__msg__Path path_msg;
    nav_msgs__msg__Path__init(&path_msg);

    /* frame_id = "map" */
    rcutils_allocator_t alloc = rcutils_get_default_allocator();
    char *fid = (char *)alloc.allocate(8, alloc.state);
    if (fid) {
        memcpy(fid, "map", 4);
        path_msg.header.frame_id.data = fid;
        path_msg.header.frame_id.size = 3;
        path_msg.header.frame_id.capacity = 8;
    }

    /* Allocate poses array */
    uint16_t n = path->num_points;
    geometry_msgs__msg__PoseStamped *poses =
        (geometry_msgs__msg__PoseStamped *)alloc.allocate(
            n * sizeof(geometry_msgs__msg__PoseStamped), alloc.state);
    if (!poses) {
        fprintf(stderr, "[MPCC] WARNING: Could not allocate raceline poses\n");
        return;
    }

    for (uint16_t i = 0; i < n; i++) {
        geometry_msgs__msg__PoseStamped__init(&poses[i]);
        /* Allocate frame_id for each pose */
        char *pfid = (char *)alloc.allocate(8, alloc.state);
        if (pfid) {
            memcpy(pfid, "map", 4);
            poses[i].header.frame_id.data = pfid;
            poses[i].header.frame_id.size = 3;
            poses[i].header.frame_id.capacity = 8;
        }
        float x = fp_to_float(path->points[i].x_ref);
        float y = fp_to_float(path->points[i].y_ref);
        float phi = fp_to_float(path->points[i].phi_ref);
        poses[i].pose.position.x = x;
        poses[i].pose.position.y = y;
        poses[i].pose.position.z = 0.0;
        /* Quaternion from yaw */
        poses[i].pose.orientation.x = 0.0;
        poses[i].pose.orientation.y = 0.0;
        poses[i].pose.orientation.z = sin(phi * 0.5);
        poses[i].pose.orientation.w = cos(phi * 0.5);
    }

    path_msg.poses.data = poses;
    path_msg.poses.size = n;
    path_msg.poses.capacity = n;

    rcl_publish(&raceline_pub, &path_msg, NULL);
    printf("[MPCC] Published racing line (%d points) to /mpcc/raceline\n", n);

    /* Cleanup: free individual frame_id strings, then poses array */
    for (uint16_t i = 0; i < n; i++) {
        if (poses[i].header.frame_id.data)
            alloc.deallocate(poses[i].header.frame_id.data, alloc.state);
    }
    alloc.deallocate(poses, alloc.state);
    if (fid) alloc.deallocate(fid, alloc.state);
}

/* ── Odometry Callback ───────────────────────────────────────────────────── */
static void odom_callback(const void *msg_in)
{
    const nav_msgs__msg__Odometry *msg =
        (const nav_msgs__msg__Odometry *)msg_in;

    /* Cartesian pose */
    double X   = msg->pose.pose.position.x;
    double Y   = msg->pose.pose.position.y;
    double psi = quat_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    /* Body-frame velocities */
    double vx_body = msg->twist.twist.linear.x;
    double vy_body = msg->twist.twist.linear.y;
    double omega   = msg->twist.twist.angular.z;

    /* Pack into VehicleState (x, y, psi, vx, vy, omega) */
    current_vehicle_state.position_x_meters     = float_to_fp((float)X);
    current_vehicle_state.position_y_meters     = float_to_fp((float)Y);
    current_vehicle_state.heading_angle_radians = float_to_fp((float)psi);
    current_vehicle_state.longitudinal_velocity_meters_per_second = float_to_fp((float)vx_body);
    current_vehicle_state.lateral_velocity_meters_per_second      = float_to_fp((float)vy_body);
    current_vehicle_state.yaw_rate_radians_per_second             = float_to_fp((float)omega);

    state_valid = 1;
}

/* ── Control Timer Callback ──────────────────────────────────────────────── */
static uint32_t solve_count = 0;
#define STARTUP_RAMP_STEPS 40  /* 40 * 50ms = 2 seconds */

static void control_timer_callback(rcl_timer_t *timer, int64_t last_call)
{
    (void)timer;
    (void)last_call;

    if (!state_valid) return;

    /* Convert Cartesian vehicle state → MPCC Frenet state */
    MPCCState_t mpcc_state = mpcc_state_from_vehicle_state(
                                &current_vehicle_state, current_s);

    /* Update s hint for next iteration */
    current_s = mpcc_state.s;

    /* Solve MPCC */
    MPCCResult_t result;
    MPCCStatus_t status = mpcc_compute_control(&mpcc_state, &result);

    solve_count++;

    float a_x_cmd   = fp_to_float(result.optimal_control.a_x);
    float delta_cmd  = fp_to_float(result.optimal_control.delta);
    float v_theta_cmd = fp_to_float(result.optimal_control.v_theta);

    /* ── Startup ramp: gently increase control authority ────────────── */
    if (solve_count <= STARTUP_RAMP_STEPS) {
        float ramp = (float)solve_count / (float)STARTUP_RAMP_STEPS;
        /* Accel: ramp from 1.0 → 7.0 m/s² over 2 seconds */
        float max_ax = 1.0f + 6.0f * ramp;
        if (a_x_cmd >  max_ax) a_x_cmd =  max_ax;
        if (a_x_cmd < -max_ax) a_x_cmd = -max_ax;
        /* Steer: ramp from 0.1 → 0.43 rad over 2 seconds */
        float max_d = 0.1f + 0.33f * ramp;
        if (delta_cmd >  max_d) delta_cmd =  max_d;
        if (delta_cmd < -max_d) delta_cmd = -max_d;
    }

    /* ── If solver didn't converge, mpcc.c already falls back to the
     *    shifted warm-start control (last good plan). Just cap accel
     *    for safety — steering from the warm-start is trustworthy. ── */
    if (status != MPCC_STATUS_SUCCESS) {
        if (a_x_cmd >  3.0f) a_x_cmd =  3.0f;
        if (a_x_cmd < -3.0f) a_x_cmd = -3.0f;
    }

    /* ── Gentle correction: limit speed when far from raceline ────── */
    /* Only active until the car joins the raceline for the first time.
     * Prevents building dangerous momentum during initial transient.  */
    {
        static int joined_raceline = 0;
        float n_abs = fp_to_float(mpcc_state.n);
        if (n_abs < 0.0f) n_abs = -n_abs;

        if (n_abs < 0.15f) joined_raceline = 1;

        if (!joined_raceline && n_abs > 0.15f) {
            float vx_now = fp_to_float(mpcc_state.vx);
            /* Target max speed scales with proximity to raceline:
             *   |n| >= 0.5  →  max_vx = 1.0 m/s  (crawl)
             *   |n|  = 0.3  →  max_vx = 2.2 m/s
             *   |n| <= 0.15 →  no limit from here                    */
            float prox = 1.0f - n_abs / 0.5f;
            if (prox < 0.0f) prox = 0.0f;
            float max_vx = 1.0f + 3.0f * prox;

            if (vx_now > max_vx) {
                if (a_x_cmd > 0.0f) a_x_cmd = 0.0f;
            } else {
                if (a_x_cmd > 1.0f) a_x_cmd = 1.0f;
            }
        }
    }

    /* Minimum acceleration: prevent car from nearly stopping.
     * At very low speed the optimizer can't plan turns effectively. */
    {
        float vx_now = fp_to_float(mpcc_state.vx);
        if (vx_now < 1.5f && a_x_cmd < 0.5f)
            a_x_cmd = 0.5f;
    }

    /* Diagnostic: show state + ACTUAL commands sent (after ramp/clamp) */
    if (solve_count <= 20 || (solve_count % 10 == 0)) {
        fprintf(stderr,
            "[MPCC %3u] s=%.2f n=%.3f a=%.3f vx=%.2f | "
            "d=%.4f ax=%.3f vt=%.3f | st=%d it=%u\n",
            solve_count,
            fp_to_float(mpcc_state.s),
            fp_to_float(mpcc_state.n),
            fp_to_float(mpcc_state.alpha),
            fp_to_float(mpcc_state.vx),
            delta_cmd, a_x_cmd, v_theta_cmd,
            (int)status, result.admm_iterations);
        fflush(stderr);
    }

    /* Publish Ackermann drive command */
    drive_msg.header.stamp.sec     = 0;
    drive_msg.header.stamp.nanosec = 0;
    drive_msg.drive.steering_angle        = delta_cmd;
    drive_msg.drive.steering_angle_velocity = 0.0f;

    /* gym_bridge with control_input=['accl','steering_angle']
     * interprets drive.speed as acceleration command (m/s^2). */
    drive_msg.drive.speed         = a_x_cmd;
    drive_msg.drive.acceleration  = a_x_cmd;
    drive_msg.drive.jerk          = 0.0f;

    rcl_publish(&drive_pub, &drive_msg, NULL);
}

/* ── Main ────────────────────────────────────────────────────────────────── */
int main(int argc, const char *argv[])
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* Init */
    rclc_support_t support;
    rclc_support_init(&support, argc, argv, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "mpcc_node", "", &support);

    /* ── Initialize MPCC ─────────────────────────────────────────────── */
    mpcc_initialize();
    current_s = 0;

    /* Load reference trajectory from CSV */
    const char *trajectory_file = NULL;
    if (argc >= 2)
    {
        trajectory_file = argv[1];
    }
    else
    {
        trajectory_file = "/ros2_ws/src/f1tenth_planning/trajectories/Spielberg_raceline_optimized_wide.csv";
    }

    MPCCReferencePath_t ref_path;
    if (load_trajectory_csv(trajectory_file, &ref_path))
    {
        mpcc_set_reference_path(&ref_path);
        printf("[MPCC] Reference path set successfully\n");
    }
    else
    {
        fprintf(stderr, "[MPCC] FATAL: No reference path — controller will not work\n");
    }

    /* ── Publisher: raceline visualization (transient_local for rviz) ── */
    {
        rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
        pub_opts.qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        pub_opts.qos.depth = 1;
        pub_opts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        rcl_publisher_init(
            &raceline_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
            "/mpcc/raceline", &pub_opts);
    }

    /* Publish raceline once (transient_local keeps it available to rviz) */
    if (ref_path.num_points > 0)
        publish_raceline(&ref_path);

    /* ── Subscriber: odometry ────────────────────────────────────────── */
    rclc_subscription_init_default(
        &odom_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/ego_racecar/ground_truth");

    /* ── Publisher: drive command ─────────────────────────────────────── */
    rclc_publisher_init_default(
        &drive_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped),
        "/drive");

    /* Initialize drive message (must allocate header.frame_id string) */
    ackermann_msgs__msg__AckermannDriveStamped__init(&drive_msg);
    {
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        char *fid = (char *)alloc.allocate(16, alloc.state);
        if (fid) { fid[0] = '\0'; drive_msg.header.frame_id.data = fid;
                    drive_msg.header.frame_id.size = 0;
                    drive_msg.header.frame_id.capacity = 16; }
    }

    /* Initialize odom message (must allocate header string fields) */
    nav_msgs__msg__Odometry__init(&odom_msg);
    {
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        char *fid = (char *)alloc.allocate(64, alloc.state);
        if (fid) { fid[0] = '\0'; odom_msg.header.frame_id.data = fid;
                    odom_msg.header.frame_id.size = 0;
                    odom_msg.header.frame_id.capacity = 64; }
        char *cid = (char *)alloc.allocate(64, alloc.state);
        if (cid) { cid[0] = '\0'; odom_msg.child_frame_id.data = cid;
                    odom_msg.child_frame_id.size = 0;
                    odom_msg.child_frame_id.capacity = 64; }
    }

    /* ── Publisher: predicted path ────────────────────────────────────── */
    rclc_publisher_init_default(
        &predicted_path_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
        "/mpcc/predicted_path");

    /* ── Timer: control loop at dt interval ──────────────────────────── */
    const unsigned int timer_period_ms = 50;  /* 20 Hz default */
    rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(timer_period_ms),
        control_timer_callback);

    /* ── Executor ────────────────────────────────────────────────────── */
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &odom_sub, &odom_msg,
                                   &odom_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);

    /* Spin */
    rclc_executor_spin(&executor);

    /* Cleanup */
    rcl_subscription_fini(&odom_sub, &node);
    rcl_publisher_fini(&drive_pub, &node);
    rcl_publisher_fini(&predicted_path_pub, &node);
    rcl_publisher_fini(&raceline_pub, &node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
