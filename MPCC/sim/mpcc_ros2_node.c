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
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/pose_stamped.h>

#include <stdio.h>
#include <math.h>
#include <rcutils/allocator.h>

#include "mpcc_types.h"
#include "mpcc.h"
/* VehicleState_t is now defined in mpcc_types.h — no MPC dependency */

/* ── Globals ─────────────────────────────────────────────────────────────── */
static VehicleState_t   current_vehicle_state;
static float    current_s;          /* arc-length hint for warm-start */
static int              state_valid = 0;

/* ROS objects */
static rcl_subscription_t   odom_sub;
static rcl_publisher_t      drive_pub;
static rcl_publisher_t      predicted_path_pub;
static rcl_publisher_t      raceline_pub;
static rcl_timer_t          control_timer;

static nav_msgs__msg__Odometry                          odom_msg;
static ackermann_msgs__msg__AckermannDriveStamped       drive_msg;

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
        pt->s_ref       = (float)s;
        pt->x_ref       = (float)x;
        pt->y_ref       = (float)y;
        pt->phi_ref     = (float)psi;
        pt->kappa_ref   = (float)kappa;
        pt->vx_ref      = (n >= 6) ? (float)vx : 3.0f;
        /* Subtract car half-width so n bounds keep the body inside the track */
        #define CAR_HALF_WIDTH 0.155f  /* F1/10th ~0.31m wide */
        if (n >= 9) {
            float lb = (float)d_left  - CAR_HALF_WIDTH;
            float rb = (float)d_right - CAR_HALF_WIDTH;
            if (lb < 0.05f) lb = 0.05f; /* minimum 5cm */
            if (rb < 0.05f) rb = 0.05f;
            pt->left_bound  = lb;
            pt->right_bound = rb;
        } else {
            pt->left_bound  = 0.5f;
            pt->right_bound = 0.5f;
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
           path->num_points, file_path, path->total_length);

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
        float x = path->points[i].x_ref;
        float y = path->points[i].y_ref;
        float phi = path->points[i].phi_ref;
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

    {
        const rcl_ret_t rc = rcl_publish(&raceline_pub, &path_msg, NULL);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: failed to publish /mpcc/raceline: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
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
    current_vehicle_state.pos_x     = (float)X;
    current_vehicle_state.pos_y     = (float)Y;
    current_vehicle_state.heading = (float)psi;
    current_vehicle_state.long_vel = (float)vx_body;
    current_vehicle_state.lat_vel      = (float)vy_body;
    current_vehicle_state.yaw_rate             = (float)omega;

    state_valid = 1;
}

/* ── Raceline Subscription ────────────────────────────────────────────── */
static rcl_subscription_t raceline_sub;
static nav_msgs__msg__Path  raceline_msg;

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

    /* Extract x, y, heading, velocity from poses */
    for (uint16_t i = 0; i < n; i++) {
        const geometry_msgs__msg__PoseStamped *ps = &msg->poses.data[i];
        ref_path.points[i].x_ref  = (float)ps->pose.position.x;
        ref_path.points[i].y_ref  = (float)ps->pose.position.y;
        ref_path.points[i].vx_ref = (float)ps->pose.position.z; /* velocity in z */
        ref_path.points[i].phi_ref = (float)quat_to_yaw(
            ps->pose.orientation.x, ps->pose.orientation.y,
            ps->pose.orientation.z, ps->pose.orientation.w);
        ref_path.points[i].left_bound  = 0.5f;
        ref_path.points[i].right_bound = 0.5f;
    }

    /* Compute s from cumulative Euclidean distance */
    ref_path.points[0].s_ref = 0.0f;
    for (uint16_t i = 1; i < n; i++) {
        float dx = ref_path.points[i].x_ref - ref_path.points[i - 1].x_ref;
        float dy = ref_path.points[i].y_ref - ref_path.points[i - 1].y_ref;
        ref_path.points[i].s_ref = ref_path.points[i - 1].s_ref
                                   + sqrtf(dx * dx + dy * dy);
    }

    /* Curvature from heading central differences */
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
    ref_path.is_closed = 1;

    mpcc_set_reference_path(&ref_path);
    printf("[MPCC] Updated raceline from topic (%d points, %.1f m)\n",
           n, ref_path.total_length);

    /* Re-publish processed raceline for rviz */
    publish_raceline(&ref_path);
}

/* ── Control Timer Callback ──────────────────────────────────────────────── */
static uint32_t solve_count = 0;

/* Reusable predicted path message (pre-allocated) */
static geometry_msgs__msg__PoseArray predicted_path_msg;
static int predicted_path_msg_inited = 0;

static void publish_predicted_path(const MPCCResult_t *result)
{
    MPCCConfiguration_t cfg = mpcc_get_configuration();
    uint16_t n = cfg.horizon_steps + 1;  /* states[0..N] */
    if (n > MPCC_MAX_HORIZON + 1) n = MPCC_MAX_HORIZON + 1;

    if (!predicted_path_msg_inited) {
        geometry_msgs__msg__PoseArray__init(&predicted_path_msg);
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        /* frame_id = "map" */
        char *fid = (char *)alloc.allocate(8, alloc.state);
        if (fid) {
            memcpy(fid, "map", 4);
            predicted_path_msg.header.frame_id.data = fid;
            predicted_path_msg.header.frame_id.size = 3;
            predicted_path_msg.header.frame_id.capacity = 8;
        }
        /* Pre-allocate poses array for max horizon */
        geometry_msgs__msg__Pose *poses =
            (geometry_msgs__msg__Pose *)alloc.allocate(
                (MPCC_MAX_HORIZON + 1) * sizeof(geometry_msgs__msg__Pose),
                alloc.state);
        if (!poses) return;
        predicted_path_msg.poses.data = poses;
        predicted_path_msg.poses.capacity = MPCC_MAX_HORIZON + 1;
        predicted_path_msg_inited = 1;
    }

    predicted_path_msg.poses.size = n;
    for (uint16_t i = 0; i < n; i++) {
        const MPCCState_t *st = &result->predicted_states[i];
        predicted_path_msg.poses.data[i].position.x = st->X;
        predicted_path_msg.poses.data[i].position.y = st->Y;
        predicted_path_msg.poses.data[i].position.z = 0.0;
        predicted_path_msg.poses.data[i].orientation.x = 0.0;
        predicted_path_msg.poses.data[i].orientation.y = 0.0;
        predicted_path_msg.poses.data[i].orientation.z = sin(st->psi * 0.5f);
        predicted_path_msg.poses.data[i].orientation.w = cos(st->psi * 0.5f);
    }

    { rcl_ret_t rc_ = rcl_publish(&predicted_path_pub, &predicted_path_msg, NULL); (void)rc_; }
}

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

    float a_x_cmd   = result.optimal_control.a_x;
    float delta_cmd  = result.optimal_control.delta;
    float v_theta_cmd = result.optimal_control.v_theta;

    if (!isfinite((double)delta_cmd) || !isfinite((double)a_x_cmd)) {
        fprintf(stderr,
            "[MPCC] WARNING: non-finite control (delta=%.6f ax=%.6f), forcing safe brake\n",
            (double)delta_cmd, (double)a_x_cmd);
        delta_cmd = 0.0f;
        a_x_cmd = -7.0f;
    }

    if (delta_cmd > 0.4189f) delta_cmd = 0.4189f;
    if (delta_cmd < -0.4189f) delta_cmd = -0.4189f;
    if (a_x_cmd > 7.31f) a_x_cmd = 7.31f;
    if (a_x_cmd < -7.31f) a_x_cmd = -7.31f;

    /* Diagnostic: show state + actual commands sent */
    if (solve_count <= 20 || (solve_count % 10 == 0)) {
        fprintf(stderr,
            "[MPCC %3u] s=%.2f vx=%.2f X=%.2f Y=%.2f psi=%.3f | "
            "d=%.4f ax=%.3f vt=%.3f | st=%d it=%u\n",
            solve_count,
            mpcc_state.s,
            mpcc_state.vx,
            mpcc_state.X,
            mpcc_state.Y,
            mpcc_state.psi,
            delta_cmd, a_x_cmd, v_theta_cmd,
            (int)status, result.admm_iterations);
        fflush(stderr);
    }

    /* Publish Ackermann drive command */
    drive_msg.header.stamp.sec     = 0;
    drive_msg.header.stamp.nanosec = 0;
    drive_msg.drive.steering_angle        = delta_cmd;
    drive_msg.drive.steering_angle_velocity = 0.0f;

    /* Keep exact MPC sim behavior: gym_bridge control_input=['accl','steering_angle']
     * reads drive.speed as acceleration command. */
    drive_msg.drive.speed         = a_x_cmd;
    drive_msg.drive.acceleration  = a_x_cmd;
    drive_msg.drive.jerk          = 0.0f;

    {
        const rcl_ret_t rc = rcl_publish(&drive_pub, &drive_msg, NULL);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: failed to publish /drive: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }

    /* Publish predicted horizon path for visualization */
    if (status == MPCC_STATUS_SUCCESS || status == MPCC_STATUS_MAX_ITERATIONS) {
        publish_predicted_path(&result);
    }
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

    /* ── Initialize MPCC (with env var overrides for tuning) ─────────── */
    {
        /* Start from defaults, then override from env vars if present */
        mpcc_initialize();
        MPCCConfiguration_t cfg = mpcc_get_configuration();

        const char *v;
        if ((v = getenv("Q_CONTOURING")))    cfg.weight_contouring = (float)atof(v);
        if ((v = getenv("Q_LAG")))           cfg.weight_lag        = (float)atof(v);
        if ((v = getenv("Q_PROGRESS")))      cfg.weight_progress   = (float)atof(v);
        if ((v = getenv("Q_VX")))            cfg.weight_vx         = (float)atof(v);
        if ((v = getenv("VX_REF")))          cfg.vx_ref            = (float)atof(v);
        if ((v = getenv("Q_VY")))            cfg.weight_vy         = (float)atof(v);
        if ((v = getenv("Q_OMEGA")))         cfg.weight_omega      = (float)atof(v);
        if ((v = getenv("R_DELTA")))         cfg.weight_delta      = (float)atof(v);
        if ((v = getenv("R_AX")))            cfg.weight_ax         = (float)atof(v);
        if ((v = getenv("R_VTHETA")))        cfg.weight_v_theta    = (float)atof(v);
        if ((v = getenv("W_DELTA_RATE")))    cfg.weight_delta_rate = (float)atof(v);
        if ((v = getenv("W_AX_RATE")))       cfg.weight_ax_rate    = (float)atof(v);
        if ((v = getenv("W_VTHETA_RATE")))   cfg.weight_v_theta_rate = (float)atof(v);
        if ((v = getenv("Q_CONTOURING_TERM"))) cfg.weight_contouring_terminal = (float)atof(v);
        if ((v = getenv("Q_LAG_TERM")))      cfg.weight_lag_terminal = (float)atof(v);
        if ((v = getenv("Q_PROGRESS_TERM"))) cfg.weight_progress_terminal = (float)atof(v);
        if ((v = getenv("ADMM_RHO")))        cfg.admm_rho          = (float)atof(v);
        if ((v = getenv("ADMM_MAX_ITER")))   cfg.admm_max_iterations = (uint16_t)atoi(v);
        if ((v = getenv("ADMM_TOL")))        cfg.admm_tolerance    = (float)atof(v);
        if ((v = getenv("HORIZON")))         cfg.horizon_steps     = (uint16_t)atoi(v);
        if ((v = getenv("DT")))              cfg.dt                = (float)atof(v);
        if ((v = getenv("V_THETA_MAX")))     cfg.v_theta_max       = (float)atof(v);
        if ((v = getenv("V_THETA_MIN")))     cfg.v_theta_min       = (float)atof(v);
        if ((v = getenv("MU")))              cfg.mu                = (float)atof(v);
        if ((v = getenv("C_SF")))            cfg.C_Sf              = (float)atof(v);
        if ((v = getenv("C_SR")))            cfg.C_Sr              = (float)atof(v);
        if ((v = getenv("AX_MAX")))          cfg.ax_max            = (float)atof(v);
        if ((v = getenv("AX_MIN")))          cfg.ax_min            = (float)atof(v);

        /* Apply the possibly-modified config */
        mpcc_set_configuration(&cfg);

        printf("[MPCC] Config: N=%d dt=%.3f Q_c=%.1f Q_l=%.1f "
               "Q_prog=%.1f R_delta=%.2f W_drate=%.1f ADMM_rho=%.2f\n",
               cfg.horizon_steps, cfg.dt,
               cfg.weight_contouring, cfg.weight_lag,
               cfg.weight_progress,
               cfg.weight_delta, cfg.weight_delta_rate,
               cfg.admm_rho);
    }
    current_s = 0;

    /* Load reference trajectory from CSV */
    const char *trajectory_file = NULL;
    if (argc >= 2)
    {
        trajectory_file = argv[1];
    }
    else
    {
        trajectory_file = "/ros2_ws/src/f1tenth_planning/trajectories/Spielberg_raceline.csv";
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
        rcl_ret_t rc = rcl_publisher_init(
            &raceline_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
            "/mpcc/raceline", &pub_opts);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: failed to init /mpcc/raceline publisher: %s\n",
                rcl_get_error_string().str);
            rcl_reset_error();
        }
    }

    /* Publish raceline once (transient_local keeps it available to rviz) */
    if (ref_path.num_points > 0)
        publish_raceline(&ref_path);

    /* ── Subscriber: odometry ────────────────────────────────────────── */
    rclc_subscription_init_default(
        &odom_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/ego_racecar/ground_truth");

    /* ── Subscriber: raceline updates ────────────────────────────────── */
    rclc_subscription_init_default(
        &raceline_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
        "/local_raceline");

    /* Initialize raceline message (pre-allocate poses for rclc) */
    nav_msgs__msg__Path__init(&raceline_msg);
    {
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        char *fid = (char *)alloc.allocate(64, alloc.state);
        if (fid) { fid[0] = '\0'; raceline_msg.header.frame_id.data = fid;
                    raceline_msg.header.frame_id.size = 0;
                    raceline_msg.header.frame_id.capacity = 64; }
        /* Pre-allocate poses array (each PoseStamped + its strings) */
        geometry_msgs__msg__PoseStamped__Sequence__init(
            &raceline_msg.poses, MPCC_MAX_PATH_POINTS);
    }

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
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &odom_sub, &odom_msg,
                                   &odom_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &raceline_sub, &raceline_msg,
                                   &raceline_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);

    /* Spin */
    rclc_executor_spin(&executor);

    /* Cleanup */
    {
        rcl_ret_t rc = rcl_subscription_fini(&odom_sub, &node);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_subscription_fini failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    {
        rcl_ret_t rc = rcl_subscription_fini(&raceline_sub, &node);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_subscription_fini(raceline) failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    {
        rcl_ret_t rc = rcl_publisher_fini(&drive_pub, &node);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_publisher_fini(drive) failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    {
        rcl_ret_t rc = rcl_publisher_fini(&predicted_path_pub, &node);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_publisher_fini(predicted_path) failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    {
        rcl_ret_t rc = rcl_publisher_fini(&raceline_pub, &node);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_publisher_fini(raceline) failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    {
        rcl_ret_t rc = rcl_timer_fini(&control_timer);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_timer_fini failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    rclc_executor_fini(&executor);
    {
        rcl_ret_t rc = rcl_node_fini(&node);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "[MPCC] WARNING: rcl_node_fini failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }
    rclc_support_fini(&support);

    return 0;
}
