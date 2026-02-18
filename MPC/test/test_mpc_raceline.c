/**
 * @file test_mpc_raceline.c
 * @brief Comprehensive Offline MPC Simulation using Spielberg Raceline
 *
 * Runs the full MPC closed-loop controller on the Spielberg raceline,
 * simulating the same code path as the ROS2 node but without ROS2.
 * Detects anomalies that could cause "random turning" behaviour:
 *
 *   - Heading wrap-around at ±π boundary
 *   - Sudden steering direction reversals
 *   - Steering at saturation limits
 *   - QP solver failures / max-iterations
 *   - Fixed-point overflow in Phi / Hessian computation
 *   - Large heading tracking errors
 *   - Linearization issues at specific operating points
 *
 * Compile:
 *   gcc -Wall -Wextra -O2 -I../include -o test_mpc_raceline.exe \
 *       test_mpc_raceline.c ../src/fp_math.c ../src/vehicle_model.c \
 *       ../src/qp_solver.c ../src/mpc.c -lm
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mpc.h"
#include "mpc_types.h"
#include "fp_math.h"
#include "vehicle_model.h"

/*===========================================================================
 * Test Framework (Minimal)
 *===========================================================================*/
static int tests_passed = 0;
static int tests_failed = 0;

static void check_condition(const char *name, int condition)
{
    if (condition) {
        tests_passed++;
    } else {
        tests_failed++;
        printf("  [FAIL] %s\n", name);
    }
}

/*===========================================================================
 * Raceline Data Structures
 *===========================================================================*/

/** Maximum number of waypoints */
#define MAX_WAYPOINTS 2000

/** MPC prediction horizon (mirroring ROS2 node) */
#define HORIZON 10

/** Time step (seconds) — 5ms = 200 Hz */
#define DT 0.005

/** Average waypoint spacing from Spielberg trajectory */
#define AVG_WAYPOINT_SPACING 0.346

/** Max velocity for reference trajectory */
#define MAX_REF_VELOCITY 20.0

/** Waypoint structure (stored as double for reference, converted to FP for MPC) */
typedef struct {
    double s;       /* arc length [m] */
    double x;       /* x position [m] */
    double y;       /* y position [m] */
    double psi;     /* heading [rad] */
    double kappa;   /* curvature [1/m] */
    double vx;      /* velocity [m/s] */
    double ax;      /* acceleration [m/s²] */
} Waypoint_t;

static Waypoint_t raceline[MAX_WAYPOINTS];
static int raceline_count = 0;

/*===========================================================================
 * Anomaly Detection Structures
 *===========================================================================*/

/** Types of anomalies to detect */
typedef enum {
    ANOMALY_STEERING_REVERSAL,      /* Steering flips sign by >15° in 1 step */
    ANOMALY_STEERING_SATURATION,    /* Steering hits ±0.4189 limit */
    ANOMALY_HEADING_WRAP,           /* Heading difference > π between steps */
    ANOMALY_LARGE_HEADING_ERROR,    /* Heading error > 0.5 rad */
    ANOMALY_SOLVER_FAILURE,         /* QP solver failed */
    ANOMALY_SOLVER_MAX_ITER,        /* QP solver hit max iterations */
    ANOMALY_LARGE_LATERAL_ERROR,    /* >1.5m from raceline */
    ANOMALY_FP_OVERFLOW_SUSPECT,    /* Control output jumps suspiciously */
    ANOMALY_TYPE_COUNT
} AnomalyType_t;

static const char *anomaly_names[] = {
    "Steering Reversal (>15°/step)",
    "Steering Saturation",
    "Heading Wrap-Around",
    "Large Heading Error (>0.5rad)",
    "Solver Failure",
    "Solver Max Iterations",
    "Large Lateral Error (>1.5m)",
    "FP Overflow Suspect"
};

#define MAX_ANOMALY_LOG 200

typedef struct {
    int step;
    int waypoint_index;
    AnomalyType_t type;
    double value;         /* Associated numeric value */
    double x, y, heading; /* Vehicle state at anomaly */
    double steering;      /* Steering command */
} AnomalyRecord_t;

static AnomalyRecord_t anomaly_log[MAX_ANOMALY_LOG];
static int anomaly_counts[ANOMALY_TYPE_COUNT] = {0};
static int total_anomalies = 0;

static void log_anomaly(int step, int wp_idx, AnomalyType_t type, double value,
                         double x, double y, double heading, double steering)
{
    anomaly_counts[type]++;
    if (total_anomalies < MAX_ANOMALY_LOG) {
        AnomalyRecord_t *r = &anomaly_log[total_anomalies];
        r->step = step;
        r->waypoint_index = wp_idx;
        r->type = type;
        r->value = value;
        r->x = x;
        r->y = y;
        r->heading = heading;
        r->steering = steering;
        total_anomalies++;
    }
}

/*===========================================================================
 * CSV Loading
 *===========================================================================*/

static int load_raceline(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        /* Try relative paths */
        const char *alt_paths[] = {
            "../../f1tenth_planning/trajectories/Spielberg_raceline.csv",
            "../../../f1tenth_planning/trajectories/Spielberg_raceline.csv",
            "f1tenth_planning/trajectories/Spielberg_raceline.csv",
            NULL
        };
        for (int i = 0; alt_paths[i] != NULL; i++) {
            f = fopen(alt_paths[i], "r");
            if (f) {
                printf("[LOAD] Opened: %s\n", alt_paths[i]);
                break;
            }
        }
        if (!f) {
            fprintf(stderr, "[LOAD] ERROR: Cannot open raceline CSV: %s\n", path);
            return 0;
        }
    } else {
        printf("[LOAD] Opened: %s\n", path);
    }

    char buf[512];
    raceline_count = 0;

    while (fgets(buf, sizeof(buf), f)) {
        if (buf[0] == '#' || buf[0] == '\n' || buf[0] == '\r') continue;
        if (raceline_count >= MAX_WAYPOINTS) break;

        Waypoint_t *wp = &raceline[raceline_count];
        int n = sscanf(buf, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                       &wp->s, &wp->x, &wp->y, &wp->psi,
                       &wp->kappa, &wp->vx, &wp->ax);
        if (n >= 6) {
            raceline_count++;
        }
    }
    fclose(f);
    printf("[LOAD] Loaded %d waypoints\n", raceline_count);
    return raceline_count > 0;
}

/*===========================================================================
 * Closest Waypoint Search (mirrors ROS2 node logic)
 *===========================================================================*/

static int last_closest = 0;

static int find_closest_waypoint(double px, double py, double heading)
{
    if (raceline_count == 0) return 0;

    int window = raceline_count / 4;
    if (window < 20) window = 20;

    int best = last_closest;
    double best_dist = 1e18;

    double dir_x = cos(heading);
    double dir_y = sin(heading);

    for (int off = -window; off <= window; off++) {
        int idx = (last_closest + off) % raceline_count;
        if (idx < 0) idx += raceline_count;

        double dx = raceline[idx].x - px;
        double dy = raceline[idx].y - py;
        double d2 = dx*dx + dy*dy;

        double dot = dx * dir_x + dy * dir_y;
        if (dot < -0.5 && d2 > 0.25) continue;

        if (d2 < best_dist) {
            best_dist = d2;
            best = idx;
        }
    }
    last_closest = best;
    return best;
}

/*===========================================================================
 * Reference Trajectory Builder (mirrors ROS2 node logic)
 *===========================================================================*/

static void build_reference(int closest_idx, double vehicle_heading,
                             TrajectoryReferencePoint_t *ref)
{
    double prev_heading = vehicle_heading;

    for (int step = 0; step < HORIZON; step++) {
        int base_idx = (closest_idx + step) % raceline_count;
        double ref_vel = raceline[base_idx].vx;
        if (ref_vel < 3.0) ref_vel = 3.0;
        if (ref_vel > MAX_REF_VELOCITY) ref_vel = MAX_REF_VELOCITY;

        double expected_dist = ref_vel * DT * (step + 1);
        int wp_ahead = (int)(expected_dist / AVG_WAYPOINT_SPACING);
        if (wp_ahead < step + 1) wp_ahead = step + 1;

        int wp_idx = (closest_idx + wp_ahead) % raceline_count;

        ref[step].reference_position_x_meters = DOUBLE_TO_FP(raceline[wp_idx].x);
        ref[step].reference_position_y_meters = DOUBLE_TO_FP(raceline[wp_idx].y);

        /* Unwrap heading to maintain continuity across ±π boundary */
        double ref_heading = raceline[wp_idx].psi;
        double delta = ref_heading - prev_heading;
        while (delta > M_PI) { ref_heading -= 2.0 * M_PI; delta = ref_heading - prev_heading; }
        while (delta < -M_PI) { ref_heading += 2.0 * M_PI; delta = ref_heading - prev_heading; }
        prev_heading = ref_heading;

        ref[step].reference_heading_radians = DOUBLE_TO_FP(ref_heading);
        ref[step].reference_velocity_meters_per_second =
            DOUBLE_TO_FP(raceline[base_idx].vx);
        ref[step].reference_lateral_velocity_meters_per_second = 0;
        ref[step].reference_yaw_rate_radians_per_second = 0;
    }
}

/*===========================================================================
 * Angle wrapping helper
 *===========================================================================*/

static double wrap_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/*===========================================================================
 * Simulation-Matched Velocity Helper
 *===========================================================================
 *
 * In the ROS2 simulation, the velocity command sent to the car comes
 * directly from the trajectory CSV — NOT from the MPC velocity output.
 * The MPC's velocity output is effectively ignored; only steering is used.
 *
 * Additionally, the ROS2 node reduces velocity when the car drifts off-track:
 *   - >1.0m from trajectory: velocity *= 0.5
 *   - >0.5m from trajectory: velocity *= 0.8
 *
 * This function replicates that exact behavior for test accuracy.
 */
static double get_sim_velocity(int closest_idx, double px, double py)
{
    double vel = raceline[closest_idx].vx;

    /* Clamp to valid range (same as ROS2 node) */
    if (vel < 3.0) vel = 3.0;
    if (vel > MAX_REF_VELOCITY) vel = MAX_REF_VELOCITY;

    /* Distance-based speed reduction (mirrors mpc_ros2_node.c) */
    double dx = px - raceline[closest_idx].x;
    double dy = py - raceline[closest_idx].y;
    double distance_from_trajectory = sqrt(dx * dx + dy * dy);

    if (distance_from_trajectory > 1.0) {
        vel *= 0.5;  /* More than 1m off-track */
    } else if (distance_from_trajectory > 0.5) {
        vel *= 0.8;  /* 0.5–1.0m off-track */
    }

    return vel;
}

/*===========================================================================
 * Print MPC Configuration (for test diagnostics)
 *===========================================================================*/
static void print_mpc_configuration(void)
{
    MpcConfiguration_t config = mpc_get_configuration();
    printf("\n  === MPC Configuration (active weights) ===\n");
    printf("  Horizon:            %d steps\n", config.prediction_horizon_steps);
    printf("  Time step:          %.4f s\n", FP_TO_DOUBLE(config.time_step_seconds));
    printf("  Weight position X:  %.4f\n", FP_TO_DOUBLE(config.weight_position_x));
    printf("  Weight position Y:  %.4f\n", FP_TO_DOUBLE(config.weight_position_y));
    printf("  Weight heading:     %.4f\n", FP_TO_DOUBLE(config.weight_heading));
    printf("  Weight velocity:    %.4f\n", FP_TO_DOUBLE(config.weight_velocity));
    printf("  Weight steer effort:%.4f\n", FP_TO_DOUBLE(config.weight_steering_effort));
    printf("  Weight steer rate:  %.4f\n", FP_TO_DOUBLE(config.weight_steering_rate));
    printf("  Weight vel effort:  %.4f\n", FP_TO_DOUBLE(config.weight_force_effort));
    printf("  Weight vel rate:    %.4f\n", FP_TO_DOUBLE(config.weight_force_rate));
    printf("  Max solver iters:   %d\n", config.maximum_solver_iterations);
    printf("  Position tracking:  %s\n",
           (config.weight_position_x == 0 && config.weight_position_y == 0) ?
           "DISABLED (heading-only)" : "ENABLED");
    printf("\n");
}

/*===========================================================================
 * TEST 1: Full Spielberg Raceline Closed-Loop Simulation
 *===========================================================================
 *
 * Starts the car at waypoint 0 and runs the full MPC loop for the entire
 * track length (~1000 steps). Detects every category of anomaly.
 */
static void test_full_raceline_simulation(void)
{
    printf("\n[TEST] Full Spielberg Raceline Simulation (Sim-Matched)\n");
    printf("  Horizon=%d, dt=%.3fs, waypoints=%d\n", HORIZON, DT, raceline_count);
    printf("  NOTE: Using trajectory velocity for vehicle propagation (matches ROS2 node)\n");
    printf("        MPC velocity output is ignored — only steering is used.\n");
    printf("        Distance-based speed reduction: >1.0m→50%%, >0.5m→80%%\n");

    /* Print active configuration */
    mpc_initialize();
    print_mpc_configuration();

    /* Reset anomaly tracking */
    memset(anomaly_counts, 0, sizeof(anomaly_counts));
    total_anomalies = 0;
    last_closest = 0;

    mpc_reset();

    /* Start at first waypoint */
    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(raceline[0].x);
    state.position_y_meters = DOUBLE_TO_FP(raceline[0].y);
    state.heading_angle_radians = DOUBLE_TO_FP(raceline[0].psi);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(raceline[0].vx);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    double prev_steering = 0.0;
    int success_count = 0;
    int total_steps = 0;
    double max_heading_error = 0.0;
    double max_lateral_error = 0.0;
    double max_steering_change = 0.0;
    double sum_lateral_error = 0.0;
    double sum_heading_error = 0.0;

    /* Run for enough steps to cover whole track (~1000 waypoints, at ~0.35m each
       = 346m total. At avg ~12 m/s, that's ~29s. At dt=0.05, ~580 steps.
       Run extra for safety. */
    int sim_steps = 1500;

    for (int step = 0; step < sim_steps; step++) {
        double px = FP_TO_DOUBLE(state.position_x_meters);
        double py = FP_TO_DOUBLE(state.position_y_meters);
        double psi = FP_TO_DOUBLE(state.heading_angle_radians);
        double vel = FP_TO_DOUBLE(state.longitudinal_velocity_meters_per_second);

        /* Find closest waypoint */
        int closest = find_closest_waypoint(px, py, psi);

        /* Build reference trajectory */
        TrajectoryReferencePoint_t ref[HORIZON];
        build_reference(closest, psi, ref);

        /* Run MPC */
        MpcSolverResult_t result;
        MpcSolverStatus_t status = mpc_compute_optimal_control(&state, ref, &result);

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);

        total_steps++;

        /* === Anomaly Detection === */

        /* 1. Solver status */
        if (status == MPC_STATUS_SUCCESS) {
            success_count++;
        } else if (status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED) {
            success_count++;
            log_anomaly(step, closest, ANOMALY_SOLVER_MAX_ITER,
                        (double)result.iterations_used, px, py, psi, steer);
        } else {
            log_anomaly(step, closest, ANOMALY_SOLVER_FAILURE,
                        (double)status, px, py, psi, steer);
        }

        /* 2. Steering reversal */
        double steer_change = steer - prev_steering;
        if (fabs(steer_change) > fabs(max_steering_change))
            max_steering_change = steer_change;

        if (fabs(steer_change) > 0.26) { /* >15° per step */
            log_anomaly(step, closest, ANOMALY_STEERING_REVERSAL,
                        steer_change, px, py, psi, steer);
        }

        /* 3. Steering saturation */
        if (fabs(steer) > 0.41) {
            log_anomaly(step, closest, ANOMALY_STEERING_SATURATION,
                        steer, px, py, psi, steer);
        }

        /* 4. Heading error */
        double ref_heading = raceline[closest].psi;
        double heading_err = wrap_angle(psi - ref_heading);
        if (fabs(heading_err) > fabs(max_heading_error))
            max_heading_error = heading_err;
        sum_heading_error += fabs(heading_err);

        if (fabs(heading_err) > 0.5) {
            log_anomaly(step, closest, ANOMALY_LARGE_HEADING_ERROR,
                        heading_err, px, py, psi, steer);
        }

        /* 5. Lateral error */
        double dx = px - raceline[closest].x;
        double dy = py - raceline[closest].y;
        double lat_err = sqrt(dx*dx + dy*dy);
        if (lat_err > max_lateral_error) max_lateral_error = lat_err;
        sum_lateral_error += lat_err;

        if (lat_err > 1.5) {
            log_anomaly(step, closest, ANOMALY_LARGE_LATERAL_ERROR,
                        lat_err, px, py, psi, steer);
        }

        /* 6. Heading wrap-around in vehicle state */
        if (step > 0) {
            if (closest > 0) {
                int prev_close = (closest - 1 + raceline_count) % raceline_count;
                double ref_delta = raceline[closest].psi - raceline[prev_close].psi;
                if (fabs(ref_delta) > 3.0) {
                    log_anomaly(step, closest, ANOMALY_HEADING_WRAP,
                                ref_delta, px, py, psi, steer);
                }
            }
        }

        /* 7. Suspicious output (likely FP overflow) */
        if (fabs(steer) > 0.41 && fabs(steer_change) > 0.3 &&
            fabs(heading_err) < 0.1) {
            log_anomaly(step, closest, ANOMALY_FP_OVERFLOW_SUSPECT,
                        steer_change, px, py, psi, steer);
        }

        prev_steering = steer;

        /* === Sim-matched vehicle propagation ===
         * In the ROS2 simulation, velocity comes from the trajectory CSV
         * (with distance-based reduction), NOT from MPC velocity output.
         * Only MPC steering is used as the control command. */
        double sim_vel = get_sim_velocity(closest, px, py);
        ControlInput_t sim_control;
        sim_control.steering_angle_radians = result.optimal_control.steering_angle_radians;
        sim_control.longitudinal_force_newtons = DOUBLE_TO_FP(sim_vel);
        state = vehicle_model_predict_next_state(&state, &sim_control, DOUBLE_TO_FP(DT));

        /* Progress report every 300 steps */
        if (step % 300 == 0 || step == sim_steps - 1) {
            printf("  Step %4d: pos=(%.1f,%.1f) hdg=%.2f vel=%.1f(cmd=%.1f) steer=%.3f wp=%d lat_err=%.2f\n",
                   step, px, py, psi, vel, sim_vel, steer, closest, lat_err);
        }
    }

    /* Print summary */
    printf("\n  === Simulation Summary ===\n");
    printf("  Total steps:         %d\n", total_steps);
    printf("  Solver success:      %d (%.1f%%)\n",
           success_count, 100.0 * success_count / total_steps);
    printf("  Max heading error:   %.4f rad (%.1f°)\n",
           max_heading_error, max_heading_error * 180.0 / M_PI);
    printf("  Avg heading error:   %.4f rad (%.1f°)\n",
           sum_heading_error / total_steps,
           (sum_heading_error / total_steps) * 180.0 / M_PI);
    printf("  Max lateral error:   %.4f m\n", max_lateral_error);
    printf("  Avg lateral error:   %.4f m\n", sum_lateral_error / total_steps);
    printf("  Max steering change: %.4f rad/step (%.1f°/step)\n",
           max_steering_change, max_steering_change * 180.0 / M_PI);

    printf("\n  === Anomaly Counts ===\n");
    int critical_anomalies = 0;
    for (int i = 0; i < ANOMALY_TYPE_COUNT; i++) {
        printf("  %-40s: %d\n", anomaly_names[i], anomaly_counts[i]);
        if (i == ANOMALY_STEERING_REVERSAL || i == ANOMALY_SOLVER_FAILURE ||
            i == ANOMALY_FP_OVERFLOW_SUSPECT || i == ANOMALY_LARGE_HEADING_ERROR)
            critical_anomalies += anomaly_counts[i];
    }

    /* Print first few anomalies of each type for debugging */
    printf("\n  === First Anomalies (up to 5 per type) ===\n");
    for (int t = 0; t < (int)ANOMALY_TYPE_COUNT; t++) {
        int printed = 0;
        for (int i = 0; i < total_anomalies && printed < 5; i++) {
            if ((int)anomaly_log[i].type == t) {
                AnomalyRecord_t *r = &anomaly_log[i];
                printf("  [%s] step=%d wp=%d val=%.4f pos=(%.1f,%.1f) hdg=%.2f steer=%.3f\n",
                       anomaly_names[t], r->step, r->waypoint_index,
                       r->value, r->x, r->y, r->heading, r->steering);
                printed++;
            }
        }
    }

    /* Assertions
     *
     * Thresholds reflect realistic MPC performance with heading-only tracking
     * (position weights disabled). Without position tracking, lateral drift
     * is expected; the MPC tracks heading which indirectly follows the path.
     *
     * These thresholds match the ROS2 simulation behavior:
     * - Velocity from trajectory CSV (not MPC output)
     * - Distance-based speed reduction when off-track
     */
    check_condition("Solver success rate >= 95%",
                    success_count >= total_steps * 95 / 100);
    check_condition("No FP overflow suspects",
                    anomaly_counts[ANOMALY_FP_OVERFLOW_SUSPECT] == 0);
    check_condition("No solver failures",
                    anomaly_counts[ANOMALY_SOLVER_FAILURE] == 0);
    check_condition("Max heading error < 1.5 rad",
                    fabs(max_heading_error) < 1.5);
    check_condition("Avg heading error < 0.3 rad",
                    (sum_heading_error / total_steps) < 0.3);
    check_condition("Max lateral error < 5.0 m",
                    max_lateral_error < 5.0);
    check_condition("Avg lateral error < 2.0 m",
                    (sum_lateral_error / total_steps) < 2.0);
    check_condition("Few steering reversals (<100)",
                    anomaly_counts[ANOMALY_STEERING_REVERSAL] < 100);
}

/*===========================================================================
 * TEST 2: Heading Wrap-Around Stress Test
 *===========================================================================
 *
 * Specifically tests the MPC at the 3 ±π boundary crossings found on
 * the Spielberg track. The car is placed exactly at the crossing point
 * with heading near ±π and the reference crosses the boundary.
 *
 * This is the most likely cause of "random turning" — when the heading
 * error computation sees a jump from -π to +π (or vice versa), it
 * produces a ~2π error, causing the MPC to command maximum steering.
 */
static void test_heading_wrap_crossings(void)
{
    printf("\n[TEST] Heading Wrap-Around Crossings (3 locations)\n");

    /* The 3 wrap-around locations from the Spielberg raceline:
     * idx 228→229:  heading -3.132 → 3.137  (turning clockwise through -π/+π)
     * idx 273→274:  heading  3.139 → -3.102  (turning counter-clockwise)
     * idx 758→759:  heading -3.121 → 3.062  (turning clockwise)
     */
    int wrap_indices[] = {225, 270, 755};
    const char *wrap_names[] = {
        "Wrap @ idx 229 (hdg -3.13→+3.14)",
        "Wrap @ idx 274 (hdg +3.14→-3.10)",
        "Wrap @ idx 759 (hdg -3.12→+3.06)"
    };

    for (int w = 0; w < 3; w++) {
        printf("\n  --- %s ---\n", wrap_names[w]);

        int start_idx = wrap_indices[w];
        if (start_idx >= raceline_count) continue;

        mpc_initialize();
        mpc_reset();
        last_closest = start_idx;

        /* Start the car at this waypoint */
        VehicleState_t state;
        state.position_x_meters = DOUBLE_TO_FP(raceline[start_idx].x);
        state.position_y_meters = DOUBLE_TO_FP(raceline[start_idx].y);
        state.heading_angle_radians = DOUBLE_TO_FP(raceline[start_idx].psi);
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(raceline[start_idx].vx);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

        double max_steer = 0.0;
        double max_steer_change = 0.0;
        double max_heading_err = 0.0;
        int solver_ok = 0;
        double prev_steer = 0.0;
        int any_giant_steer = 0;  /* Flag: steering > 0.35 rad */

        /* Run 60 steps through the crossing (~3 seconds) */
        for (int step = 0; step < 60; step++) {
            double px = FP_TO_DOUBLE(state.position_x_meters);
            double py = FP_TO_DOUBLE(state.position_y_meters);
            double psi = FP_TO_DOUBLE(state.heading_angle_radians);

            int closest = find_closest_waypoint(px, py, psi);

            TrajectoryReferencePoint_t ref[HORIZON];
            build_reference(closest, psi, ref);

            MpcSolverResult_t result;
            MpcSolverStatus_t status = mpc_compute_optimal_control(&state, ref, &result);

            double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);

            if (status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                solver_ok++;

            if (fabs(steer) > fabs(max_steer)) max_steer = steer;
            double steer_change = steer - prev_steer;
            if (fabs(steer_change) > fabs(max_steer_change))
                max_steer_change = steer_change;

            double h_err = wrap_angle(psi - raceline[closest].psi);
            if (fabs(h_err) > fabs(max_heading_err)) max_heading_err = h_err;

            if (fabs(steer) > 0.35) any_giant_steer = 1;

            prev_steer = steer;

            /* Print detail in the critical zone (first 20 steps) */
            if (step < 20) {
                printf("  step=%2d wp=%3d hdg=%.4f ref_hdg=%.4f err=%.4f steer=%.4f\n",
                       step, closest, psi,
                       raceline[closest].psi, h_err, steer);
            }

            /* Sim-matched: use trajectory velocity */
            double sim_vel = get_sim_velocity(closest,
                FP_TO_DOUBLE(state.position_x_meters),
                FP_TO_DOUBLE(state.position_y_meters));
            ControlInput_t sim_ctrl;
            sim_ctrl.steering_angle_radians = result.optimal_control.steering_angle_radians;
            sim_ctrl.longitudinal_force_newtons = DOUBLE_TO_FP(sim_vel);
            state = vehicle_model_predict_next_state(&state, &sim_ctrl, DOUBLE_TO_FP(DT));
        }

        printf("  Results: max_steer=%.4f max_steer_change=%.4f max_hdg_err=%.4f solver_ok=%d/60\n",
               max_steer, max_steer_change, max_heading_err, solver_ok);

        char msg[128];
        snprintf(msg, sizeof(msg), "%s: no giant steering (>0.35)", wrap_names[w]);
        check_condition(msg, !any_giant_steer);

        snprintf(msg, sizeof(msg), "%s: heading error < 0.8 rad", wrap_names[w]);
        check_condition(msg, fabs(max_heading_err) < 0.8);

        snprintf(msg, sizeof(msg), "%s: solver ok >= 55/60", wrap_names[w]);
        check_condition(msg, solver_ok >= 55);
    }
}

/*===========================================================================
 * TEST 3: Fixed-Point Position Overflow on Spielberg Coordinates
 *===========================================================================
 *
 * The Spielberg track has coordinates x ∈ [-73.6, 12], y ∈ [-41, 55].
 * In Q16.16: x=-73.6 → -73.6 × 65536 = -4,823,450 (fits in int32).
 * But when we compute x_free[k] = A^k × x0, the entries can grow.
 * A[0][2] = dt × (-v × sin(ψ)) ≈ 0.05 × (-20 × 1) = -1.0
 * So x_free multiplies position by ~1.0 per step — should be OK.
 * But Phi[m][0][0] * Q[0] * Phi[m][0][0] with large m could overflow.
 *
 * This test checks that the Hessian doesn't contain garbage values
 * when using realistic Spielberg coordinates.
 */
static void test_fp_position_overflow(void)
{
    printf("\n[TEST] Fixed-Point Position Overflow on Spielberg Coordinates\n");

    mpc_initialize();
    mpc_reset();

    /* Pick the extreme coordinate corner: x=-73.6, y=53.5 */
    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(-73.6);
    state.position_y_meters = DOUBLE_TO_FP(53.5);
    state.heading_angle_radians = DOUBLE_TO_FP(0.02);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(20.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    /* Build reference near that position */
    TrajectoryReferencePoint_t ref[HORIZON];
    for (int i = 0; i < HORIZON; i++) {
        ref[i].reference_position_x_meters = DOUBLE_TO_FP(-73.6 + 0.5 * (i+1));
        ref[i].reference_position_y_meters = DOUBLE_TO_FP(53.5);
        ref[i].reference_heading_radians = DOUBLE_TO_FP(0.02);
        ref[i].reference_velocity_meters_per_second = DOUBLE_TO_FP(20.0);
        ref[i].reference_lateral_velocity_meters_per_second = 0;
        ref[i].reference_yaw_rate_radians_per_second = 0;
    }
    MpcSolverResult_t result;
    MpcSolverStatus_t status = mpc_compute_optimal_control(&state, ref, &result);

    double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    double vel = FP_TO_DOUBLE(result.optimal_control.longitudinal_force_newtons);

    printf("  State: x=-73.6, y=53.5, hdg=0.02, v=20.0\n");
    printf("  Result: steer=%.4f, vel=%.2f, status=%d, iter=%d\n",
           steer, vel, status, result.iterations_used);

    /* With Q_x=Q_y=0, position should not affect steering at all */
    check_condition("Large position: solver OK",
                    status == MPC_STATUS_SUCCESS ||
                    status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED);
    check_condition("Large position: reasonable steering (|steer| < 0.1)",
                    fabs(steer) < 0.1);

    /* Now test with large negative Y */
    state.position_y_meters = DOUBLE_TO_FP(-41.0);
    for (int i = 0; i < HORIZON; i++) {
        ref[i].reference_position_y_meters = DOUBLE_TO_FP(-41.0);
    }

    status = mpc_compute_optimal_control(&state, ref, &result);
    steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);

    printf("  State: x=-73.6, y=-41.0 → steer=%.4f, status=%d\n", steer, status);

    check_condition("Large negative Y: solver OK",
                    status == MPC_STATUS_SUCCESS ||
                    status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED);
    check_condition("Large negative Y: reasonable steering",
                    fabs(steer) < 0.1);
}

/*===========================================================================
 * TEST 4: Heading Near ±π Boundary (Unit Test)
 *===========================================================================
 *
 * Tests MPC with heading values very close to ±π. The key concern is that
 * normalize_angle in mpc.c uses while-loops — if the free-response
 * propagation pushes heading beyond ±π and then the reference is on the
 * other side, the tracking error d[k][2] could be ~2π instead of ~0.
 */
static void test_heading_near_pi_boundary(void)
{
    printf("\n[TEST] Heading Near ±π Boundary\n");

    /* Test cases: heading just below -π, just above +π */
    double test_headings[] = {
        3.13,    /* Just below +π (3.14159) */
        -3.13,   /* Just above -π */
        3.10,    /* Moderate negative region when wrapped */
        -3.10,
        3.14,    /* Very close to π */
        -3.14,
        M_PI - 0.001,  /* Right at the edge */
        -M_PI + 0.001,
    };
    int num_tests = sizeof(test_headings) / sizeof(test_headings[0]);

    for (int t = 0; t < num_tests; t++) {
        mpc_initialize();
        mpc_reset();

        double h = test_headings[t];

        VehicleState_t state;
        state.position_x_meters = DOUBLE_TO_FP(0.0);
        state.position_y_meters = DOUBLE_TO_FP(0.0);
        state.heading_angle_radians = DOUBLE_TO_FP(h);
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

        /* Reference: same heading but slightly different (simulating continuous driving) */
        TrajectoryReferencePoint_t ref[HORIZON];
        double ref_h = h + 0.02;  /* Slightly ahead in heading */
        for (int i = 0; i < HORIZON; i++) {
            ref[i].reference_position_x_meters = DOUBLE_TO_FP(cos(h) * 0.25 * (i+1));
            ref[i].reference_position_y_meters = DOUBLE_TO_FP(sin(h) * 0.25 * (i+1));
            ref[i].reference_heading_radians = DOUBLE_TO_FP(ref_h);
            ref[i].reference_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
            ref[i].reference_lateral_velocity_meters_per_second = 0;
            ref[i].reference_yaw_rate_radians_per_second = 0;
        }

        MpcSolverResult_t result;
        MpcSolverStatus_t status = mpc_compute_optimal_control(&state, ref, &result);

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);

        printf("  hdg=%.4f, ref_hdg=%.4f → steer=%.4f, status=%d\n",
               h, ref_h, steer, status);

        char msg[128];
        snprintf(msg, sizeof(msg), "Heading %.4f: no wild steering (|s|<0.3)", h);
        check_condition(msg, fabs(steer) < 0.3);
    }
}

/*===========================================================================
 * TEST 5: Heading Cross from +π to -π (and vice versa)
 *===========================================================================
 *
 * The most dangerous case: the vehicle heading is at +3.13 and the reference
 * heading is at -3.13 (which is only ~0.02 rad difference in reality, but
 * naively appears as ~6.26 rad difference).
 *
 * If normalize_angle or DOUBLE_TO_FP doesn't handle this correctly,
 * the MPC will see a huge heading error and command max steering.
 */
static void test_heading_cross_pi(void)
{
    printf("\n[TEST] Heading Cross ±π Boundary\n");

    struct {
        double vehicle_heading;
        double reference_heading;
        const char *desc;
    } cases[] = {
        {  3.13, -3.13, "+π → -π (small real diff ~0.02)"},
        { -3.13,  3.13, "-π → +π (small real diff ~0.02)"},
        {  3.10, -3.00, "+3.10 → -3.00 (real diff ~0.28)"},
        { -3.00,  3.10, "-3.00 → +3.10 (real diff ~0.28)"},
        {  M_PI - 0.01, -(M_PI - 0.01), "π-0.01 → -(π-0.01) (diff ~0.02)"},
        {  2.50, -2.50, "+2.50 → -2.50 (real diff ~1.28)"},
    };
    int num_cases = sizeof(cases) / sizeof(cases[0]);

    for (int c = 0; c < num_cases; c++) {
        mpc_initialize();
        mpc_reset();

        double vh = cases[c].vehicle_heading;
        double rh = cases[c].reference_heading;

        /* The "true" angular difference (shortest path) */
        double true_diff = wrap_angle(rh - vh);

        VehicleState_t state;
        state.position_x_meters = DOUBLE_TO_FP(0.0);
        state.position_y_meters = DOUBLE_TO_FP(0.0);
        state.heading_angle_radians = DOUBLE_TO_FP(vh);
        state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

        TrajectoryReferencePoint_t ref[HORIZON];
        for (int i = 0; i < HORIZON; i++) {
            ref[i].reference_position_x_meters = DOUBLE_TO_FP(0.0);
            ref[i].reference_position_y_meters = DOUBLE_TO_FP(0.0);
            ref[i].reference_heading_radians = DOUBLE_TO_FP(rh);
            ref[i].reference_velocity_meters_per_second = DOUBLE_TO_FP(5.0);
            ref[i].reference_lateral_velocity_meters_per_second = 0;
            ref[i].reference_yaw_rate_radians_per_second = 0;
        }

        MpcSolverResult_t result;
        mpc_compute_optimal_control(&state, ref, &result);

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);

        /* Proportional steering expectation:
         * For small angular diffs (<0.5), steering should be modest (<0.3)
         * For large angular diffs (>1.0), steering can be larger
         * But it should NEVER be max-steering for a 0.02 rad real diff */
        int is_reasonable;
        if (fabs(true_diff) < 0.1) {
            is_reasonable = fabs(steer) < 0.2;  /* Small diff = small steer */
        } else if (fabs(true_diff) < 0.5) {
            is_reasonable = fabs(steer) < 0.35;
        } else {
            is_reasonable = 1;  /* Large diff can reasonably need large steer */
        }

        printf("  %s\n", cases[c].desc);
        printf("    vh=%.4f, rh=%.4f, true_diff=%.4f → steer=%.4f [%s]\n",
               vh, rh, true_diff, steer, is_reasonable ? "OK" : "SUSPECT");

        char msg[128];
        snprintf(msg, sizeof(msg), "%s: proportional steering", cases[c].desc);
        check_condition(msg, is_reasonable);
    }
}

/*===========================================================================
 * TEST 6: DOUBLE_TO_FP → FP_TO_DOUBLE Round-Trip at ±π
 *===========================================================================
 *
 * Tests that DOUBLE_TO_FP handles values near ±π without silent overflow
 * or precision loss that would affect normalize_angle.
 */
static void test_fp_pi_round_trip(void)
{
    printf("\n[TEST] FP Round-Trip at ±π Values\n");

    double test_vals[] = {
        M_PI, -M_PI, M_PI - 0.001, -M_PI + 0.001,
        M_PI + 0.001, -M_PI - 0.001,  /* Slightly beyond ±π */
        2 * M_PI, -2 * M_PI,
        3.13, -3.13, 3.14, -3.14,
        3.14159, -3.14159
    };
    int count = sizeof(test_vals) / sizeof(test_vals[0]);

    for (int i = 0; i < count; i++) {
        double v = test_vals[i];
        fixed_point_t fp = DOUBLE_TO_FP(v);
        double back = FP_TO_DOUBLE(fp);
        double err = fabs(back - v);

        printf("  %.6f → FP(%d) → %.6f (err=%.6f)\n", v, fp, back, err);

        char msg[128];
        snprintf(msg, sizeof(msg), "FP round-trip %.4f: error < 0.001", v);
        /* Q16.16 precision is ~0.00002, so 0.001 is very generous */
        check_condition(msg, err < 0.001);

        /* Check if normalize_angle works on the FP value */
        fixed_point_t normalized = fp;
        while (normalized > FP_PI)  normalized -= FP_TWO_PI;
        while (normalized < -FP_PI) normalized += FP_TWO_PI;

        double norm_back = FP_TO_DOUBLE(normalized);
        snprintf(msg, sizeof(msg), "normalize(%.4f): in [-π,+π]", v);
        check_condition(msg, norm_back >= -M_PI - 0.001 && norm_back <= M_PI + 0.001);
    }
}

/*===========================================================================
 * TEST 7: Linearization at High Speed + Max Steering
 *===========================================================================
 *
 * Tests that the B matrix doesn't produce overflow when v=20 m/s and
 * δ=0.4189 rad. B[2][0] = dt × v/(L×cos²δ). With v=20, L=0.33,
 * cos(0.42)≈0.91, cos²≈0.83, B[2][0] = 0.05×20/(0.33×0.83) = 3.65.
 * In FP: 3.65 × 65536 = 239,206 — fits comfortably in int32.
 *
 * But Phi[m] = A^m × B grows with m. Check if Phi[9] overflows.
 */
static void test_linearization_overflow(void)
{
    printf("\n[TEST] Linearization Overflow at High Speed + Max Steering\n");

    mpc_initialize();

    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(0.0);
    state.position_y_meters = DOUBLE_TO_FP(0.0);
    state.heading_angle_radians = DOUBLE_TO_FP(0.0);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(20.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    ControlInput_t ctrl;
    ctrl.steering_angle_radians = DOUBLE_TO_FP(0.4189);  /* Max steering */
    ctrl.longitudinal_force_newtons = DOUBLE_TO_FP(20.0);

    fixed_point_t A[6][6], B[6][2];
    vehicle_model_compute_linearization(
        &state, &ctrl, DOUBLE_TO_FP(DT), A, B);

    printf("  B matrix at v=20, δ=0.42:\n");
    for (int r = 0; r < 6; r++) {
        printf("    B[%d] = [%.4f, %.4f]\n", r,
               FP_TO_DOUBLE(B[r][0]), FP_TO_DOUBLE(B[r][1]));
    }
    printf("  A matrix diagonal + A[0][2], A[1][2]:\n");
    printf("    A[0][2]=%.4f, A[1][2]=%.4f, A[3][3]=%.4f\n",
           FP_TO_DOUBLE(A[0][2]), FP_TO_DOUBLE(A[1][2]),
           FP_TO_DOUBLE(A[3][3]));

    /* Now compute Phi up to horizon 10 manually */
    fixed_point_t Phi[10][6][2];
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 2; c++)
            Phi[0][r][c] = B[r][c];

    for (int m = 1; m < 10; m++) {
        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 2; c++) {
                fixed_point_t sum = 0;
                for (int k = 0; k < 6; k++) {
                    sum = fp_add(sum, fp_mul(A[r][k], Phi[m-1][k][c]));
                }
                Phi[m][r][c] = sum;
            }
        }
    }

    printf("  Phi matrix propagation (steering→heading = Phi[m][2][0]):\n");
    int overflow_detected = 0;
    for (int m = 0; m < 10; m++) {
        double phi_val = FP_TO_DOUBLE(Phi[m][2][0]);
        printf("    Phi[%d][2][0] = %.4f (raw=%d)\n", m, phi_val, Phi[m][2][0]);

        /* Check for overflow: if raw FP value wraps around, the double will be wrong */
        if (fabs(phi_val) > 100.0) {
            overflow_detected = 1;
            printf("    ^^^ OVERFLOW DETECTED!\n");
        }
    }

    check_condition("Phi propagation: no overflow in 10 steps",
                    !overflow_detected);

    /* Also check the Hessian diagonal (approximate: 2 × sum Phi^T Q Phi) */
    /* The heading term dominates: Phi[m][2][0]^2 summed over m */
    double hess_approx = 0;
    for (int m = 0; m < 10; m++) {
        double phi = FP_TO_DOUBLE(Phi[m][2][0]);
        hess_approx += phi * phi;
    }
    hess_approx *= 2.0;  /* The factor of 2 in QP formulation */
    printf("  Approximate Hessian[0][0] (steering) = %.4f\n", hess_approx);
    printf("  In Q16.16: %d (max: %d)\n",
           (int)(hess_approx * 65536), 2147483647);

    check_condition("Hessian diagonal fits in Q16.16",
                    hess_approx * 65536 < 2147483647.0 &&
                    hess_approx * 65536 > -2147483648.0);
}

/*===========================================================================
 * TEST 8: Sequential MPC Calls Through ±π Crossing
 *===========================================================================
 *
 * Simulates the car driving through the ±π crossing one step at a time,
 * keeping MPC state (previous_control_input) between calls — exactly
 * as the real system would. The rate penalty term uses previous_control_input,
 * so if there's a sudden steering spike, the next step's rate penalty
 * could amplify or correct it.
 */
static void test_sequential_pi_crossing(void)
{
    printf("\n[TEST] Sequential MPC Calls Through ±π Crossing\n");

    if (raceline_count == 0) {
        printf("  SKIP: no raceline loaded\n");
        return;
    }

    /* Use the first ±π crossing (idx ~229) */
    int start = 220;
    int end = 245;

    mpc_initialize();
    mpc_reset();
    last_closest = start;

    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(raceline[start].x);
    state.position_y_meters = DOUBLE_TO_FP(raceline[start].y);
    state.heading_angle_radians = DOUBLE_TO_FP(raceline[start].psi);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(raceline[start].vx);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    double max_steer = 0;
    double max_err = 0;
    int bad_steps = 0;

    printf("  Driving through wp %d → %d (heading ±π boundary)\n", start, end);

    for (int step = 0; step < (end - start) * 3; step++) {
        double px = FP_TO_DOUBLE(state.position_x_meters);
        double py = FP_TO_DOUBLE(state.position_y_meters);
        double psi = FP_TO_DOUBLE(state.heading_angle_radians);

        int closest = find_closest_waypoint(px, py, psi);

        TrajectoryReferencePoint_t ref[HORIZON];
        build_reference(closest, psi, ref);

        MpcSolverResult_t result;
        mpc_compute_optimal_control(&state, ref, &result);

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
        double h_err = wrap_angle(psi - raceline[closest].psi);

        if (fabs(steer) > fabs(max_steer)) max_steer = steer;
        if (fabs(h_err) > fabs(max_err)) max_err = h_err;

        /* Flag if steering is suspiciously large given the smooth track */
        if (fabs(steer) > 0.30) bad_steps++;

        printf("  step=%2d wp=%3d hdg=%.4f ref=%.4f err=%.4f steer=%.4f %s\n",
               step, closest, psi, raceline[closest].psi,
               h_err, steer, fabs(steer) > 0.30 ? "***" : "");

        /* Sim-matched: use trajectory velocity */
        double sim_vel = get_sim_velocity(closest, px, py);
        ControlInput_t sim_ctrl;
        sim_ctrl.steering_angle_radians = result.optimal_control.steering_angle_radians;
        sim_ctrl.longitudinal_force_newtons = DOUBLE_TO_FP(sim_vel);
        state = vehicle_model_predict_next_state(&state, &sim_ctrl, DOUBLE_TO_FP(DT));
    }

    printf("  max_steer=%.4f max_err=%.4f bad_steps=%d\n",
           max_steer, max_err, bad_steps);

    check_condition("Sequential ±π crossing: few bad steps (<5)",
                    bad_steps < 5);
    check_condition("Sequential ±π crossing: heading error < 0.5",
                    fabs(max_err) < 0.5);
}

/*===========================================================================
 * TEST 9: Rapid Curvature Changes
 *===========================================================================
 *
 * Tests MPC at sections of the track with high curvature (tight corners).
 * High curvature means the linearization changes rapidly between steps,
 * which can cause the condensed MPC formulation to be inaccurate.
 */
static void test_high_curvature_sections(void)
{
    printf("\n[TEST] High Curvature Sections\n");

    if (raceline_count == 0) {
        printf("  SKIP: no raceline loaded\n");
        return;
    }

    /* Find waypoints with highest curvature */
    double max_kappa = 0;
    int max_kappa_idx = 0;
    for (int i = 0; i < raceline_count; i++) {
        if (fabs(raceline[i].kappa) > max_kappa) {
            max_kappa = fabs(raceline[i].kappa);
            max_kappa_idx = i;
        }
    }
    printf("  Max curvature: κ=%.4f at wp %d (x=%.1f, y=%.1f)\n",
           max_kappa, max_kappa_idx,
           raceline[max_kappa_idx].x, raceline[max_kappa_idx].y);

    /* Test at the sharpest corner */
    int start = (max_kappa_idx - 10 + raceline_count) % raceline_count;

    mpc_initialize();
    mpc_reset();
    last_closest = start;

    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(raceline[start].x);
    state.position_y_meters = DOUBLE_TO_FP(raceline[start].y);
    state.heading_angle_radians = DOUBLE_TO_FP(raceline[start].psi);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(raceline[start].vx);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    int solver_ok = 0;
    double max_lat_error = 0;
    double max_heading_error = 0;

    for (int step = 0; step < 80; step++) {
        double px = FP_TO_DOUBLE(state.position_x_meters);
        double py = FP_TO_DOUBLE(state.position_y_meters);
        double psi = FP_TO_DOUBLE(state.heading_angle_radians);

        int closest = find_closest_waypoint(px, py, psi);

        TrajectoryReferencePoint_t ref[HORIZON];
        build_reference(closest, psi, ref);

        MpcSolverResult_t result;
        MpcSolverStatus_t status = mpc_compute_optimal_control(&state, ref, &result);

        if (status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
            solver_ok++;

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
        double h_err = wrap_angle(psi - raceline[closest].psi);
        double dx = px - raceline[closest].x;
        double dy = py - raceline[closest].y;
        double lat_err = sqrt(dx*dx + dy*dy);

        if (lat_err > max_lat_error) max_lat_error = lat_err;
        if (fabs(h_err) > fabs(max_heading_error)) max_heading_error = h_err;

        if (step % 10 == 0) {
            printf("  step=%2d wp=%3d κ=%.3f hdg_err=%.3f lat_err=%.2f steer=%.3f\n",
                   step, closest, raceline[closest].kappa, h_err, lat_err, steer);
        }

        /* Sim-matched: use trajectory velocity */
        double sim_vel = get_sim_velocity(closest,
            FP_TO_DOUBLE(state.position_x_meters),
            FP_TO_DOUBLE(state.position_y_meters));
        ControlInput_t sim_ctrl;
        sim_ctrl.steering_angle_radians = result.optimal_control.steering_angle_radians;
        sim_ctrl.longitudinal_force_newtons = DOUBLE_TO_FP(sim_vel);
        state = vehicle_model_predict_next_state(&state, &sim_ctrl, DOUBLE_TO_FP(DT));
    }

    printf("  solver_ok=%d/80, max_lat_err=%.3f, max_hdg_err=%.3f\n",
           solver_ok, max_lat_error, max_heading_error);

    check_condition("High curvature: solver ok >= 70/80", solver_ok >= 70);
    check_condition("High curvature: lateral error < 3.0m", max_lat_error < 3.0);
    check_condition("High curvature: heading error < 1.0 rad", fabs(max_heading_error) < 1.0);
}

/*===========================================================================
 * TEST 10: Velocity Transition Stability
 *===========================================================================
 *
 * The Spielberg raceline has velocity ranging from 5.6 to 20 m/s.
 * When velocity drops sharply (entering a corner), the B matrix changes
 * significantly — B[2][0] ∝ v, so at low speed, steering has less
 * effect on heading change. This tests if the MPC handles the
 * transition smoothly.
 */
static void test_velocity_transitions(void)
{
    printf("\n[TEST] Velocity Transition Stability\n");

    if (raceline_count == 0) {
        printf("  SKIP: no raceline loaded\n");
        return;
    }

    /* Find biggest velocity drop */
    double max_vel_drop = 0;
    int drop_idx = 0;
    for (int i = 1; i < raceline_count; i++) {
        double drop = raceline[i-1].vx - raceline[i].vx;
        if (drop > max_vel_drop) {
            max_vel_drop = drop;
            drop_idx = i;
        }
    }
    printf("  Max velocity drop: %.2f m/s at wp %d (%.1f→%.1f m/s)\n",
           max_vel_drop, drop_idx,
           raceline[drop_idx-1].vx, raceline[drop_idx].vx);

    /* Start 20 waypoints before the drop */
    int start = (drop_idx - 20 + raceline_count) % raceline_count;

    mpc_initialize();
    mpc_reset();
    last_closest = start;

    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(raceline[start].x);
    state.position_y_meters = DOUBLE_TO_FP(raceline[start].y);
    state.heading_angle_radians = DOUBLE_TO_FP(raceline[start].psi);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(raceline[start].vx);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    double max_steer_change = 0;
    double prev_steer = 0;
    int oscillation_count = 0;  /* Count steering sign changes */
    double prev_steer_sign = 0;

    for (int step = 0; step < 60; step++) {
        double px = FP_TO_DOUBLE(state.position_x_meters);
        double py = FP_TO_DOUBLE(state.position_y_meters);
        double psi = FP_TO_DOUBLE(state.heading_angle_radians);

        int closest = find_closest_waypoint(px, py, psi);

        TrajectoryReferencePoint_t ref[HORIZON];
        build_reference(closest, psi, ref);

        MpcSolverResult_t result;
        mpc_compute_optimal_control(&state, ref, &result);

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
        double steer_change = steer - prev_steer;
        if (fabs(steer_change) > fabs(max_steer_change))
            max_steer_change = steer_change;

        /* Count oscillations (steering sign changes) */
        double sign = steer > 0.01 ? 1.0 : (steer < -0.01 ? -1.0 : 0.0);
        if (sign != 0 && prev_steer_sign != 0 && sign != prev_steer_sign) {
            oscillation_count++;
        }
        if (sign != 0) prev_steer_sign = sign;

        prev_steer = steer;

        if (step % 10 == 0) {
            printf("  step=%2d wp=%3d vel=%.1f steer=%.4f Δsteer=%.4f\n",
                   step, closest, raceline[closest].vx, steer, steer_change);
        }

        /* Sim-matched: use trajectory velocity */
        double sim_vel = get_sim_velocity(closest,
            FP_TO_DOUBLE(state.position_x_meters),
            FP_TO_DOUBLE(state.position_y_meters));
        ControlInput_t sim_ctrl;
        sim_ctrl.steering_angle_radians = result.optimal_control.steering_angle_radians;
        sim_ctrl.longitudinal_force_newtons = DOUBLE_TO_FP(sim_vel);
        state = vehicle_model_predict_next_state(&state, &sim_ctrl, DOUBLE_TO_FP(DT));
    }

    printf("  max_steer_change=%.4f, oscillations=%d\n",
           max_steer_change, oscillation_count);

    check_condition("Velocity transition: max steer change < 0.5",
                    fabs(max_steer_change) < 0.5);
    check_condition("Velocity transition: oscillations < 15",
                    oscillation_count < 15);
}

/*===========================================================================
 * TEST 11: Reference Heading Unwrap Consistency
 *===========================================================================
 *
 * Verifies that the reference trajectory builder correctly unwraps
 * headings across the ±π boundary. The unwrapped heading sequence
 * should be smooth (no jumps > π between consecutive steps).
 */
static void test_reference_unwrap(void)
{
    printf("\n[TEST] Reference Heading Unwrap Consistency\n");

    if (raceline_count == 0) {
        printf("  SKIP: no raceline loaded\n");
        return;
    }

    /* Test unwrapping at each of the 3 crossing locations */
    int crossing_starts[] = {225, 270, 755};
    int num_crossings = 3;
    int unwrap_failures = 0;

    for (int c = 0; c < num_crossings; c++) {
        int idx = crossing_starts[c];
        if (idx >= raceline_count) continue;

        double vehicle_heading = raceline[idx].psi;

        TrajectoryReferencePoint_t ref[HORIZON];
        build_reference(idx, vehicle_heading, ref);

        /* Check continuity of unwrapped headings */
        double prev_h = vehicle_heading;
        int jumps = 0;

        printf("  Crossing at wp %d (vehicle hdg=%.4f):\n", idx, vehicle_heading);
        for (int i = 0; i < HORIZON; i++) {
            double h = FP_TO_DOUBLE(ref[i].reference_heading_radians);
            double delta = h - prev_h;
            if (fabs(delta) > M_PI) {
                jumps++;
                printf("    JUMP at step %d: %.4f → %.4f (Δ=%.4f)\n",
                       i, prev_h, h, delta);
            }
            prev_h = h;
        }

        printf("    Heading jumps: %d\n", jumps);
        if (jumps > 0) unwrap_failures++;
    }

    check_condition("Reference unwrap: no heading jumps > π",
                    unwrap_failures == 0);
}

/*===========================================================================
 * TEST 12: fp_atan2 Comprehensive Test
 *===========================================================================
 *
 * The fp_atan2 bug (now fixed) was one root cause of random turning.
 * This test exhaustively checks fp_atan2 across all quadrants and
 * at the ±boundary to ensure the fix is complete.
 */
static void test_fp_atan2_comprehensive(void)
{
    printf("\n[TEST] fp_atan2 Comprehensive Validation\n");

    int total = 0, errors = 0;

    /* Test at many angles around the circle */
    for (int deg = -180; deg <= 180; deg += 5) {
        double angle_rad = deg * M_PI / 180.0;
        double y = sin(angle_rad);
        double x = cos(angle_rad);

        /* Test at different magnitudes */
        double magnitudes[] = {0.01, 0.1, 1.0, 5.0, 50.0, 200.0};
        for (int m = 0; m < 6; m++) {
            double mag = magnitudes[m];
            fixed_point_t fp_y = DOUBLE_TO_FP(y * mag);
            fixed_point_t fp_x = DOUBLE_TO_FP(x * mag);

            /* Skip if values would overflow Q16.16 */
            if (fabs(y * mag) > 32000 || fabs(x * mag) > 32000) continue;

            fixed_point_t fp_result = fp_atan2(fp_y, fp_x);
            double result = FP_TO_DOUBLE(fp_result);
            double expected = atan2(y * mag, x * mag);
            double err = fabs(wrap_angle(result - expected));

            total++;
            if (err > 0.1) {
                errors++;
                if (errors <= 10) {
                    printf("  ERROR: atan2(%.2f, %.2f) = %.4f, expected %.4f (err=%.4f)\n",
                           y * mag, x * mag, result, expected, err);
                }
            }
        }
    }

    printf("  Tested %d cases, %d errors (%.1f%%)\n",
           total, errors, 100.0 * errors / total);

    check_condition("fp_atan2: <1% error rate",
                    errors < total / 100 + 1);
}

/*===========================================================================
 * TEST 13: Free-Response Heading Propagation
 *===========================================================================
 *
 * The MPC computes x_free[k] = A^(k+1) × x0 for the free response.
 * The heading state can grow beyond ±π during propagation.
 * normalize_angle is called on d[k][2], but x_prev[2] is also normalized.
 * This test verifies the whole chain works correctly when starting
 * near ±π.
 */
static void test_free_response_heading(void)
{
    printf("\n[TEST] Free-Response Heading Propagation\n");

    mpc_initialize();

    /* Simulate what happens inside build_qp_from_prediction */
    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(0.0);
    state.position_y_meters = DOUBLE_TO_FP(0.0);
    state.heading_angle_radians = DOUBLE_TO_FP(3.13);  /* Near +π */
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(15.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    ControlInput_t ctrl;
    ctrl.steering_angle_radians = DOUBLE_TO_FP(0.1);  /* Slight turn */
    ctrl.longitudinal_force_newtons = DOUBLE_TO_FP(15.0);

    /* Propagate using the vehicle model (which normalizes heading) */
    printf("  Starting heading: 3.13 (near +π)\n");
    for (int k = 0; k < 15; k++) {
        VehicleState_t next = vehicle_model_predict_next_state(&state, &ctrl, DOUBLE_TO_FP(DT));
        double h = FP_TO_DOUBLE(next.heading_angle_radians);
        double prev_h = FP_TO_DOUBLE(state.heading_angle_radians);
        double jump = fabs(h - prev_h);

        printf("  step %2d: heading=%.4f (Δ=%.4f)%s\n",
               k, h, h - prev_h, jump > 3 ? " *** WRAP ***" : "");

        state = next;
    }

    /* The heading should have crossed ±π smoothly */
    double final_heading = FP_TO_DOUBLE(state.heading_angle_radians);
    printf("  Final heading: %.4f\n", final_heading);

    /* Verify heading is in valid range */
    check_condition("Free response: heading in [-π,+π]",
                    final_heading >= -M_PI - 0.01 && final_heading <= M_PI + 0.01);
}

/*===========================================================================
 * TEST 14: Rate Penalty Interaction with Heading Wrap
 *===========================================================================
 *
 * When the heading crosses ±π, the MPC might produce a wildly different
 * steering command. If the rate penalty is high (w_sr=1.0), the NEXT
 * step will penalize deviating from that wild command, potentially
 * causing persistent random turning for several steps.
 */
static void test_rate_penalty_after_wrap(void)
{
    printf("\n[TEST] Rate Penalty Interaction After Heading Wrap\n");

    mpc_initialize();
    mpc_reset();

    /* Step 1: Drive straight at heading near +π */
    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(0.0);
    state.position_y_meters = DOUBLE_TO_FP(0.0);
    state.heading_angle_radians = DOUBLE_TO_FP(3.10);
    state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(10.0);
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    /* Reference makes small heading changes crossing π */
    double ref_headings[] = {3.11, 3.12, 3.13, 3.14, -3.13, -3.12, -3.11, -3.10, -3.09, -3.08};

    double steerings[15];

    for (int iter = 0; iter < 12; iter++) {
        TrajectoryReferencePoint_t ref[HORIZON];
        for (int i = 0; i < HORIZON; i++) {
            int h_idx = iter + i;
            if (h_idx >= 10) h_idx = 9;
            ref[i].reference_position_x_meters = DOUBLE_TO_FP(0.0);
            ref[i].reference_position_y_meters = DOUBLE_TO_FP(0.0);
            ref[i].reference_heading_radians = DOUBLE_TO_FP(ref_headings[h_idx < 10 ? h_idx : 9]);
            ref[i].reference_velocity_meters_per_second = DOUBLE_TO_FP(10.0);
            ref[i].reference_lateral_velocity_meters_per_second = 0;
            ref[i].reference_yaw_rate_radians_per_second = 0;
        }

        MpcSolverResult_t result;
        mpc_compute_optimal_control(&state, ref, &result);

        double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
        steerings[iter] = steer;

        printf("  iter=%2d hdg=%.4f ref=%.4f steer=%.4f\n",
               iter, FP_TO_DOUBLE(state.heading_angle_radians),
               ref_headings[iter < 10 ? iter : 9], steer);

        state = vehicle_model_predict_next_state(
            &state, &result.optimal_control, DOUBLE_TO_FP(DT));
    }

    /* Check: after crossing ±π, steering should stay reasonable */
    int wild_count = 0;
    for (int i = 0; i < 12; i++) {
        if (fabs(steerings[i]) > 0.35) wild_count++;
    }

    printf("  Wild steering events (>0.35 rad): %d/12\n", wild_count);
    check_condition("Rate penalty: wild steerings < 3 across ±π", wild_count < 3);
}

/*===========================================================================
 * Main
 *===========================================================================*/

int main(int argc, char *argv[])
{
    printf("============================================================\n");
    printf("  MPC Raceline & Edge-Case Diagnostic Test Suite\n");
    printf("============================================================\n");
    printf("  Purpose: Diagnose 'car randomly turning' issue\n");
    printf("  Tests: Spielberg raceline simulation + targeted edge cases\n");
    printf("============================================================\n\n");

    /* Load the Spielberg raceline */
    const char *csv_path = (argc > 1) ? argv[1] :
        "../../f1tenth_planning/trajectories/Spielberg_raceline.csv";

    int loaded = load_raceline(csv_path);
    if (!loaded) {
        printf("WARNING: Could not load raceline CSV. Track-based tests will be skipped.\n");
        printf("Usage: %s <path_to_Spielberg_raceline.csv>\n", argv[0]);
    }

    /* === Tests that don't need the raceline === */
    printf("\n--- FIXED-POINT & MATH EDGE CASES ---\n");
    test_fp_pi_round_trip();
    test_fp_atan2_comprehensive();

    printf("\n--- HEADING BOUNDARY TESTS ---\n");
    test_heading_near_pi_boundary();
    test_heading_cross_pi();
    test_free_response_heading();
    test_rate_penalty_after_wrap();

    printf("\n--- LINEARIZATION & OVERFLOW TESTS ---\n");
    test_linearization_overflow();
    test_fp_position_overflow();

    /* === Tests that need the raceline === */
    if (loaded) {
        printf("\n--- RACELINE-BASED TESTS ---\n");
        test_reference_unwrap();
        test_sequential_pi_crossing();
        test_heading_wrap_crossings();
        test_high_curvature_sections();
        test_velocity_transitions();

        printf("\n--- FULL SIMULATION ---\n");
        test_full_raceline_simulation();
    }

    printf("\n============================================================\n");
    printf("  RESULTS: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("============================================================\n");

    return tests_failed > 0 ? 1 : 0;
}
