/**
 * @file test_with_raceline.c
 * @brief Test Pure Pursuit with actual Spielberg raceline data
 *
 * Loads the real raceline CSV, feeds waypoints one-by-one via
 * the scalar interface (mode=1), finalizes (mode=2), then
 * simulates the car following the trajectory (mode=0).
 *
 * Compile & run:
 *   gcc -I./include -o build/test_raceline \
 *       src/pure_pursuit_fpga.c src/fp_math_hls.c test/test_with_raceline.c -lm
 *   ./build/test_raceline
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "fpga_interface.h"
#include "fp_math_hls.h"

/* Top-level function (scalar interface) */
extern void pure_pursuit_fpga(
    uint32_t mode,
    uint32_t wp_index,
    int32_t wp_x, int32_t wp_y, int32_t wp_theta,
    int32_t wp_vel, int32_t wp_kappa,
    uint32_t wp_total,
    int32_t st_x, int32_t st_y, int32_t st_theta, int32_t st_vel,
    uint32_t st_wp_idx,
    int32_t p_min_la, int32_t p_max_la, int32_t p_la_gain,
    int32_t p_wheelbase, int32_t p_max_steer, int32_t p_max_vel,
    uint32_t p_la_points,
    int32_t* out_steering, int32_t* out_velocity,
    int32_t* out_cte, int32_t* out_heading_err,
    int32_t* out_lookahead, uint32_t* out_target_wp,
    uint32_t* out_status,
    uint32_t* out_traj_loaded, uint32_t* out_traj_size
);

/* ================================================================
 * Waypoint storage (raw doubles loaded from CSV)
 * ================================================================ */
typedef struct {
    double x, y, psi, kappa, vx;
} RacelinePoint;

/* ================================================================
 * Load raceline from CSV
 * Format: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
 * ================================================================ */
static int load_raceline_csv(const char* filename,
                              RacelinePoint* pts,
                              int max_points)
{
    FILE* f = fopen(filename, "r");
    if (!f) {
        printf("ERROR: Cannot open %s\n", filename);
        return -1;
    }

    char line[512];
    int count = 0;

    /* Skip header line */
    fgets(line, sizeof(line), f);

    while (fgets(line, sizeof(line), f) && count < max_points) {
        double s, x, y, psi, kappa, vx, ax;
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &s, &x, &y, &psi, &kappa, &vx, &ax) == 7)
        {
            pts[count].x     = x;
            pts[count].y     = y;
            pts[count].psi   = psi;
            pts[count].kappa = kappa;
            pts[count].vx    = vx;
            count++;
        }
    }
    fclose(f);
    return count;
}

/* ================================================================
 * Find closest waypoint to a position
 * ================================================================ */
static int find_closest_waypoint(const RacelinePoint* pts, int n,
                                  double x, double y)
{
    double min_dist = 1e9;
    int best = 0;
    for (int i = 0; i < n; i++) {
        double dx = pts[i].x - x;
        double dy = pts[i].y - y;
        double d = dx*dx + dy*dy;
        if (d < min_dist) {
            min_dist = d;
            best = i;
        }
    }
    return best;
}

/* ================================================================
 * Simple bicycle model for simulating vehicle motion
 * ================================================================ */
typedef struct {
    double x, y, theta, velocity;
} VehicleState;

static void bicycle_step(VehicleState* v, double steering, double velocity,
                          double wheelbase, double dt)
{
    v->velocity = velocity;
    v->x     += v->velocity * cos(v->theta) * dt;
    v->y     += v->velocity * sin(v->theta) * dt;
    v->theta += v->velocity * tan(steering) / wheelbase * dt;

    /* Normalize angle */
    while (v->theta >  M_PI) v->theta -= 2*M_PI;
    while (v->theta < -M_PI) v->theta += 2*M_PI;
}

/* ================================================================
 * Helper: call pure_pursuit_fpga with zeroed unused fields
 * ================================================================ */
static void call_fpga(uint32_t mode,
                      uint32_t wp_index, int32_t wp_x, int32_t wp_y,
                      int32_t wp_theta, int32_t wp_vel, int32_t wp_kappa,
                      uint32_t wp_total,
                      int32_t st_x, int32_t st_y, int32_t st_theta,
                      int32_t st_vel, uint32_t st_wp_idx,
                      int32_t p_min_la, int32_t p_max_la, int32_t p_la_gain,
                      int32_t p_wheelbase, int32_t p_max_steer,
                      int32_t p_max_vel, uint32_t p_la_points,
                      int32_t* out_steering, int32_t* out_velocity,
                      int32_t* out_cte, int32_t* out_heading_err,
                      int32_t* out_lookahead, uint32_t* out_target_wp,
                      uint32_t* out_status,
                      uint32_t* out_traj_loaded, uint32_t* out_traj_size)
{
    pure_pursuit_fpga(mode,
                      wp_index, wp_x, wp_y, wp_theta, wp_vel, wp_kappa,
                      wp_total,
                      st_x, st_y, st_theta, st_vel, st_wp_idx,
                      p_min_la, p_max_la, p_la_gain,
                      p_wheelbase, p_max_steer, p_max_vel, p_la_points,
                      out_steering, out_velocity,
                      out_cte, out_heading_err,
                      out_lookahead, out_target_wp,
                      out_status,
                      out_traj_loaded, out_traj_size);
}

/* ================================================================ */
int main(int argc, char** argv)
{
    const char* csv_path = "../f1tenth_planning/trajectories/Spielberg_raceline.csv";
    if (argc > 1) csv_path = argv[1];

    printf("========================================\n");
    printf("  Pure Pursuit - Spielberg Raceline Test\n");
    printf("========================================\n\n");

    /* ---- Load raceline from CSV ---- */
    RacelinePoint raceline[MAX_TRAJECTORY_SIZE];
    int num_wp = load_raceline_csv(csv_path, raceline, MAX_TRAJECTORY_SIZE);
    if (num_wp <= 0) {
        printf("Failed to load raceline\n");
        return 1;
    }
    printf("Loaded %d waypoints from %s\n", num_wp, csv_path);

    /* ---- Output variables ---- */
    int32_t  out_steering = 0, out_velocity = 0, out_cte = 0;
    int32_t  out_heading_err = 0, out_lookahead = 0;
    uint32_t out_target_wp = 0, out_status = 0;
    uint32_t out_traj_loaded = 0, out_traj_size = 0;

    /* ---- Load trajectory into FPGA: one waypoint per call (mode=1) ---- */
    printf("Loading %d waypoints into FPGA BRAM...\n", num_wp);
    for (int i = 0; i < num_wp; i++) {
        call_fpga(1,  /* mode = LOAD_WAYPOINT */
                  (uint32_t)i,
                  DOUBLE_TO_FP(raceline[i].x),
                  DOUBLE_TO_FP(raceline[i].y),
                  DOUBLE_TO_FP(raceline[i].psi),
                  DOUBLE_TO_FP(raceline[i].vx),
                  DOUBLE_TO_FP(raceline[i].kappa),
                  (uint32_t)num_wp,
                  0, 0, 0, 0, 0,   /* state (unused) */
                  0, 0, 0, 0, 0, 0, 0,  /* params (unused) */
                  &out_steering, &out_velocity,
                  &out_cte, &out_heading_err,
                  &out_lookahead, &out_target_wp,
                  &out_status,
                  &out_traj_loaded, &out_traj_size);
    }

    /* ---- Finalize trajectory (mode=2) ---- */
    call_fpga(2,  /* mode = FINALIZE */
              0, 0, 0, 0, 0, 0, (uint32_t)num_wp,
              0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0,
              &out_steering, &out_velocity,
              &out_cte, &out_heading_err,
              &out_lookahead, &out_target_wp,
              &out_status,
              &out_traj_loaded, &out_traj_size);

    printf("Trajectory loaded: %u, size: %u\n\n", out_traj_loaded, out_traj_size);
    if (!out_traj_loaded) {
        printf("FAIL: Trajectory not loaded\n");
        return 1;
    }

    /* ---- Parameters (fixed-point) ---- */
    int32_t p_min_la     = DOUBLE_TO_FP(0.5);
    int32_t p_max_la     = DOUBLE_TO_FP(2.0);
    int32_t p_la_gain    = DOUBLE_TO_FP(0.3);
    int32_t p_wheelbase  = DOUBLE_TO_FP(0.324);
    int32_t p_max_steer  = DOUBLE_TO_FP(0.4189);
    int32_t p_max_vel    = DOUBLE_TO_FP(6.0);
    uint32_t p_la_points = 20;

    /* ---- Initialize vehicle at first waypoint ---- */
    VehicleState car;
    car.x        = raceline[0].x;
    car.y        = raceline[0].y;
    car.theta    = raceline[0].psi;
    car.velocity = 0.0;

    double dt = 0.01;        /* 100 Hz control loop */
    double wheelbase = 0.324;
    int sim_steps = 5000;    /* 50 seconds of driving */
    int print_every = 100;   /* Print every 1 second */

    printf("Starting simulation: %d steps, dt=%.3f s\n", sim_steps, dt);
    printf("%-6s  %-10s %-10s %-8s %-10s %-10s %-8s %-8s\n",
           "Step", "X", "Y", "Theta", "Steering", "Velocity", "CTE", "WP_idx");
    printf("------  ---------- ---------- -------- ---------- ---------- -------- --------\n");

    double max_cte = 0;
    int errors = 0;

    for (int step = 0; step < sim_steps; step++) {
        /* Find closest waypoint */
        int wp_idx = find_closest_waypoint(raceline, num_wp, car.x, car.y);

        /* Compute steering (mode=0) */
        call_fpga(0,
                  0, 0, 0, 0, 0, 0, 0,   /* wp fields (unused in mode 0) */
                  DOUBLE_TO_FP(car.x),
                  DOUBLE_TO_FP(car.y),
                  DOUBLE_TO_FP(car.theta),
                  DOUBLE_TO_FP(car.velocity),
                  (uint32_t)wp_idx,
                  p_min_la, p_max_la, p_la_gain,
                  p_wheelbase, p_max_steer, p_max_vel, p_la_points,
                  &out_steering, &out_velocity,
                  &out_cte, &out_heading_err,
                  &out_lookahead, &out_target_wp,
                  &out_status,
                  &out_traj_loaded, &out_traj_size);

        if (out_status != STATUS_OK) {
            printf("Step %d: ERROR status=%u\n", step, out_status);
            errors++;
            if (errors > 10) break;
            continue;
        }

        double steering = FP_TO_DOUBLE(out_steering);
        double cmd_vel  = FP_TO_DOUBLE(out_velocity);
        double cte      = FP_TO_DOUBLE(out_cte);

        if (fabs(cte) > max_cte) max_cte = fabs(cte);

        /* Print periodically */
        if (step % print_every == 0) {
            printf("%-6d  %-10.3f %-10.3f %-8.3f %-10.4f %-10.2f %-8.4f %-8d\n",
                   step, car.x, car.y, car.theta * 180.0/M_PI,
                   steering * 180.0/M_PI, cmd_vel, cte, wp_idx);
        }

        /* Simulate vehicle motion */
        bicycle_step(&car, steering, cmd_vel, wheelbase, dt);
    }

    printf("\n========================================\n");
    printf("  Simulation Complete\n");
    printf("========================================\n");
    printf("Max cross-track error: %.4f m\n", max_cte);
    printf("Errors: %d\n", errors);

    if (max_cte < 1.0 && errors == 0) {
        printf("RESULT: PASS (CTE < 1.0m, no errors)\n");
    } else {
        printf("RESULT: FAIL (CTE=%.3f, errors=%d)\n", max_cte, errors);
    }

    return (max_cte < 1.0 && errors == 0) ? 0 : 1;
}
