/**
 * @file test_sim_drive_pure_pursuit.cpp
 * @brief Realistic Pure Pursuit simulation on my_track_raceline.
 *
 * Closed-loop simulation test for the Pure Pursuit controller with behavior
 * aligned to MPC/test/test_sim_drive.c:
 *   - Configurable physics and control rates (SIM_DT, PP_DT)
 *   - Wall-collision checks against raceline bounds + body margin
 *   - Gym-like single-track RK4 plant model
 *   - Optional realistic effects (drag, nonlinear tires, sensor noise)
 *   - Detailed summary metrics and pass/fail checks
 *
 * Build:
 *   colcon build --packages-select f1tenth_control --symlink-install
 *
 * Run:
 *   ./build/f1tenth_control/test_sim_drive_pure_pursuit
 */

#include "algorithms/pure_pursuit.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using f1tenth_control::PurePursuit;
using f1tenth_control::PurePursuitConfig;
using f1tenth_control::PurePursuitOutput;
using f1tenth_control::TrajectoryPoint;
using f1tenth_control::VehicleState;

constexpr double kSimDtDefault = 0.005;
constexpr double kControlDtDefault = 0.005;
constexpr double kSimDurationDefault = 100.0;
constexpr double kSpeedTimeConstantDefault = 0.17180616;

constexpr double kMaxSteering = 0.4189;
constexpr double kMaxVelocity = 20.0;
constexpr double kPhysicalMaxAccel = 7.31;
constexpr double kVehicleHalfWidth = 0.137;
constexpr double kDefaultBodySafetyMargin = 0.06;
constexpr double kTrajectoryMaxVelocity = 20.0;
constexpr double kClosestWindowMin = 20;

// Realistic simulation enhancement constants (mirrors MPC test harness).
constexpr double kRollingResistanceN = 2.79;
constexpr double kPacejkaCShape = 1.9;
constexpr double kNoisePosM = 0.01;
constexpr double kNoiseHeadingRad = 0.009;
constexpr double kNoiseVxMs = 0.05;
constexpr double kNoiseOmegaRad = 0.05;

struct WaypointWithBounds {
    TrajectoryPoint pt;
    double left_bound{5.0};
    double right_bound{5.0};
};

struct PathProjection {
    double s{0.0};
    double lateral_error{0.0};
    double heading_error{0.0};
    double left_bound{5.0};
    double right_bound{5.0};
};

struct SimConfig {
    double sim_dt{kSimDtDefault};
    double control_dt{kControlDtDefault};
    double sim_duration{kSimDurationDefault};
    double speed_time_constant{kSpeedTimeConstantDefault};

    double body_safety_margin{kDefaultBodySafetyMargin};
    double start_offset_lat{0.0};
    double start_offset_x{0.0};
    double start_offset_y{0.0};
    double start_heading_offset{0.0};
    double start_speed{0.0};
    int start_index{0};

    bool verbose{false};
    bool realistic_tires{false};
    bool realistic_drive{false};
    bool realistic_noise{false};
    unsigned int sim_seed{42u};

    double max_speed_command{kMaxVelocity};

    bool realisticMode() const {
        return realistic_tires || realistic_drive || realistic_noise;
    }
};

struct SimMetrics {
    int steps_executed{0};
    double simulated_time{0.0};

    double max_lat_err{0.0};
    double sum_lat_err{0.0};
    double max_hdg_err{0.0};
    double sum_hdg_err{0.0};
    double max_vel_err{0.0};
    double sum_vel_err{0.0};

    double max_vx{0.0};
    double sum_vx{0.0};
    double time_above_5ms{0.0};

    int wall_collisions{0};
    double progress_m{0.0};
    double avg_progress_mps{0.0};
    int completed_laps{0};
    double total_lap_time{0.0};
    double avg_lap_time{0.0};

    double max_steer_change{0.0};
    int steer_reversals{0};

    int controller_calls{0};
    int controller_ok{0};
    bool controller_invalid_output{false};
    double total_compute_us{0.0};
    double max_compute_us{0.0};
};

struct StState {
    double x{0.0};
    double y{0.0};
    double delta{0.0};
    double v{0.0};
    double psi{0.0};
    double psi_dot{0.0};
    double beta{0.0};
};

int tests_passed = 0;
int tests_failed = 0;

void check(const std::string& name, bool cond) {
    if (cond) {
        ++tests_passed;
        std::cout << "  [PASS] " << name << "\n";
    } else {
        ++tests_failed;
        std::cout << "  [FAIL] " << name << "\n";
    }
}

double clampValue(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

double wrapAngle(double a) {
    while (a > f1tenth_control::constants::PI) {
        a -= f1tenth_control::constants::TWO_PI;
    }
    while (a < -f1tenth_control::constants::PI) {
        a += f1tenth_control::constants::TWO_PI;
    }
    return a;
}

double getEnvDouble(const char* name, double fallback) {
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return fallback;
    }

    char* end = nullptr;
    const double value = std::strtod(env, &end);
    if (end == env || (end != nullptr && *end != '\0')) {
        return fallback;
    }

    return value;
}

bool getEnvFlag(const char* name) {
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return false;
    }
    return std::atoi(env) != 0;
}

bool parseCsvLine(const std::string& line, std::vector<double>& values) {
    values.clear();
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ',')) {
        try {
            values.push_back(std::stod(token));
        } catch (...) {
            return false;
        }
    }

    return !values.empty();
}

std::string findRacelinePath() {
    const char* env_path = std::getenv("RACELINE_PATH");
    if (env_path != nullptr && env_path[0] != '\0') {
        std::ifstream f(env_path);
        if (f.good()) {
            return std::string(env_path);
        }
    }

    const std::vector<std::string> candidates = {
        "f1tenth_planning/trajectories/my_track_raceline.csv",
        "trajectories/my_track_raceline.csv",
        "../trajectories/my_track_raceline.csv",
        "../../MPC/trajectories/my_track_raceline.csv",
        "../../f1tenth_planning/trajectories/my_track_raceline.csv",
        "../../../f1tenth_planning/trajectories/my_track_raceline.csv"
    };

    for (const auto& path : candidates) {
        std::ifstream f(path);
        if (f.good()) {
            return path;
        }
    }

    return candidates.front();
}

bool loadRaceline(const std::string& csv_path,
                  std::vector<WaypointWithBounds>& raceline,
                  double& track_length,
                  bool verbose) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return false;
    }

    raceline.clear();
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::vector<double> v;
        if (!parseCsvLine(line, v)) {
            continue;
        }

        if (v.size() < 6) {
            continue;
        }

        WaypointWithBounds wp;
        wp.pt.arc_length = v[0];
        wp.pt.x = v[1];
        wp.pt.y = v[2];
        wp.pt.heading = v[3];
        wp.pt.curvature = v[4];
        wp.pt.velocity = v[5];

        if (v.size() >= 9) {
            wp.left_bound = v[7];
            wp.right_bound = v[8];
        }

        wp.pt.left_bound = wp.left_bound;
        wp.pt.right_bound = wp.right_bound;
        raceline.push_back(wp);
    }

    if (raceline.size() < 3) {
        return false;
    }

    const double speed_gain = getEnvDouble("TRAJECTORY_SPEED_GAIN", 1.0);
    for (auto& wp : raceline) {
        wp.pt.velocity = clampValue(wp.pt.velocity * speed_gain, 0.0, kTrajectoryMaxVelocity);
    }

    track_length = raceline.back().pt.arc_length - raceline.front().pt.arc_length;
    if (track_length < 1e-3) {
        track_length = 0.0;
        for (size_t i = 0; i + 1 < raceline.size(); ++i) {
            const auto& a = raceline[i].pt;
            const auto& b = raceline[i + 1].pt;
            track_length += std::hypot(b.x - a.x, b.y - a.y);
        }
        track_length += std::hypot(raceline.front().pt.x - raceline.back().pt.x,
                                   raceline.front().pt.y - raceline.back().pt.y);
    }

    if (verbose) {
        std::cout << "[LOAD] " << csv_path << "\n";
    }
    std::cout << "  Loaded " << raceline.size() << " waypoints\n";

    return track_length > 1e-3;
}

size_t findClosestWaypoint(const std::vector<WaypointWithBounds>& raceline,
                           double px,
                           double py,
                           double heading,
                           size_t last_closest) {
    const size_t n = raceline.size();
    if (n == 0) {
        return 0;
    }

    size_t window = std::max(static_cast<size_t>(kClosestWindowMin), n / 4);
    if (window > n / 2) {
        window = n / 2;
    }

    size_t best = last_closest % n;
    double best_dist = std::numeric_limits<double>::max();
    const double dir_x = std::cos(heading);
    const double dir_y = std::sin(heading);

    const int n_i = static_cast<int>(n);
    for (int off = -static_cast<int>(window); off <= static_cast<int>(window); ++off) {
        int idx = static_cast<int>(last_closest % n) + off;
        idx = (idx % n_i + n_i) % n_i;
        const size_t i = static_cast<size_t>(idx);

        const double dx = raceline[i].pt.x - px;
        const double dy = raceline[i].pt.y - py;
        const double d2 = dx * dx + dy * dy;

        const double dot = dx * dir_x + dy * dir_y;
        if (dot < -0.5 && d2 > 0.25) {
            continue;
        }

        const double hdg_diff = wrapAngle(heading - raceline[i].pt.heading);
        if (std::abs(hdg_diff) > 1.57 && d2 < 4.0) {
            continue;
        }

        if (d2 < best_dist) {
            best_dist = d2;
            best = i;
        }
    }

    return best;
}

double wrapTrackS(double s,
                  const std::vector<WaypointWithBounds>& raceline,
                  double track_length) {
    if (track_length <= 1e-6 || raceline.size() <= 1) {
        return s;
    }

    const double s0 = raceline.front().pt.arc_length;
    while (s < s0) {
        s += track_length;
    }
    while (s >= s0 + track_length) {
        s -= track_length;
    }
    return s;
}

PathProjection projectPoseToRaceline(const std::vector<WaypointWithBounds>& raceline,
                                     double px,
                                     double py,
                                     double psi,
                                     size_t closest_index,
                                     double track_length) {
    PathProjection out;
    const size_t n = raceline.size();
    if (n == 0) {
        return out;
    }

    const size_t idx0 = std::min(closest_index, n - 1);
    const size_t idx1 = (idx0 + 1) % n;

    const double ax = raceline[idx0].pt.x;
    const double ay = raceline[idx0].pt.y;
    const double bx = raceline[idx1].pt.x;
    const double by = raceline[idx1].pt.y;

    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = px - ax;
    const double apy = py - ay;

    const double ab_len2 = abx * abx + aby * aby;
    double t = 0.0;
    if (ab_len2 > 1e-12) {
        t = (apx * abx + apy * aby) / ab_len2;
    }
    t = clampValue(t, 0.0, 1.0);

    const double path_x = ax + t * abx;
    const double path_y = ay + t * aby;

    const double h0 = raceline[idx0].pt.heading;
    const double h1 = raceline[idx1].pt.heading;
    const double dh = wrapAngle(h1 - h0);
    const double path_psi = wrapAngle(h0 + t * dh);

    const double dx = px - path_x;
    const double dy = py - path_y;

    out.lateral_error = -dx * std::sin(path_psi) + dy * std::cos(path_psi);
    out.heading_error = wrapAngle(psi - path_psi);
    out.left_bound = raceline[idx0].left_bound + t * (raceline[idx1].left_bound - raceline[idx0].left_bound);
    out.right_bound = raceline[idx0].right_bound + t * (raceline[idx1].right_bound - raceline[idx0].right_bound);

    double s0 = raceline[idx0].pt.arc_length;
    double s1 = raceline[idx1].pt.arc_length;
    if (idx0 == n - 1 && track_length > 1e-6) {
        s1 += track_length;
    }
    out.s = wrapTrackS(s0 + t * (s1 - s0), raceline, track_length);

    return out;
}

PurePursuitConfig loadControllerConfig(double body_safety_margin) {
    PurePursuitConfig cfg;

    // Defaults are tuned launch-like values from this repository.
    cfg.min_lookahead = getEnvDouble("PP_MIN_LOOKAHEAD", 0.37634354);
    cfg.max_lookahead = getEnvDouble("PP_MAX_LOOKAHEAD", 1.0562852);
    cfg.lookahead_gain = getEnvDouble("PP_LOOKAHEAD_GAIN", 0.062011484);
    cfg.cte_lookahead_weight = getEnvDouble("PP_CTE_LOOKAHEAD_WEIGHT", 1.0);
    cfg.cte_lookahead_gain = getEnvDouble("PP_CTE_LOOKAHEAD_GAIN", 0.041540516);
    cfg.curvature_lookahead_gain = getEnvDouble("PP_CURV_LOOKAHEAD_GAIN", 1.9003721);

    cfg.curvature_speed_factor = getEnvDouble("PP_CURV_SPEED_FACTOR", 0.1015252);
    cfg.curvature_speed_floor_ratio = getEnvDouble("PP_CURV_SPEED_FLOOR", 0.52401066);
    cfg.cte_speed_factor = getEnvDouble("PP_CTE_SPEED_FACTOR", 0.53756776);
    cfg.cte_speed_floor_ratio = getEnvDouble("PP_CTE_SPEED_FLOOR", 0.7826799);
    cfg.curvature_preview_factor = getEnvDouble("PP_CURV_PREVIEW_FACTOR", 1.6245233);

    cfg.max_lateral_accel = getEnvDouble("MAX_LAT_ACCEL", 7.27);
    cfg.min_regulated_speed = getEnvDouble("PP_MIN_REG_SPEED", 0.30);

    cfg.max_steering = getEnvDouble("PP_MAX_STEERING", kMaxSteering);
    cfg.wheelbase = getEnvDouble("PP_WHEELBASE", 0.324);
    cfg.position_tolerance = getEnvDouble("PP_POSITION_TOL", 0.5);

    cfg.vehicle_half_width = getEnvDouble("VEHICLE_HALF_WIDTH", kVehicleHalfWidth);
    cfg.wall_safety_margin = body_safety_margin;
    cfg.corridor_half_width_ref = getEnvDouble("PP_CORRIDOR_HALF_WIDTH_REF", 0.25);
    cfg.corridor_speed_floor_ratio = getEnvDouble("PP_CORRIDOR_SPEED_FLOOR", 0.20);
    cfg.corridor_lookahead_factor = getEnvDouble("PP_CORRIDOR_LOOKAHEAD_FACTOR", 2.0);

    return cfg;
}

SimConfig loadSimulationConfig() {
    SimConfig cfg;

    const double sim_dt_raw = getEnvDouble("SIM_DT", kSimDtDefault);
    cfg.sim_dt = (sim_dt_raw > 1e-6) ? sim_dt_raw : kSimDtDefault;

    const double ctrl_dt_raw = getEnvDouble("PP_DT", getEnvDouble("MPC_DT", kControlDtDefault));
    cfg.control_dt = (ctrl_dt_raw > 1e-6) ? ctrl_dt_raw : kControlDtDefault;

    const double duration_raw = getEnvDouble("SIM_DURATION", kSimDurationDefault);
    cfg.sim_duration = (duration_raw > 1e-3) ? duration_raw : kSimDurationDefault;

    const double tau_raw = getEnvDouble("PP_SPEED_TAU", kSpeedTimeConstantDefault);
    cfg.speed_time_constant = (tau_raw > 1e-3) ? tau_raw : kSpeedTimeConstantDefault;

    cfg.body_safety_margin = getEnvDouble("BODY_SAFETY_MARGIN", kDefaultBodySafetyMargin);

    cfg.start_offset_lat = getEnvDouble("START_OFFSET_LAT", 0.0);
    cfg.start_offset_x = getEnvDouble("START_OFFSET_X", 0.0);
    cfg.start_offset_y = getEnvDouble("START_OFFSET_Y", 0.0);
    cfg.start_heading_offset = getEnvDouble("START_HEADING_OFFSET", 0.0);
    cfg.start_speed = getEnvDouble("START_SPEED", 0.0);
    cfg.start_index = static_cast<int>(getEnvDouble("START_INDEX", 0.0));

    cfg.verbose = std::getenv("VERBOSE") != nullptr;

    const bool realistic_all = getEnvFlag("REALISTIC_SIM");
    cfg.realistic_tires = realistic_all || getEnvFlag("REALISTIC_TIRES");
    cfg.realistic_drive = realistic_all || getEnvFlag("REALISTIC_DRIVE");
    cfg.realistic_noise = realistic_all || getEnvFlag("REALISTIC_NOISE");

    const double seed_value = getEnvDouble("SIM_SEED", 42.0);
    cfg.sim_seed = static_cast<unsigned int>(std::max(0.0, seed_value));

    cfg.max_speed_command = getEnvDouble("PP_MAX_SPEED", 9.0168122);
    if (cfg.max_speed_command < 0.1) {
        cfg.max_speed_command = 0.1;
    }
    if (cfg.max_speed_command > kMaxVelocity) {
        cfg.max_speed_command = kMaxVelocity;
    }

    return cfg;
}

StState addScaled(const StState& s, const StState& k, double h) {
    StState out;
    out.x = s.x + h * k.x;
    out.y = s.y + h * k.y;
    out.delta = s.delta + h * k.delta;
    out.v = s.v + h * k.v;
    out.psi = s.psi + h * k.psi;
    out.psi_dot = s.psi_dot + h * k.psi_dot;
    out.beta = s.beta + h * k.beta;
    return out;
}

StState combineRk4(const StState& s,
                   const StState& k1,
                   const StState& k2,
                   const StState& k3,
                   const StState& k4,
                   double dt) {
    StState out;
    out.x = s.x + (dt / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    out.y = s.y + (dt / 6.0) * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
    out.delta = s.delta + (dt / 6.0) * (k1.delta + 2.0 * k2.delta + 2.0 * k3.delta + k4.delta);
    out.v = s.v + (dt / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    out.psi = s.psi + (dt / 6.0) * (k1.psi + 2.0 * k2.psi + 2.0 * k3.psi + k4.psi);
    out.psi_dot = s.psi_dot + (dt / 6.0) * (k1.psi_dot + 2.0 * k2.psi_dot + 2.0 * k3.psi_dot + k4.psi_dot);
    out.beta = s.beta + (dt / 6.0) * (k1.beta + 2.0 * k2.beta + 2.0 * k3.beta + k4.beta);
    return out;
}

SimMetrics runSimulation(const std::vector<WaypointWithBounds>& raceline,
                         double track_length,
                         const SimConfig& sim_cfg,
                         const PurePursuitConfig& pp_cfg) {
    SimMetrics metrics;
    if (raceline.empty()) {
        return metrics;
    }

    std::vector<TrajectoryPoint> pp_traj;
    pp_traj.reserve(raceline.size());
    for (const auto& wp : raceline) {
        TrajectoryPoint pt = wp.pt;
        pt.left_bound = wp.left_bound;
        pt.right_bound = wp.right_bound;
        pp_traj.push_back(pt);
    }

    PurePursuit controller(pp_cfg);
    controller.setTrajectory(pp_traj);

    int start_index = sim_cfg.start_index;
    if (start_index < 0) {
        start_index = 0;
    }
    if (start_index >= static_cast<int>(raceline.size())) {
        start_index = static_cast<int>(raceline.size()) - 1;
    }

    const auto& start_wp = raceline[static_cast<size_t>(start_index)];
    const double start_normal = start_wp.pt.heading + f1tenth_control::constants::PI / 2.0;

    VehicleState state;
    state.pose.x = start_wp.pt.x + sim_cfg.start_offset_lat * std::cos(start_normal) + sim_cfg.start_offset_x;
    state.pose.y = start_wp.pt.y + sim_cfg.start_offset_lat * std::sin(start_normal) + sim_cfg.start_offset_y;
    state.pose.theta = wrapAngle(start_wp.pt.heading + sim_cfg.start_heading_offset);
    state.velocity = sim_cfg.start_speed;
    state.angular_velocity = 0.0;
    state.steering_angle = 0.0;

    VehicleState true_state = state;

    size_t last_closest = static_cast<size_t>(start_index);
    size_t last_true_closest = last_closest;

    const int sim_steps = std::max(1, static_cast<int>(sim_cfg.sim_duration / sim_cfg.sim_dt));
    int control_interval = static_cast<int>(sim_cfg.control_dt / sim_cfg.sim_dt + 0.5);
    if (control_interval < 1) {
        control_interval = 1;
    }

    std::mt19937 rng(sim_cfg.sim_seed);
    std::normal_distribution<double> normal_dist(0.0, 1.0);
    auto randn = [&]() { return normal_dist(rng); };

    // Gym-like single-track model parameters (matching MPC test harness).
    constexpr double mu = 0.743;
    constexpr double mass = 3.314;
    constexpr double iz = 0.035;
    constexpr double c_sf = 4.297;
    constexpr double c_sr = 3.473;
    constexpr double lf = 0.166;
    constexpr double lr = 0.16;
    constexpr double h_cg = 0.0703;
    constexpr double g_acc = 9.81;
    constexpr double sv_max = 2.8492;
    constexpr double s_max = 0.4189;
    constexpr double v_switch = 7.319;
    constexpr double v_min = 0.0;
    constexpr double v_max = 20.0;
    constexpr double lwb = 0.326;

    StState st;
    st.x = true_state.pose.x;
    st.y = true_state.pose.y;
    st.delta = true_state.steering_angle;
    st.v = true_state.velocity;
    st.psi = true_state.pose.theta;
    st.psi_dot = true_state.angular_velocity;
    st.beta = 0.0;

    double cmd_steer = 0.0;
    double cmd_accel = 0.0;
    double actual_steer = 0.0;
    double prev_steer = 0.0;

    int progress_initialized = 0;
    double unwrapped_s = 0.0;
    double last_projected_s = 0.0;
    double start_projected_s = 0.0;
    double next_lap_marker_s = 0.0;
    double last_lap_cross_time = 0.0;

    auto stDynamics = [&](const StState& s, double steer_vel, double accel_cmd) {
        StState d;

        if (s.v < 0.5) {
            const double tan_delta = std::tan(s.delta);
            const double beta_hat = std::atan(tan_delta * lr / lwb);
            const double denom = 1.0 + std::pow(tan_delta * lr / lwb, 2.0);
            const double beta_dot = (denom > 1e-9)
                ? (1.0 / denom) * (lr / (lwb * std::pow(std::cos(s.delta), 2.0))) * steer_vel
                : 0.0;

            d.x = s.v * std::cos(s.psi + beta_hat);
            d.y = s.v * std::sin(s.psi + beta_hat);
            d.delta = steer_vel;
            d.v = accel_cmd;
            d.psi = s.v * std::cos(beta_hat) * tan_delta / lwb;
            d.psi_dot = (1.0 / lwb) * (
                accel_cmd * std::cos(s.beta) * tan_delta
                - s.v * std::sin(s.beta) * tan_delta * beta_dot
                + (s.v * std::cos(s.beta) * steer_vel) / std::pow(std::cos(s.delta), 2.0));
            d.beta = beta_dot;
            return d;
        }

        const double vx = s.v * std::cos(s.beta);
        const double vy = s.v * std::sin(s.beta);
        const double vx_safe = (vx > 0.5) ? vx : 0.5;

        const double fzf = mass * (g_acc * lr - accel_cmd * h_cg) / lwb;
        const double fzr = mass * (g_acc * lf + accel_cmd * h_cg) / lwb;

        const double alpha_f = s.delta - std::atan2(vy + lf * s.psi_dot, vx_safe);
        const double alpha_r = -std::atan2(vy - lr * s.psi_dot, vx_safe);

        double fyf = 0.0;
        double fyr = 0.0;
        if (sim_cfg.realistic_tires) {
            const double b_f = c_sf / kPacejkaCShape;
            const double b_r = c_sr / kPacejkaCShape;
            const double d_f = mu * fzf;
            const double d_r = mu * fzr;
            fyf = d_f * std::sin(kPacejkaCShape * std::atan(b_f * alpha_f));
            fyr = d_r * std::sin(kPacejkaCShape * std::atan(b_r * alpha_r));
        } else {
            fyf = mu * c_sf * alpha_f * fzf;
            fyr = mu * c_sr * alpha_r * fzr;
        }

        const double fx_raw = mass * accel_cmd;
        const double fx = sim_cfg.realistic_drive ? (fx_raw - kRollingResistanceN) : fx_raw;

        const double cos_delta = std::cos(s.delta);
        const double sin_delta = std::sin(s.delta);

        const double dvx_dt = (fx - fyf * sin_delta + mass * vy * s.psi_dot) / mass;
        const double dvy_dt = (fyf * cos_delta + fyr - mass * vx * s.psi_dot) / mass;

        const double v_safe = (s.v > 0.001) ? s.v : 0.001;
        double v_sq = s.v * s.v;
        if (v_sq < 0.001) {
            v_sq = 0.001;
        }

        d.x = s.v * std::cos(s.psi + s.beta);
        d.y = s.v * std::sin(s.psi + s.beta);
        d.delta = steer_vel;
        d.v = (vx * dvx_dt + vy * dvy_dt) / v_safe;
        d.psi = s.psi_dot;
        d.psi_dot = (lf * fyf * cos_delta - lr * fyr) / iz;
        d.beta = (vx * dvy_dt - vy * dvx_dt) / v_sq;

        return d;
    };

    if (sim_cfg.verbose) {
        std::cout << "\n  Step | Time  | vx    | v_cmd | e_y   | e_psi | cmd_st | act_st | accel | wp  | wall?\n";
        std::cout << "  -----|-------|-------|-------|-------|-------|--------|--------|-------|-----|------\n";
    }

    for (int step = 0; step < sim_steps; ++step) {
        const double t = static_cast<double>(step) * sim_cfg.sim_dt;
        metrics.steps_executed = step + 1;

        const size_t closest = findClosestWaypoint(
            raceline, state.pose.x, state.pose.y, state.pose.theta, last_closest);
        last_closest = closest;

        const size_t true_closest = findClosestWaypoint(
            raceline, true_state.pose.x, true_state.pose.y, true_state.pose.theta, last_true_closest);
        last_true_closest = true_closest;

        const PathProjection true_proj = projectPoseToRaceline(
            raceline,
            true_state.pose.x,
            true_state.pose.y,
            true_state.pose.theta,
            true_closest,
            track_length);

        if (!progress_initialized) {
            start_projected_s = true_proj.s;
            last_projected_s = true_proj.s;
            unwrapped_s = true_proj.s;
            next_lap_marker_s = start_projected_s + track_length;
            last_lap_cross_time = 0.0;
            progress_initialized = 1;
        } else {
            double ds = true_proj.s - last_projected_s;
            if (track_length > 1e-6) {
                if (ds < -0.5 * track_length) {
                    ds += track_length;
                } else if (ds > 0.5 * track_length) {
                    ds -= track_length;
                }
            }
            unwrapped_s += ds;
            last_projected_s = true_proj.s;
        }

        metrics.progress_m = unwrapped_s - start_projected_s;
        while (track_length > 1e-6 && unwrapped_s >= next_lap_marker_s) {
            ++metrics.completed_laps;
            metrics.total_lap_time += (t - last_lap_cross_time);
            last_lap_cross_time = t;
            next_lap_marker_s += track_length;
        }

        const double speed_mps = std::max(0.0, true_state.velocity);
        if (speed_mps > metrics.max_vx) {
            metrics.max_vx = speed_mps;
        }
        metrics.sum_vx += speed_mps;

        const double e_y = true_proj.lateral_error;
        const double e_psi = true_proj.heading_error;

        metrics.max_lat_err = std::max(metrics.max_lat_err, std::abs(e_y));
        metrics.sum_lat_err += std::abs(e_y);
        metrics.max_hdg_err = std::max(metrics.max_hdg_err, std::abs(e_psi));
        metrics.sum_hdg_err += std::abs(e_psi);

        const double ref_speed = raceline[true_closest].pt.velocity;
        const double vel_err = std::abs(speed_mps - ref_speed);
        metrics.max_vel_err = std::max(metrics.max_vel_err, vel_err);
        metrics.sum_vel_err += vel_err;

        if (speed_mps > 5.0) {
            metrics.time_above_5ms += sim_cfg.sim_dt;
        }

        const double footprint_margin = kVehicleHalfWidth + sim_cfg.body_safety_margin;
        int wall_hit = 0;
        if (e_y > (true_proj.left_bound - footprint_margin)) {
            wall_hit = 1;
            ++metrics.wall_collisions;
        }
        if (e_y < -(true_proj.right_bound - footprint_margin)) {
            wall_hit = -1;
            ++metrics.wall_collisions;
        }
        if (wall_hit != 0) {
            std::cout << "\n  !!! WALL CRASH: e_y = " << std::fixed << std::setprecision(3)
                      << e_y << " m (bound: "
                      << (wall_hit > 0 ? true_proj.left_bound : true_proj.right_bound)
                      << ") at step " << step << " (t=" << t
                      << "s, wp=" << true_closest << ", v=" << speed_mps << ") !!!\n";
            break;
        }

        if (step % control_interval == 0) {
            const auto t0 = std::chrono::steady_clock::now();
            const PurePursuitOutput output = controller.compute(state);
            const auto t1 = std::chrono::steady_clock::now();

            const double compute_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            metrics.total_compute_us += compute_us;
            metrics.max_compute_us = std::max(metrics.max_compute_us, compute_us);

            ++metrics.controller_calls;
            if (!output.valid) {
                metrics.controller_invalid_output = true;
                std::cout << "\n  !!! CONTROLLER OUTPUT INVALID at step " << step
                          << " (t=" << t << "s) !!!\n";
                break;
            }

            ++metrics.controller_ok;
            const double speed_cmd = clampValue(output.target_speed, 0.0, sim_cfg.max_speed_command);
            cmd_steer = clampValue(output.steering_angle, -pp_cfg.max_steering, pp_cfg.max_steering);
            cmd_accel = (speed_cmd - state.velocity) / sim_cfg.speed_time_constant;
            cmd_accel = clampValue(cmd_accel, -kPhysicalMaxAccel, kPhysicalMaxAccel);
        }

        double steer = clampValue(cmd_steer, -kMaxSteering, kMaxSteering);
        double accel_cmd = clampValue(cmd_accel, -kPhysicalMaxAccel, kPhysicalMaxAccel);
        actual_steer = steer;

        const double steer_change = actual_steer - prev_steer;
        if (std::abs(steer_change) > std::abs(metrics.max_steer_change)) {
            metrics.max_steer_change = steer_change;
        }
        if (step > 0 && actual_steer * prev_steer < 0.0 && std::abs(steer_change) > 0.1) {
            ++metrics.steer_reversals;
        }
        prev_steer = actual_steer;

        double steer_vel = 0.0;
        {
            const double diff = actual_steer - st.delta;
            if (std::abs(diff) > 1e-4) {
                steer_vel = (diff > 0.0 ? 1.0 : -1.0) * sv_max;
            }
            if (st.delta >= s_max && steer_vel > 0.0) {
                steer_vel = 0.0;
            }
            if (st.delta <= -s_max && steer_vel < 0.0) {
                steer_vel = 0.0;
            }
            steer_vel = clampValue(steer_vel, -sv_max, sv_max);
        }

        if (st.v > v_switch) {
            const double a_max_eff = kPhysicalMaxAccel * v_switch / st.v;
            if (accel_cmd > a_max_eff) {
                accel_cmd = a_max_eff;
            }
        }
        accel_cmd = clampValue(accel_cmd, -kPhysicalMaxAccel, kPhysicalMaxAccel);
        if (st.v <= v_min && accel_cmd < 0.0) {
            accel_cmd = 0.0;
        }
        if (st.v >= v_max && accel_cmd > 0.0) {
            accel_cmd = 0.0;
        }

        const StState k1 = stDynamics(st, steer_vel, accel_cmd);
        const StState k2 = stDynamics(addScaled(st, k1, 0.5 * sim_cfg.sim_dt), steer_vel, accel_cmd);
        const StState k3 = stDynamics(addScaled(st, k2, 0.5 * sim_cfg.sim_dt), steer_vel, accel_cmd);
        const StState k4 = stDynamics(addScaled(st, k3, sim_cfg.sim_dt), steer_vel, accel_cmd);
        StState sn = combineRk4(st, k1, k2, k3, k4, sim_cfg.sim_dt);

        sn.delta = clampValue(sn.delta, -s_max, s_max);
        sn.v = clampValue(sn.v, v_min, v_max);
        sn.psi = wrapAngle(sn.psi);
        st = sn;

        true_state.pose.x = sn.x;
        true_state.pose.y = sn.y;
        true_state.pose.theta = sn.psi;
        true_state.velocity = sn.v;
        true_state.angular_velocity = sn.psi_dot;
        true_state.steering_angle = sn.delta;

        if (sim_cfg.realistic_noise) {
            state.pose.x = sn.x + kNoisePosM * randn();
            state.pose.y = sn.y + kNoisePosM * randn();
            state.pose.theta = wrapAngle(sn.psi + kNoiseHeadingRad * randn());
            state.velocity = std::max(0.0, sn.v + kNoiseVxMs * randn());
            state.angular_velocity = sn.psi_dot + kNoiseOmegaRad * randn();
            state.steering_angle = sn.delta;
        } else {
            state = true_state;
        }

        if (sim_cfg.verbose) {
            const bool print_row = (step < 40) || (step % 20 == 0) || (std::abs(e_y) > 0.8);
            if (print_row) {
                std::cout << "  " << std::setw(4) << step
                          << " | " << std::setw(5) << std::fixed << std::setprecision(2) << t
                          << " | " << std::setw(5) << speed_mps
                          << " | " << std::setw(5) << raceline[closest].pt.velocity
                          << " | " << std::showpos << std::setw(6) << std::setprecision(3) << e_y
                          << " | " << std::setw(6) << e_psi
                          << " | " << std::setw(6) << std::setprecision(4) << cmd_steer
                          << " | " << std::setw(6) << actual_steer
                          << " | " << std::noshowpos << std::setw(5) << std::setprecision(2) << cmd_accel
                          << " | " << std::setw(3) << closest
                          << " | \n";
            }
        }

        if (std::abs(e_y) > 3.0) {
            std::cout << "\n  !!! CRASH: e_y = " << e_y << " m at step " << step
                      << " (t=" << t << "s, wp=" << closest << ") !!!\n";
            break;
        }
    }

    const int metric_steps = std::max(1, metrics.steps_executed);
    metrics.simulated_time = static_cast<double>(metric_steps) * sim_cfg.sim_dt;
    metrics.avg_progress_mps = metrics.progress_m / metrics.simulated_time;
    if (metrics.completed_laps > 0) {
        metrics.avg_lap_time = metrics.total_lap_time / static_cast<double>(metrics.completed_laps);
    }

    return metrics;
}

}  // namespace

int main() {
    const SimConfig sim_cfg = loadSimulationConfig();

    std::cout << "=== my_track Sim-Drive Test (Pure Pursuit, "
              << std::fixed << std::setprecision(0) << sim_cfg.sim_duration
              << "s at dt=" << std::setprecision(4) << sim_cfg.sim_dt
              << "s, " << std::setprecision(0) << (1.0 / sim_cfg.sim_dt) << "Hz) ===\n";

    int control_interval = static_cast<int>(sim_cfg.control_dt / sim_cfg.sim_dt + 0.5);
    if (control_interval < 1) {
        control_interval = 1;
    }

    std::cout << "    Controller rate: " << std::fixed << std::setprecision(0)
              << (1.0 / sim_cfg.control_dt)
              << "Hz (every " << control_interval << " sim steps)\n";

    if (sim_cfg.realisticMode()) {
        std::cout << "    REALISTIC MODE:";
        if (sim_cfg.realistic_drive) {
            std::cout << " [drag: F_roll=" << kRollingResistanceN << "N]";
        }
        if (sim_cfg.realistic_tires) {
            std::cout << " [Pacejka tires: C=" << kPacejkaCShape << "]";
        }
        if (sim_cfg.realistic_noise) {
            std::cout << " [sensor noise]";
        }
        std::cout << " [seed=" << sim_cfg.sim_seed << "]\n";
    }

    if (std::abs(sim_cfg.body_safety_margin - kDefaultBodySafetyMargin) > 1e-9) {
        std::cout << "    BODY_SAFETY_MARGIN: " << std::fixed << std::setprecision(3)
                  << sim_cfg.body_safety_margin << "m (default: "
                  << kDefaultBodySafetyMargin << "m)\n";
    }

    if (std::abs(sim_cfg.start_offset_lat) > 1e-9 ||
        std::abs(sim_cfg.start_offset_x) > 1e-9 ||
        std::abs(sim_cfg.start_offset_y) > 1e-9 ||
        std::abs(sim_cfg.start_heading_offset) > 1e-9 ||
        std::abs(sim_cfg.start_speed) > 1e-9) {
        std::cout << "    START override: lat=" << std::showpos << std::fixed << std::setprecision(3)
                  << sim_cfg.start_offset_lat << "m dx=" << sim_cfg.start_offset_x
                  << "m dy=" << sim_cfg.start_offset_y << "m hdg="
                  << sim_cfg.start_heading_offset << "rad v0="
                  << std::noshowpos << std::setprecision(2) << sim_cfg.start_speed << "m/s\n";
    }

    std::cout << "\n";

    const std::string raceline_path = findRacelinePath();
    std::vector<WaypointWithBounds> raceline;
    double track_length = 0.0;

    if (!loadRaceline(raceline_path, raceline, track_length, sim_cfg.verbose)) {
        std::cerr << "ERROR: Cannot open raceline CSV at " << raceline_path << "\n";
        return 1;
    }

    const PurePursuitConfig pp_cfg = loadControllerConfig(sim_cfg.body_safety_margin);
    const SimMetrics metrics = runSimulation(raceline, track_length, sim_cfg, pp_cfg);

    const int metric_steps = std::max(1, metrics.steps_executed);
    const double avg_lat = metrics.sum_lat_err / metric_steps;
    const double avg_hdg = metrics.sum_hdg_err / metric_steps;
    const double avg_vel = metrics.sum_vel_err / metric_steps;
    const double avg_vx = metrics.sum_vx / metric_steps;

    const double controller_success_pct =
        100.0 * static_cast<double>(metrics.controller_ok) /
        static_cast<double>(std::max(1, metrics.controller_calls));
    const double avg_compute_us =
        metrics.total_compute_us / static_cast<double>(std::max(1, metrics.controller_calls));

    std::cout << "\n  === Results (" << std::fixed << std::setprecision(1)
              << metrics.simulated_time << " seconds, Pure Pursuit, "
              << std::setprecision(0) << (1.0 / sim_cfg.control_dt) << "Hz ctrl) ===\n";
    std::cout << "  Controller valid:   " << metrics.controller_ok << " / "
              << metrics.controller_calls << " (" << std::setprecision(1)
              << controller_success_pct << "%)\n";
    std::cout << "  Max velocity:       " << std::setprecision(2) << metrics.max_vx << " m/s\n";
    std::cout << "  Avg velocity:       " << avg_vx << " m/s\n";
    std::cout << "  Max lateral error:  " << std::setprecision(3) << metrics.max_lat_err << " m\n";
    std::cout << "  Avg lateral error:  " << avg_lat << " m\n";
    std::cout << "  Max heading error:  " << std::setprecision(4) << metrics.max_hdg_err
              << " rad (" << std::setprecision(1) << metrics.max_hdg_err * 180.0 / f1tenth_control::constants::PI
              << " deg)\n";
    std::cout << "  Avg heading error:  " << std::setprecision(4) << avg_hdg
              << " rad (" << std::setprecision(1) << avg_hdg * 180.0 / f1tenth_control::constants::PI
              << " deg)\n";
    std::cout << "  Max velocity error: " << std::setprecision(2) << metrics.max_vel_err << " m/s\n";
    std::cout << "  Avg velocity error: " << avg_vel << " m/s\n";
    std::cout << "  Track progress:     " << std::setprecision(2) << metrics.progress_m
              << " m (" << metrics.avg_progress_mps << " m/s)\n";
    std::cout << "  Completed laps:     " << metrics.completed_laps << "\n";
    if (metrics.completed_laps > 0) {
        std::cout << "  Avg lap time:       " << std::setprecision(3) << metrics.avg_lap_time << " s\n";
    }
    std::cout << "  Max steer change:   " << std::setprecision(4) << metrics.max_steer_change << " rad/step\n";
    std::cout << "  Steer reversals:    " << metrics.steer_reversals << "\n";
    std::cout << "  Wall collisions:    " << metrics.wall_collisions << "\n";
    std::cout << "  Time above 5 m/s:   " << std::setprecision(1)
              << metrics.time_above_5ms << " / " << metrics.simulated_time << " s ("
              << ((metrics.simulated_time > 0.0)
                      ? (100.0 * metrics.time_above_5ms / metrics.simulated_time)
                      : 0.0)
              << "%)\n";

    std::cout << "\n  --- Controller Performance ---\n";
    std::cout << "  Avg compute time:   " << std::setprecision(1) << avg_compute_us << " us\n";
    std::cout << "  Max compute time:   " << metrics.max_compute_us << " us\n";
    std::cout << "  Total compute time: " << metrics.total_compute_us / 1000.0 << " ms\n\n";

    const double speed_threshold = sim_cfg.realisticMode() ? 0.30 : 0.50;
    check("No wall collisions", metrics.wall_collisions == 0);
    check("Max lateral error < 1.2 m", metrics.max_lat_err < 1.2);
    check("Avg lateral error < 0.5 m", avg_lat < 0.5);
    check("Avg heading error < 0.3 rad (17 deg)", avg_hdg < 0.3);
    check("Controller mostly succeeds (>80%)",
          metrics.controller_ok > metrics.controller_calls * 80 / 100);

    int speed_check_pass = 1;
    double ref_peak_speed = 0.0;
    double ref_avg_speed = 0.0;
    for (const auto& wp : raceline) {
        ref_peak_speed = std::max(ref_peak_speed, wp.pt.velocity);
        ref_avg_speed += wp.pt.velocity;
    }
    if (!raceline.empty()) {
        ref_avg_speed /= static_cast<double>(raceline.size());
    }

    if (sim_cfg.realisticMode()) {
        std::ostringstream msg;
        msg << "Reaches driving speed (>5 m/s for >" << (speed_threshold * 100.0)
            << "% of time, realistic)";
        speed_check_pass = (metrics.time_above_5ms > metrics.simulated_time * speed_threshold) ? 1 : 0;
        check(msg.str(), speed_check_pass == 1);
    } else {
        if (ref_avg_speed < 5.0) {
            const double min_avg_ratio = 0.60;
            const double min_avg_speed = ref_avg_speed * min_avg_ratio;
            std::ostringstream msg;
            msg << "Tracks low-speed map (avg speed > " << (min_avg_ratio * 100.0)
                << "% of avg ref " << std::fixed << std::setprecision(2)
                << ref_avg_speed << " m/s)";
            speed_check_pass = (avg_vx > min_avg_speed) ? 1 : 0;
            check(msg.str(), speed_check_pass == 1);
        } else {
            speed_check_pass = (metrics.time_above_5ms > metrics.simulated_time * 0.5) ? 1 : 0;
            check("Reaches driving speed (>5 m/s for >50% of time)", speed_check_pass == 1);
        }
    }

    int failed_non_speed = tests_failed - (speed_check_pass ? 0 : 1);
    if (failed_non_speed < 0) {
        failed_non_speed = 0;
    }

    std::cout << "\n=== RESULTS: " << tests_passed << " passed, "
              << tests_failed << " failed ===\n";

    if (std::getenv("PP_TUNING_CSV") != nullptr) {
        std::cout << "CSV,"
                  << tests_passed << ","
                  << tests_failed << ","
                  << metrics.max_lat_err << ","
                  << avg_lat << ","
                  << metrics.max_hdg_err << ","
                  << avg_hdg << ","
                  << metrics.max_vx << ","
                  << avg_compute_us << ","
                  << metrics.max_compute_us << ","
                  << metrics.wall_collisions << ","
                  << metrics.time_above_5ms << ","
                  << metrics.max_vel_err << ","
                  << avg_vel << ","
                  << avg_vx << ","
                  << metrics.progress_m << ","
                  << metrics.avg_progress_mps << ","
                  << metrics.completed_laps << ","
                  << metrics.avg_lap_time << ","
                  << std::abs(metrics.max_steer_change) << ","
                  << metrics.steer_reversals << ","
                  << speed_check_pass << ","
                  << failed_non_speed << ","
                  << controller_success_pct / 100.0
                  << "\n";
    }

    return (tests_failed > 0) ? 1 : 0;
}
