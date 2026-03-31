/**
 * @file test_sim_drive_stanley.cpp
 * @brief Closed-loop Stanley simulation and parameter sweep.
 *
 * This executable mirrors the Pure Pursuit simulation harness and evaluates
 * Stanley controller parameter candidates on a bounded raceline track model.
 * The sweep searches for collision-free candidates that complete target laps
 * while maximizing average speed with bounded tracking error.
 *
 * Build:
 *   colcon build --packages-select f1tenth_control --symlink-install
 *
 * Run:
 *   ./build/f1tenth_control/test_sim_drive_stanley
 *   ./build/f1tenth_control/test_sim_drive_stanley --iterations 260 --laps 3
 *   ./build/f1tenth_control/test_sim_drive_stanley --trajectory /abs/path/to/raceline.csv
 */

#include "algorithms/stanley.hpp"

#include <algorithm>
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
#include <utility>
#include <vector>

namespace {

using f1tenth_control::Stanley;
using f1tenth_control::StanleyConfig;
using f1tenth_control::StanleyOutput;
using f1tenth_control::TrajectoryPoint;
using f1tenth_control::VehicleState;

constexpr double kDefaultDt = 0.005;             // 200 Hz simulation
constexpr double kMaxSimTime = 260.0;            // seconds
constexpr double kSpeedTimeConstant = 0.30;      // seconds
constexpr double kSoftStartDuration = 2.0;       // seconds
constexpr double kSoftStartSpeed = 1.0;          // m/s
constexpr double kMaxAccel = 3.5;                // m/s^2
constexpr double kMaxBrake = 5.0;                // m/s^2
constexpr double kVehicleHalfWidth = 0.137;      // m
constexpr double kBodySafetyMargin = 0.040;      // m
constexpr size_t kClosestWindowMin = 30;

struct WaypointWithBounds {
    TrajectoryPoint pt;
    double left_bound{5.0};
    double right_bound{5.0};
};

struct SimMetrics {
    bool completed{false};
    bool collided{false};
    int laps_completed{0};
    double completion_ratio{0.0};
    double sim_time{0.0};
    double traveled_distance{0.0};
    double avg_speed{0.0};
    double max_abs_cte{0.0};
    double rms_cte{0.0};
};

struct TrackProjection {
    size_t seg_idx{0};
    double t{0.0};
    double signed_lateral{0.0};
    double left_bound{0.0};
    double right_bound{0.0};
    double dist2{std::numeric_limits<double>::max()};
};

struct Candidate {
    StanleyConfig config;
};

struct EvaluatedCandidate {
    Candidate candidate;
    SimMetrics metrics;
    double score{-1e9};
};

double clampValue(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

double normalizeAngle(double a) {
    while (a > f1tenth_control::constants::PI) {
        a -= 2.0 * f1tenth_control::constants::PI;
    }
    while (a < -f1tenth_control::constants::PI) {
        a += 2.0 * f1tenth_control::constants::PI;
    }
    return a;
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

bool loadTrajectoryWithBounds(const std::string& csv_path,
                              std::vector<WaypointWithBounds>& out,
                              double& track_length) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return false;
    }

    out.clear();
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

        WaypointWithBounds w;
        w.pt.arc_length = v[0];
        w.pt.x = v[1];
        w.pt.y = v[2];
        w.pt.heading = v[3];
        w.pt.curvature = v[4];
        w.pt.velocity = v[5];
        if (v.size() >= 9) {
            w.left_bound = v[7];
            w.right_bound = v[8];
        }
        out.push_back(w);
    }

    if (out.size() < 3) {
        return false;
    }

    track_length = 0.0;
    for (size_t i = 0; i + 1 < out.size(); ++i) {
        const auto& a = out[i].pt;
        const auto& b = out[i + 1].pt;
        track_length += std::hypot(b.x - a.x, b.y - a.y);
    }
    track_length += std::hypot(out.front().pt.x - out.back().pt.x,
                               out.front().pt.y - out.back().pt.y);

    return track_length > 1e-3;
}

std::string defaultTrajectoryPath() {
    const std::vector<std::string> candidates = {
        "f1tenth_planning/trajectories/my_track_raceline.csv",
        "../f1tenth_planning/trajectories/my_track_raceline.csv",
        "../../f1tenth_planning/trajectories/my_track_raceline.csv"
    };

    for (const auto& path : candidates) {
        std::ifstream file(path);
        if (file.good()) {
            return path;
        }
    }

    return candidates.front();
}

size_t findClosestIndex(const std::vector<WaypointWithBounds>& traj,
                        const VehicleState& state,
                        size_t last_idx) {
    const size_t n = traj.size();
    if (n == 0) {
        return 0;
    }

    size_t window = std::max(kClosestWindowMin, n / 8);
    if (window > n / 2) {
        window = n / 2;
    }

    size_t best_idx = last_idx;
    double best_d2 = std::numeric_limits<double>::max();

    for (int off = -static_cast<int>(window); off <= static_cast<int>(window); ++off) {
        const int idx_raw = static_cast<int>(last_idx) + off;
        const int wrapped =
            (idx_raw % static_cast<int>(n) + static_cast<int>(n)) % static_cast<int>(n);
        const size_t idx = static_cast<size_t>(wrapped);

        const double dx = traj[idx].pt.x - state.pose.x;
        const double dy = traj[idx].pt.y - state.pose.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = idx;
        }
    }

    return best_idx;
}

double crossTrackError(const WaypointWithBounds& wp, const VehicleState& state) {
    const double dx = state.pose.x - wp.pt.x;
    const double dy = state.pose.y - wp.pt.y;
    return -std::sin(wp.pt.heading) * dx + std::cos(wp.pt.heading) * dy;
}

TrackProjection projectToTrack(const std::vector<WaypointWithBounds>& traj,
                               const VehicleState& state,
                               size_t around_idx) {
    TrackProjection best;
    const size_t n = traj.size();
    if (n < 2) {
        return best;
    }

    const int window = static_cast<int>(std::max<size_t>(20, n / 10));
    const int n_i = static_cast<int>(n);

    for (int off = -window; off <= window; ++off) {
        int i_raw = static_cast<int>(around_idx) + off;
        int i_wrap = (i_raw % n_i + n_i) % n_i;
        const size_t i = static_cast<size_t>(i_wrap);
        const size_t j = (i + 1) % n;

        const double ax = traj[i].pt.x;
        const double ay = traj[i].pt.y;
        const double bx = traj[j].pt.x;
        const double by = traj[j].pt.y;
        const double vx = bx - ax;
        const double vy = by - ay;
        const double seg_len2 = vx * vx + vy * vy;
        if (seg_len2 <= 1e-10) {
            continue;
        }

        const double wx = state.pose.x - ax;
        const double wy = state.pose.y - ay;
        const double t = clampValue((wx * vx + wy * vy) / seg_len2, 0.0, 1.0);
        const double px = ax + t * vx;
        const double py = ay + t * vy;
        const double dx = state.pose.x - px;
        const double dy = state.pose.y - py;
        const double d2 = dx * dx + dy * dy;

        if (d2 < best.dist2) {
            const double seg_len = std::sqrt(seg_len2);
            const double nx = -vy / seg_len;
            const double ny = vx / seg_len;

            best.seg_idx = i;
            best.t = t;
            best.signed_lateral = dx * nx + dy * ny;
            best.left_bound = traj[i].left_bound + t * (traj[j].left_bound - traj[i].left_bound);
            best.right_bound = traj[i].right_bound + t * (traj[j].right_bound - traj[i].right_bound);
            best.dist2 = d2;
        }
    }

    if (best.dist2 == std::numeric_limits<double>::max()) {
        const auto& wp = traj[around_idx % n];
        best.seg_idx = around_idx % n;
        best.t = 0.0;
        best.signed_lateral = crossTrackError(wp, state);
        best.left_bound = wp.left_bound;
        best.right_bound = wp.right_bound;
        best.dist2 = best.signed_lateral * best.signed_lateral;
    }

    return best;
}

double advanceAlongTrack(const std::vector<WaypointWithBounds>& traj,
                         size_t prev_idx,
                         size_t curr_idx) {
    const size_t n = traj.size();
    if (n == 0 || prev_idx == curr_idx) {
        return 0.0;
    }

    int delta = static_cast<int>(curr_idx) - static_cast<int>(prev_idx);
    if (delta < -static_cast<int>(n) / 2) {
        delta += static_cast<int>(n);
    }
    if (delta > static_cast<int>(n) / 2) {
        delta -= static_cast<int>(n);
    }

    if (delta <= 0) {
        return 0.0;
    }

    double ds = 0.0;
    size_t idx = prev_idx;
    for (int i = 0; i < delta; ++i) {
        const size_t next = (idx + 1) % n;
        ds += std::hypot(traj[next].pt.x - traj[idx].pt.x,
                         traj[next].pt.y - traj[idx].pt.y);
        idx = next;
    }
    return ds;
}

SimMetrics runSimulation(const std::vector<WaypointWithBounds>& traj,
                         double track_length,
                         const Candidate& candidate,
                         int target_laps,
                         double dt,
                         bool verbose) {
    SimMetrics metrics;
    if (traj.empty()) {
        return metrics;
    }

    std::vector<TrajectoryPoint> st_traj;
    st_traj.reserve(traj.size());
    for (const auto& w : traj) {
        st_traj.push_back(w.pt);
    }

    Stanley controller(candidate.config);
    controller.setTrajectory(st_traj);

    VehicleState state;
    state.pose.x = traj.front().pt.x;
    state.pose.y = traj.front().pt.y;
    state.pose.theta = traj.front().pt.heading;
    state.velocity = 0.0;
    state.angular_velocity = 0.0;

    double steer_actual = 0.0;
    size_t closest_idx = 0;
    double sum_cte_sq = 0.0;
    int cte_count = 0;

    for (double t = 0.0; t < kMaxSimTime; t += dt) {
        const StanleyOutput output = controller.compute(state);
        if (!output.valid) {
            break;
        }

        const double speed_cmd = clampValue(output.target_speed, 0.0, candidate.config.max_speed);
        const double steer_cmd = clampValue(output.steering_angle,
                                            -candidate.config.max_steering,
                                            candidate.config.max_steering);

        const double soft_speed_cmd =
            (t < kSoftStartDuration) ? std::min(speed_cmd, kSoftStartSpeed) : speed_cmd;

        double accel_cmd = (soft_speed_cmd - state.velocity) / kSpeedTimeConstant;
        accel_cmd = clampValue(accel_cmd, -kMaxBrake, kMaxAccel);
        state.velocity = clampValue(state.velocity + accel_cmd * dt, 0.0, candidate.config.max_speed);

        const double max_steer_delta = std::max(1e-6, candidate.config.max_steering_rate) * dt;
        const double steer_delta = clampValue(steer_cmd - steer_actual, -max_steer_delta, max_steer_delta);
        steer_actual += steer_delta;
        steer_actual = clampValue(steer_actual,
                                  -candidate.config.max_steering,
                                  candidate.config.max_steering);

        const double wheelbase = std::max(1e-3, candidate.config.wheelbase);
        const double yaw_rate = state.velocity * std::tan(steer_actual) / wheelbase;
        const double theta = state.pose.theta;

        state.pose.x += state.velocity * std::cos(theta) * dt;
        state.pose.y += state.velocity * std::sin(theta) * dt;
        state.pose.theta = normalizeAngle(theta + yaw_rate * dt);

        state.angular_velocity = yaw_rate;
        state.steering_angle = steer_actual;

        const size_t prev_idx = closest_idx;
        closest_idx = findClosestIndex(traj, state, closest_idx);
        metrics.traveled_distance += advanceAlongTrack(traj, prev_idx, closest_idx);

        const TrackProjection proj = projectToTrack(traj, state, closest_idx);
        const double cte = proj.signed_lateral;
        metrics.max_abs_cte = std::max(metrics.max_abs_cte, std::abs(cte));
        sum_cte_sq += cte * cte;
        ++cte_count;

        const double allowed_left = proj.left_bound - (kVehicleHalfWidth + kBodySafetyMargin);
        const double allowed_right = proj.right_bound - (kVehicleHalfWidth + kBodySafetyMargin);
        if ((allowed_left > 0.0 && cte > allowed_left) ||
            (allowed_right > 0.0 && -cte > allowed_right)) {
            metrics.collided = true;
            metrics.sim_time = t;
            break;
        }

        const double laps_f = (track_length > 1e-6)
                                ? (metrics.traveled_distance / track_length)
                                : 0.0;
        metrics.laps_completed = static_cast<int>(std::floor(laps_f + 1e-6));
        if (metrics.laps_completed >= target_laps) {
            metrics.completed = true;
            metrics.sim_time = t;
            break;
        }

        metrics.sim_time = t;
    }

    if (metrics.sim_time > 1e-6) {
        metrics.avg_speed = metrics.traveled_distance / metrics.sim_time;
    }
    if (cte_count > 0) {
        metrics.rms_cte = std::sqrt(sum_cte_sq / static_cast<double>(cte_count));
    }

    const double target_dist = std::max(track_length * static_cast<double>(target_laps), 1e-6);
    metrics.completion_ratio = clampValue(metrics.traveled_distance / target_dist, 0.0, 1.0);

    if (verbose) {
        std::cout << "sim_result completed=" << metrics.completed
                  << " collided=" << metrics.collided
                  << " laps=" << metrics.laps_completed
                  << " avg_speed=" << std::fixed << std::setprecision(3) << metrics.avg_speed
                  << " max_abs_cte=" << metrics.max_abs_cte
                  << " rms_cte=" << metrics.rms_cte
                  << " completion=" << metrics.completion_ratio
                  << "\n";
    }

    return metrics;
}

double scoreCandidate(const SimMetrics& m) {
    const double tracking_penalty = 3.0 * m.max_abs_cte + 1.0 * m.rms_cte;

    if (m.completed && !m.collided) {
        return 1000.0 + 240.0 * m.avg_speed - 35.0 * tracking_penalty;
    }

    const double progress_score = 300.0 * m.completion_ratio;
    const double collision_penalty = m.collided ? 160.0 : 0.0;
    return progress_score - 80.0 * tracking_penalty - collision_penalty;
}

Candidate launchLikeBaseline() {
    Candidate c;
    c.config.k_e = 2.394;
    c.config.k_h = 1.057;
    c.config.k_s = 0.685;
    c.config.k_d = 0.069;
    c.config.max_speed = 10.0;
    c.config.min_speed = 0.5;
    c.config.speed_gain = 1.0;
    c.config.max_steering = 0.4189;
    c.config.max_steering_rate = 2.8175;
    c.config.wheelbase = 0.3302;
    c.config.position_tolerance = 0.5;
    c.config.use_feedforward = true;
    c.config.feedforward_gain = 1.412;
    c.config.curvature_speed_factor = 0.2;
    c.config.control_rate = 1.0 / kDefaultDt;
    return c;
}

Candidate conservativeSeed() {
    Candidate c;
    c.config.k_e = 1.8;
    c.config.k_h = 0.8;
    c.config.k_s = 1.4;
    c.config.k_d = 0.30;
    c.config.max_speed = 2.2;
    c.config.min_speed = 0.8;
    c.config.speed_gain = 0.95;
    c.config.max_steering = 0.4189;
    c.config.max_steering_rate = 2.2;
    c.config.wheelbase = 0.3302;
    c.config.position_tolerance = 0.5;
    c.config.use_feedforward = true;
    c.config.feedforward_gain = 0.9;
    c.config.curvature_speed_factor = 1.2;
    c.config.control_rate = 1.0 / kDefaultDt;
    return c;
}

Candidate fallbackSafeSeed() {
    Candidate c = conservativeSeed();
    c.config.max_speed = 1.6;
    c.config.speed_gain = 0.9;
    c.config.k_d = 0.35;
    c.config.curvature_speed_factor = 1.5;
    return c;
}

Candidate sampleRandom(std::mt19937& rng) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    Candidate c;
    c.config.k_e = 1.2 + 2.8 * u01(rng);
    c.config.k_h = 0.5 + 0.9 * u01(rng);
    c.config.k_s = 0.8 + 1.5 * u01(rng);
    c.config.k_d = 0.05 + 0.45 * u01(rng);
    c.config.max_speed = 1.8 + 2.8 * u01(rng);
    c.config.min_speed = 0.6 + 0.6 * u01(rng);
    c.config.speed_gain = 0.7 + 0.5 * u01(rng);
    c.config.max_steering = 0.4189;
    c.config.max_steering_rate = 1.8 + 1.6 * u01(rng);
    c.config.wheelbase = 0.3302;
    c.config.position_tolerance = 0.5;
    c.config.use_feedforward = true;
    c.config.feedforward_gain = 0.6 + 0.8 * u01(rng);
    c.config.curvature_speed_factor = 0.7 + 1.4 * u01(rng);
    c.config.control_rate = 1.0 / kDefaultDt;
    return c;
}

Candidate sampleAround(const Candidate& base, std::mt19937& rng) {
    std::normal_distribution<double> n01(0.0, 1.0);

    Candidate c = base;
    c.config.k_e = clampValue(base.config.k_e + 0.25 * n01(rng), 0.8, 4.5);
    c.config.k_h = clampValue(base.config.k_h + 0.10 * n01(rng), 0.3, 1.8);
    c.config.k_s = clampValue(base.config.k_s + 0.20 * n01(rng), 0.4, 3.0);
    c.config.k_d = clampValue(base.config.k_d + 0.05 * n01(rng), 0.0, 0.8);
    c.config.max_speed = clampValue(base.config.max_speed + 0.25 * n01(rng), 1.2, 5.5);
    c.config.min_speed = clampValue(base.config.min_speed + 0.08 * n01(rng), 0.2, 1.5);
    c.config.speed_gain = clampValue(base.config.speed_gain + 0.08 * n01(rng), 0.4, 1.5);
    c.config.max_steering_rate =
        clampValue(base.config.max_steering_rate + 0.20 * n01(rng), 0.6, 4.0);
    c.config.feedforward_gain =
        clampValue(base.config.feedforward_gain + 0.08 * n01(rng), 0.2, 1.6);
    c.config.curvature_speed_factor =
        clampValue(base.config.curvature_speed_factor + 0.15 * n01(rng), 0.2, 2.5);
    c.config.control_rate = 1.0 / kDefaultDt;
    return c;
}

void printCandidate(const EvaluatedCandidate& e, size_t rank = 0) {
    if (rank > 0) {
        std::cout << "[TOP " << rank << "] ";
    }

    std::cout << std::fixed << std::setprecision(3)
              << "score=" << e.score
              << " avg_v=" << e.metrics.avg_speed
              << " completed=" << e.metrics.completed
              << " collided=" << e.metrics.collided
              << " laps=" << e.metrics.laps_completed
              << " completion=" << e.metrics.completion_ratio
              << " t=" << e.metrics.sim_time
              << " cte_max=" << e.metrics.max_abs_cte
              << " cfg{"
              << "k_e=" << e.candidate.config.k_e
              << ", k_h=" << e.candidate.config.k_h
              << ", k_s=" << e.candidate.config.k_s
              << ", k_d=" << e.candidate.config.k_d
              << ", vmax=" << e.candidate.config.max_speed
              << ", vmin=" << e.candidate.config.min_speed
              << ", speed_gain=" << e.candidate.config.speed_gain
              << ", ff=" << e.candidate.config.feedforward_gain
              << ", k_curv_v=" << e.candidate.config.curvature_speed_factor
              << "}"
              << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string trajectory_path = defaultTrajectoryPath();
    int iterations = 240;
    int target_laps = 3;
    int seed = 42;
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--trajectory" && i + 1 < argc) {
            trajectory_path = argv[++i];
        } else if (arg == "--iterations" && i + 1 < argc) {
            iterations = std::max(10, std::atoi(argv[++i]));
        } else if (arg == "--laps" && i + 1 < argc) {
            target_laps = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = std::atoi(argv[++i]);
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--help") {
            std::cout << "Usage: test_sim_drive_stanley [options]\n"
                      << "  --trajectory <csv_path>\n"
                      << "  --iterations <N>\n"
                      << "  --laps <N>\n"
                      << "  --seed <N>\n"
                      << "  --verbose\n";
            return 0;
        }
    }

    std::vector<WaypointWithBounds> trajectory;
    double track_length = 0.0;
    if (!loadTrajectoryWithBounds(trajectory_path, trajectory, track_length)) {
        std::cerr << "ERROR: Failed to load trajectory: " << trajectory_path << "\n";
        return 1;
    }

    std::cout << "Loaded " << trajectory.size() << " waypoints from " << trajectory_path
              << " (track_length=" << std::fixed << std::setprecision(2)
              << track_length << " m)\n";
    std::cout << "Sweep settings: iterations=" << iterations
              << " laps=" << target_laps
              << " seed=" << seed
              << " dt=" << kDefaultDt << "\n\n";

    std::vector<EvaluatedCandidate> all;
    all.reserve(static_cast<size_t>(iterations) + 12);

    const Candidate baseline = launchLikeBaseline();
    const Candidate conservative = conservativeSeed();
    const Candidate fallback = fallbackSafeSeed();

    EvaluatedCandidate best;
    best.score = -1e9;

    bool have_feasible = false;
    EvaluatedCandidate best_feasible;
    best_feasible.score = -1e9;

    auto evaluate = [&](const Candidate& candidate) {
        EvaluatedCandidate e;
        e.candidate = candidate;
        e.metrics = runSimulation(trajectory, track_length, candidate, target_laps, kDefaultDt, false);
        e.score = scoreCandidate(e.metrics);
        all.push_back(e);

        if (e.score > best.score) {
            best = e;
        }

        if (e.metrics.completed && !e.metrics.collided) {
            if (!have_feasible || e.metrics.avg_speed > best_feasible.metrics.avg_speed) {
                best_feasible = e;
            }
            have_feasible = true;
        }
    };

    evaluate(baseline);
    evaluate(conservative);
    evaluate(fallback);

    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    for (int i = 0; i < iterations; ++i) {
        evaluate(sampleRandom(rng));
    }

    if (have_feasible) {
        for (int i = 0; i < iterations / 2; ++i) {
            evaluate(sampleAround(best_feasible.candidate, rng));
        }
    } else {
        for (int i = 0; i < iterations / 3; ++i) {
            evaluate(sampleAround(fallback, rng));
        }
    }

    std::sort(all.begin(), all.end(), [](const EvaluatedCandidate& a, const EvaluatedCandidate& b) {
        if (std::abs(a.score - b.score) > 1e-6) {
            return a.score > b.score;
        }
        if (a.metrics.completed != b.metrics.completed) {
            return a.metrics.completed > b.metrics.completed;
        }
        if (std::abs(a.metrics.completion_ratio - b.metrics.completion_ratio) > 1e-6) {
            return a.metrics.completion_ratio > b.metrics.completion_ratio;
        }
        if (std::abs(a.metrics.avg_speed - b.metrics.avg_speed) > 1e-6) {
            return a.metrics.avg_speed > b.metrics.avg_speed;
        }
        return a.metrics.max_abs_cte < b.metrics.max_abs_cte;
    });

    std::cout << "Baseline (launch-like defaults):\n";
    EvaluatedCandidate baseline_eval;
    baseline_eval.candidate = baseline;
    baseline_eval.metrics = runSimulation(trajectory, track_length, baseline, target_laps, kDefaultDt, verbose);
    baseline_eval.score = scoreCandidate(baseline_eval.metrics);
    printCandidate(baseline_eval);

    std::cout << "Conservative seed:\n";
    EvaluatedCandidate conservative_eval;
    conservative_eval.candidate = conservative;
    conservative_eval.metrics =
        runSimulation(trajectory, track_length, conservative, target_laps, kDefaultDt, verbose);
    conservative_eval.score = scoreCandidate(conservative_eval.metrics);
    printCandidate(conservative_eval);

    std::cout << "\nTop 5 candidates:\n";
    for (size_t i = 0; i < std::min<size_t>(5, all.size()); ++i) {
        printCandidate(all[i], i + 1);
    }

    const EvaluatedCandidate& winner = all.front();
    const bool feasible_winner = winner.metrics.completed && !winner.metrics.collided;
    if (feasible_winner) {
        std::cout << "\nRecommended parameters (best collision-free set):\n";
    } else {
        std::cout << "\nRecommended parameters (no collision-free set found; least-bad candidate):\n";
    }

    std::cout << std::fixed << std::setprecision(4)
              << "  k_e: " << winner.candidate.config.k_e << "\n"
              << "  k_h: " << winner.candidate.config.k_h << "\n"
              << "  k_s: " << winner.candidate.config.k_s << "\n"
              << "  k_d: " << winner.candidate.config.k_d << "\n"
              << "  max_speed: " << winner.candidate.config.max_speed << "\n"
              << "  min_speed: " << winner.candidate.config.min_speed << "\n"
              << "  speed_gain: " << winner.candidate.config.speed_gain << "\n"
              << "  feedforward_gain: " << winner.candidate.config.feedforward_gain << "\n"
              << "  curvature_speed_factor: " << winner.candidate.config.curvature_speed_factor << "\n"
              << "  max_steering_rate: " << winner.candidate.config.max_steering_rate << "\n";

    if (!feasible_winner) {
        std::cerr << "ERROR: Stanley sweep did not find a collision-free full-lap solution.\n";
        return 2;
    }

    return 0;
}
