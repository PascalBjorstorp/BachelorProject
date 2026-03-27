/**
 * @file test_sim_drive_pure_pursuit.cpp
 * @brief Closed-loop pure pursuit simulation and parameter sweep.
 *
 * This executable mirrors the intent of MPC/test/test_sim_drive.c for the
 * pure pursuit controller. It loads a raceline CSV with wall bounds,
 * simulates a kinematic bicycle plant, detects wall collisions from left/right
 * track bounds, and sweeps controller parameters to maximize average speed
 * while completing a target number of laps.
 *
 * Build:
 *   colcon build --packages-select f1tenth_control --symlink-install
 *
 * Run:
 *   ./build/f1tenth_control/test_sim_drive_pure_pursuit
 *   ./build/f1tenth_control/test_sim_drive_pure_pursuit --iterations 320 --laps 3
 *   ./build/f1tenth_control/test_sim_drive_pure_pursuit --trajectory /abs/path/to/raceline.csv
 */

#include "f1tenth_control/algorithms/pure_pursuit.hpp"

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

using f1tenth_control::PurePursuit;
using f1tenth_control::PurePursuitConfig;
using f1tenth_control::PurePursuitOutput;
using f1tenth_control::TrajectoryPoint;
using f1tenth_control::VehicleState;

constexpr double kDefaultDt = 0.01;                  // 100 Hz simulation
constexpr double kMaxSimTime = 240.0;                // seconds
constexpr double kSpeedTimeConstant = 0.30;          // seconds
constexpr double kSteerTimeConstant = 0.10;          // seconds
constexpr double kMaxSteeringRate = 2.85;            // rad/s
constexpr double kMaxAccel = 4.0;                    // m/s^2
constexpr double kMaxBrake = 6.0;                    // m/s^2
constexpr double kMaxLateralAccel = 7.31;            // m/s^2 (mu*g from measured params)
constexpr double kVehicleHalfWidth = 0.137;          // m
constexpr double kBodySafetyMargin = 0.020;          // m
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

struct Candidate {
    PurePursuitConfig config;
    double max_speed{2.0};
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
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
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

    if (out.size() < 2) {
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
        const int wrapped = (idx_raw % static_cast<int>(n) + static_cast<int>(n)) % static_cast<int>(n);
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

    std::vector<TrajectoryPoint> pp_traj;
    pp_traj.reserve(traj.size());
    for (const auto& w : traj) {
        pp_traj.push_back(w.pt);
    }

    PurePursuit controller(candidate.config);
    controller.setTrajectory(pp_traj);

    VehicleState state;
    state.pose.x = traj.front().pt.x;
    state.pose.y = traj.front().pt.y;
    state.pose.theta = traj.front().pt.heading;
    state.velocity = std::min(candidate.max_speed, std::max(0.5, traj.front().pt.velocity));

    double steer_actual = 0.0;
    size_t closest_idx = 0;
    double sum_cte_sq = 0.0;
    int cte_count = 0;

    for (double t = 0.0; t < kMaxSimTime; t += dt) {
        const PurePursuitOutput output = controller.compute(state);
        if (!output.valid) {
            break;
        }

        const double speed_cmd = clampValue(output.target_speed, 0.0, candidate.max_speed);
        const double steer_cmd = clampValue(output.steering_angle,
                                            -candidate.config.max_steering,
                                            candidate.config.max_steering);

        double steer_rate_cmd = (steer_cmd - steer_actual) / kSteerTimeConstant;
        steer_rate_cmd = clampValue(steer_rate_cmd, -kMaxSteeringRate, kMaxSteeringRate);
        steer_actual += steer_rate_cmd * dt;
        steer_actual = clampValue(steer_actual,
                                  -candidate.config.max_steering,
                                  candidate.config.max_steering);

        double accel_cmd = (speed_cmd - state.velocity) / kSpeedTimeConstant;
        accel_cmd = clampValue(accel_cmd, -kMaxBrake, kMaxAccel);
        state.velocity += accel_cmd * dt;
        state.velocity = clampValue(state.velocity, 0.0, candidate.max_speed);

        const double nominal_yaw_rate =
            state.velocity * std::tan(steer_actual) / candidate.config.wheelbase;
        const double nominal_lat_accel = std::abs(state.velocity * nominal_yaw_rate);
        double grip_scale = 1.0;
        if (nominal_lat_accel > kMaxLateralAccel) {
            grip_scale = kMaxLateralAccel / nominal_lat_accel;
        }

        const double yaw_rate = nominal_yaw_rate * grip_scale;
        state.angular_velocity = yaw_rate;
        state.steering_angle = steer_actual;

        const double theta = state.pose.theta;
        state.pose.x += state.velocity * std::cos(theta) * dt;
        state.pose.y += state.velocity * std::sin(theta) * dt;

        // Under high lateral demand, emulate understeer by drifting outward.
        if (grip_scale < 0.999) {
            const double slip_speed = state.velocity * (1.0 - grip_scale);
            const double nx = -std::sin(theta);
            const double ny = std::cos(theta);
            const double outward_sign = (steer_actual >= 0.0) ? -1.0 : 1.0;
            state.pose.x += outward_sign * nx * slip_speed * dt;
            state.pose.y += outward_sign * ny * slip_speed * dt;
        }

        state.pose.theta = normalizeAngle(theta + yaw_rate * dt);

        const size_t prev_idx = closest_idx;
        closest_idx = findClosestIndex(traj, state, closest_idx);
        metrics.traveled_distance += advanceAlongTrack(traj, prev_idx, closest_idx);

        const double cte = crossTrackError(traj[closest_idx], state);
        metrics.max_abs_cte = std::max(metrics.max_abs_cte, std::abs(cte));
        sum_cte_sq += cte * cte;
        ++cte_count;

        const double allowed_left = traj[closest_idx].left_bound - (kVehicleHalfWidth + kBodySafetyMargin);
        const double allowed_right = traj[closest_idx].right_bound - (kVehicleHalfWidth + kBodySafetyMargin);
        if ((allowed_left > 0.0 && cte > allowed_left) ||
            (allowed_right > 0.0 && -cte > allowed_right)) {
            metrics.collided = true;
            metrics.sim_time = t;
            break;
        }

        const double laps_f = (track_length > 1e-6) ? (metrics.traveled_distance / track_length) : 0.0;
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
    if (m.completed && !m.collided) {
        const double tracking_penalty = 2.5 * m.max_abs_cte + 1.5 * m.rms_cte;
        return m.avg_speed - tracking_penalty;
    }

    double penalty = 0.0;
    if (m.collided) {
        penalty += 1.0;
    }
    const double tracking_penalty = 2.5 * m.max_abs_cte + 1.5 * m.rms_cte;
    return m.avg_speed * m.completion_ratio - penalty - tracking_penalty;
}

Candidate launchLikeBaseline() {
    Candidate c;
    c.config.min_lookahead = 0.49;
    c.config.max_lookahead = 1.67;
    c.config.lookahead_gain = 0.16;
    c.config.cte_lookahead_weight = 1.0;
    c.config.cte_lookahead_gain = 0.07;
    c.config.curvature_lookahead_gain = 0.07;
    c.config.curvature_speed_factor = 0.48;
    c.config.curvature_speed_floor_ratio = 0.36;
    c.config.max_steering = 0.4189;
    c.config.wheelbase = 0.3302;
    c.config.position_tolerance = 0.5;
    c.max_speed = 5.0;
    return c;
}

Candidate conservativeSeed() {
    Candidate c;
    c.config.min_lookahead = 0.60;
    c.config.max_lookahead = 1.80;
    c.config.lookahead_gain = 0.10;
    c.config.cte_lookahead_weight = 1.0;
    c.config.cte_lookahead_gain = 0.03;
    c.config.curvature_lookahead_gain = 0.06;
    c.config.curvature_speed_factor = 0.55;
    c.config.curvature_speed_floor_ratio = 0.50;
    c.config.max_steering = 0.4189;
    c.config.wheelbase = 0.3302;
    c.config.position_tolerance = 0.5;
    c.max_speed = 2.4;
    return c;
}

Candidate sampleRandom(std::mt19937& rng) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    Candidate c;
    c.config.min_lookahead = 0.40 + 0.60 * u01(rng);
    c.config.max_lookahead = c.config.min_lookahead + 0.70 + 1.20 * u01(rng);
    c.config.max_lookahead = std::min(c.config.max_lookahead, 2.60);
    c.config.lookahead_gain = 0.06 + 0.10 * u01(rng);
    c.config.cte_lookahead_weight = 1.0;
    c.config.cte_lookahead_gain = 0.00 + 0.08 * u01(rng);
    c.config.curvature_lookahead_gain = 0.00 + 0.16 * u01(rng);
    c.config.curvature_speed_factor = 0.25 + 0.85 * u01(rng);
    c.config.curvature_speed_floor_ratio = 0.35 + 0.35 * u01(rng);
    c.config.max_steering = 0.4189;
    c.config.wheelbase = 0.3302;
    c.config.position_tolerance = 0.5;
    c.max_speed = 1.8 + 1.5 * u01(rng);
    return c;
}

Candidate sampleAround(const Candidate& base, std::mt19937& rng) {
    std::normal_distribution<double> n01(0.0, 1.0);

    Candidate c = base;
    c.config.min_lookahead = clampValue(base.config.min_lookahead + 0.06 * n01(rng), 0.35, 1.20);
    c.config.max_lookahead = clampValue(base.config.max_lookahead + 0.15 * n01(rng), 1.0, 2.8);
    if (c.config.max_lookahead < c.config.min_lookahead + 0.45) {
        c.config.max_lookahead = c.config.min_lookahead + 0.45;
    }
    c.config.lookahead_gain = clampValue(base.config.lookahead_gain + 0.015 * n01(rng), 0.04, 0.20);
    c.config.cte_lookahead_gain = clampValue(base.config.cte_lookahead_gain + 0.012 * n01(rng), 0.0, 0.12);
    c.config.curvature_lookahead_gain = clampValue(base.config.curvature_lookahead_gain + 0.020 * n01(rng), 0.0, 0.25);
    c.config.curvature_speed_factor = clampValue(base.config.curvature_speed_factor + 0.08 * n01(rng), 0.10, 1.30);
    c.config.curvature_speed_floor_ratio = clampValue(base.config.curvature_speed_floor_ratio + 0.05 * n01(rng), 0.20, 0.80);
    c.max_speed = clampValue(base.max_speed + 0.10 * n01(rng), 1.5, 3.4);
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
              << " cte_max=" << e.metrics.max_abs_cte
              << " cfg{"
              << "vmax=" << e.candidate.max_speed
              << ", minL=" << e.candidate.config.min_lookahead
              << ", maxL=" << e.candidate.config.max_lookahead
              << ", kL=" << e.candidate.config.lookahead_gain
              << ", kCte=" << e.candidate.config.cte_lookahead_gain
              << ", kCurvL=" << e.candidate.config.curvature_lookahead_gain
              << ", kCurvV=" << e.candidate.config.curvature_speed_factor
              << ", floor=" << e.candidate.config.curvature_speed_floor_ratio
              << "}"
              << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string trajectory_path =
        "/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_planning/trajectories/hardware_raceline.csv";
    int iterations = 260;
    int target_laps = 3;
    int seed = 42;
    bool verbose = false;
    double fixed_max_speed = -1.0;

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
        } else if (arg == "--fixed-max-speed" && i + 1 < argc) {
            fixed_max_speed = std::max(0.1, std::atof(argv[++i]));
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--help") {
            std::cout << "Usage: test_sim_drive_pure_pursuit [options]\n"
                      << "  --trajectory <csv_path>\n"
                      << "  --iterations <N>\n"
                      << "  --laps <N>\n"
                      << "  --seed <N>\n"
                      << "  --fixed-max-speed <mps>\n"
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
              << " (track_length=" << std::fixed << std::setprecision(2) << track_length << " m)\n";
    std::cout << "Sweep settings: iterations=" << iterations
              << " laps=" << target_laps
              << " seed=" << seed
              << " dt=" << kDefaultDt << "\n\n";

    if (fixed_max_speed > 0.0) {
        std::cout << "Using fixed max_speed=" << fixed_max_speed
                  << " for all evaluated candidates\n\n";
    }

    std::vector<EvaluatedCandidate> all;
    all.reserve(static_cast<size_t>(iterations) + 8);

    const Candidate baseline = launchLikeBaseline();
    const Candidate conservative = conservativeSeed();

    Candidate baseline_effective = baseline;
    Candidate conservative_effective = conservative;
    if (fixed_max_speed > 0.0) {
        baseline_effective.max_speed = fixed_max_speed;
        conservative_effective.max_speed = fixed_max_speed;
    }

    SimMetrics baseline_metrics = runSimulation(
        trajectory, track_length, baseline_effective, target_laps, kDefaultDt, verbose);
    SimMetrics conservative_metrics = runSimulation(
        trajectory, track_length, conservative_effective, target_laps, kDefaultDt, verbose);

    EvaluatedCandidate best;
    best.score = -1e9;
    Candidate best_completed_seed = conservative;
    bool have_completed = false;
    double best_completed_avg_speed = -1.0;

    auto evaluate = [&](const Candidate& candidate) {
        Candidate effective = candidate;
        if (fixed_max_speed > 0.0) {
            effective.max_speed = fixed_max_speed;
        }

        EvaluatedCandidate e;
        e.candidate = effective;
        e.metrics = runSimulation(trajectory, track_length, effective, target_laps, kDefaultDt, false);
        e.score = scoreCandidate(e.metrics);
        all.push_back(e);

        if (e.score > best.score) {
            best = e;
        }
        if (e.metrics.completed && !e.metrics.collided) {
            if (!have_completed || e.metrics.avg_speed > best_completed_avg_speed) {
                best_completed_seed = e.candidate;
                best_completed_avg_speed = e.metrics.avg_speed;
            }
            have_completed = true;
        }
    };

    evaluate(baseline);
    evaluate(conservative);

    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    for (int i = 0; i < iterations; ++i) {
        evaluate(sampleRandom(rng));
    }

    if (have_completed) {
        for (int i = 0; i < iterations / 2; ++i) {
            evaluate(sampleAround(best_completed_seed, rng));
        }
    }

    std::sort(all.begin(), all.end(), [](const EvaluatedCandidate& a, const EvaluatedCandidate& b) {
        if (a.metrics.completed != b.metrics.completed) {
            return a.metrics.completed > b.metrics.completed;
        }
        if (std::abs(a.score - b.score) > 1e-6) {
            return a.score > b.score;
        }
        if (std::abs(a.metrics.avg_speed - b.metrics.avg_speed) > 1e-6) {
            return a.metrics.avg_speed > b.metrics.avg_speed;
        }
        return a.metrics.max_abs_cte < b.metrics.max_abs_cte;
    });

    std::cout << "Baseline (launch-like defaults):\n";
    EvaluatedCandidate baseline_eval;
    baseline_eval.candidate = baseline_effective;
    baseline_eval.metrics = baseline_metrics;
    baseline_eval.score = scoreCandidate(baseline_eval.metrics);
    printCandidate(baseline_eval);

    std::cout << "Conservative seed:\n";
    EvaluatedCandidate conservative_eval;
    conservative_eval.candidate = conservative_effective;
    conservative_eval.metrics = conservative_metrics;
    conservative_eval.score = scoreCandidate(conservative_eval.metrics);
    printCandidate(conservative_eval);

    std::cout << "\nTop 5 candidates:\n";
    for (size_t i = 0; i < std::min<size_t>(5, all.size()); ++i) {
        printCandidate(all[i], i + 1);
    }

    const EvaluatedCandidate& winner = all.front();
    std::cout << "\nRecommended parameters (best completed lap set):\n";
    std::cout << std::fixed << std::setprecision(4)
              << "  max_speed: " << winner.candidate.max_speed << "\n"
              << "  min_lookahead: " << winner.candidate.config.min_lookahead << "\n"
              << "  max_lookahead: " << winner.candidate.config.max_lookahead << "\n"
              << "  lookahead_gain: " << winner.candidate.config.lookahead_gain << "\n"
              << "  cte_lookahead_gain: " << winner.candidate.config.cte_lookahead_gain << "\n"
              << "  curvature_lookahead_gain: " << winner.candidate.config.curvature_lookahead_gain << "\n"
              << "  curvature_speed_factor: " << winner.candidate.config.curvature_speed_factor << "\n"
              << "  curvature_speed_floor_ratio: " << winner.candidate.config.curvature_speed_floor_ratio << "\n";

    return 0;
}
