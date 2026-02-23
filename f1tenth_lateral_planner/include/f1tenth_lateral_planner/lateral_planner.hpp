#ifndef F1TENTH_LATERAL_PLANNER_LATERAL_PLANNER_HPP_
#define F1TENTH_LATERAL_PLANNER_LATERAL_PLANNER_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace f1tenth_lateral_planner {

/**
 * @brief A single waypoint from the global raceline CSV.
 * Columns: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
 */
struct Waypoint {
    double s{0.0};      ///< Arc-length along raceline [m]
    double x{0.0};      ///< X position [m]
    double y{0.0};      ///< Y position [m]
    double psi{0.0};    ///< Heading angle [rad]
    double kappa{0.0};  ///< Curvature [1/m]
    double vx{0.0};     ///< Target speed [m/s]
    double ax{0.0};     ///< Target acceleration [m/s²]
};

/**
 * @brief Tunable parameters for the lateral planner.
 */
struct LateralPlannerConfig {
    double safety_margin_m{0.35};     ///< Extra clearance added to opponent half-width [m]
    double min_window_m{4.0};         ///< Minimum avoidance half-window length [m]
    double window_time_s{2.0};        ///< half_window = max(min_window, speed * this) [s]
    double max_lateral_shift_m{0.9};  ///< Maximum allowed lateral raceline offset [m]
    double min_replan_dist_m{0.3};    ///< Ignore opponent if closer than this [m]
    double blend_rate{0.10};          ///< Per-cycle smoothing factor for d_max (0=frozen, 1=instant)
};

/**
 * @brief Output of the planner: arrays of (x, y, psi, vx) for each raceline point.
 */
struct PlannerOutput {
    std::vector<double> xs;    ///< X positions [m]
    std::vector<double> ys;    ///< Y positions [m]
    std::vector<double> psis;  ///< Heading angles [rad]
    std::vector<double> vxs;   ///< Target speeds [m/s]
};

/**
 * @brief LateralPlanner – pure algorithm class (no ROS dependencies).
 *
 * Responsibilities (one method per concern):
 *  - loadRaceline()         : parse TUM-format CSV and cache waypoints
 *  - findClosestWaypoint()  : fast O(1) local search with global fallback
 *  - decidePassDirection()  : choose left/right pass based on opponent lateral offset
 *  - computeOffsets()       : cosine-blended lateral shift for the avoidance window
 *  - applyOffsets()         : displace raceline x,y by offset along its normal vector
 *  - fullRaceline()         : return the unmodified global raceline as PlannerOutput
 *  - update()               : top-level call — returns the path to follow this cycle
 */
class LateralPlanner {
public:
    LateralPlanner() = default;
    explicit LateralPlanner(const LateralPlannerConfig& config);

    /**
     * @brief Load the global raceline from a TUM-format CSV file.
     * Expected header (optional, skipped with '#'): s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
     * @return true on success
     */
    bool loadRaceline(const std::string& csv_path);

    /**
     * @brief Find the index of the closest waypoint to (x, y).
     * Performs a local search around `hint` first; falls back to a full scan
     * if the local minimum is on a boundary.
     */
    size_t findClosestWaypoint(double x, double y, size_t hint = 0) const;

    /**
     * @brief Main planning update.
     *
     * When no obstacle is detected (or the opponent has been passed), returns
     * the unshifted global raceline.  When an obstacle is ahead, computes a
     * cosine-blended lateral shift that smoothly deviates around the opponent
     * and then rejoins the original raceline.
     *
     * @param robot_x/y      Current robot position in map frame [m]
     * @param opp_detected   True when the obstacle filter sees an opponent
     * @param opp_x/y        Opponent centroid in map frame [m]
     * @param opp_width      Estimated opponent width [m]
     * @param current_speed  Current robot speed [m/s]
     * @param robot_hint     In/out: last known robot waypoint index (speeds up search)
     * @param opp_hint       In/out: last known opponent waypoint index
     * @return PlannerOutput Path arrays to publish
     */
    PlannerOutput update(
        double robot_x, double robot_y,
        bool opp_detected,
        double opp_x, double opp_y, double opp_width,
        double current_speed,
        size_t& robot_hint,
        size_t& opp_hint);

    // ── Accessors ────────────────────────────────────────────────────

    bool hasRaceline() const { return !raceline_.empty(); }
    size_t size() const { return raceline_.size(); }
    double totalLength() const;

    void setConfig(const LateralPlannerConfig& cfg) { config_ = cfg; }
    const LateralPlannerConfig& getConfig() const { return config_; }
    const std::vector<Waypoint>& getRaceline() const { return raceline_; }

private:
    LateralPlannerConfig config_;
    std::vector<Waypoint> raceline_;

    // ── Avoidance state machine ──────────────────────────────────────
    bool avoidance_active_{false};
    double locked_pass_dir_{0.0};  ///< +1 = pass left, -1 = pass right
    double locked_opp_s_{0.0};     ///< Arc-length of opponent when direction was locked
    double smooth_d_max_{0.0};     ///< Temporally-smoothed shift magnitude [m]

    /**
     * @brief Determine which side to pass based on the opponent's lateral
     *        offset from the nearest raceline tangent.
     * @return +1.0 (pass on left) or -1.0 (pass on right)
     */
    double decidePassDirection(double opp_x, double opp_y, size_t opp_idx) const;

    /**
     * @brief Build a per-waypoint lateral offset array using a cosine blend.
     * The path ramps up from 0 to d_max between the robot and the opponent,
     * then ramps back down to 0 over `half_window` metres past the opponent.
     */
    std::vector<double> computeOffsets(
        double robot_s, double opp_s,
        double total_s, double d_max,
        double current_speed) const;

    /**
     * @brief Shift the raceline perpendicular to its heading by `offsets`.
     */
    PlannerOutput applyOffsets(const std::vector<double>& offsets) const;

    /**
     * @brief Copy the global raceline into a PlannerOutput without any shift.
     */
    PlannerOutput fullRaceline() const;
};

}  // namespace f1tenth_lateral_planner

#endif  // F1TENTH_LATERAL_PLANNER_LATERAL_PLANNER_HPP_
