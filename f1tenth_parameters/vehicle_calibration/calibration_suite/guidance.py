"""Operator-facing guidance for every unified calibration stage.

This metadata is deliberately separate from the execution code.  The GUI may
explain a manoeuvre, but :mod:`runner` remains the only authority that can
launch it, enforce dependencies, analyse it, or accept it.
"""
from __future__ import annotations

from typing import Any


COMMON_DATA = [
    "Native 1080-point LiDAR scans at approximately 40 Hz",
    "VESC telemetry and wheel odometry",
    "VESC IMU acceleration and yaw rate",
    "Command, selector, bus and actuator echoes",
    "Timestamped trial decisions and safety events",
]

FIGURE_BY_CATEGORY = {
    "stationary": "stand",
    "manual": "metrology",
    "motion": "straight",
    "dynamic": "straight",
    "turning": "turning",
}


def _guide(
    title: str,
    summary: str,
    category: str,
    role: str,
    setup: list[str],
    during: list[str],
    expect: list[str],
    outputs: list[str],
    *,
    data: list[str] | None = None,
    setup_figure: str | None = None,
) -> dict[str, Any]:
    return {
        "title": title,
        "summary": summary,
        "category": category,
        "setup_figure": setup_figure or FIGURE_BY_CATEGORY.get(category, "straight"),
        "abc_role": role,
        "setup": setup,
        "during": during,
        "expect": expect,
        "outputs": outputs,
        "data": list(COMMON_DATA if data is None else data),
    }


STAGE_GUIDANCE: dict[str, dict[str, Any]] = {
    "steering_command_audit": _guide(
        "Steering command-chain audit",
        "Proves that one raw-servo command reaches the selector, VESC bus and servo echo without another controller interfering.",
        "stationary", "Safety prerequisite",
        ["Put the car securely on a stand; driven wheels and steering must be clear.", "Stop teleoperation, planners, MPC/MPCC and every other command publisher."],
        ["The servo makes small movements around the deployed centre.", "Respond to the terminal prompt through the GUI."],
        ["All command echoes follow the requested values within tolerance.", "The car remains stationary and no motor demand is issued."],
        ["Command ownership and echo-error audit", "Stationary capture-coverage plot"],
        data=["Raw-servo request, selector output, VESC command bus and servo echo", "VESC/IMU/odometry health channels", "Timestamped audit events"],
    ),
    "physical_metrology": _guide(
        "Direct physical vehicle metrology",
        "Validates race-ready mass, wheelbase and axle loads, then derives the CoG-to-axle distances used by later models.",
        "manual", "Direct measurement",
        ["Install the race battery, body, sensors and tyres used for testing.", "Keep all four wheel contact points at equal height while weighing axle loads.", "Complete every red field in the preparation form."],
        ["No vehicle motion or ROS bag is required.", "The suite checks SI units and mass-versus-axle-load closure; no standard-deviation entry is requested."],
        ["Front plus rear axle load agrees with mass × gravity.", "Sensor geometry is imported from deployed vehicle_geometry, not entered again."],
        ["Derived l_f/l_r and CoG location", "Physical-parameter YAML, plot and LaTeX page", "Reversible dynamic-geometry build"],
        data=["Measurement sheet with values, method and provenance", "Authoritative deployed LiDAR/IMU/base_link geometry"],
    ),
    "steering_observability": _guide(
        "LiDAR/IMU observability check",
        "Checks that the actual room and full-resolution LiDAR provide reliable motion windows before any steering value is fitted.",
        "motion", "Sensor prerequisite",
        ["Use the 14 × 14 m room, keep the 1 m wall band clear, and make fixed asymmetric features visible.", "Start on the marked 45° diagonal lane."],
        ["Stationary scan diagnostics, repeated short straights and gentle left/right turns are collected.", "Reposition when prompted and review every pass."],
        ["Every LiDAR scan has at least 1080 points and median rate ≥35 Hz.", "Windowed registrations pass speed, validity and RMSE gates."],
        ["LiDAR scan-resolution report", "Windowed ICP observability metrics and coverage plot"],
        setup_figure="observability",
    ),
    "steering_centre": _guide(
        "Steering offset training and fit",
        "Uses directed IMU/odometry probes and a fine LiDAR grid to find the raw servo value with zero curvature.",
        "motion", "A/B training and candidate update",
        ["Mark a repeatable straight lane and clear its full stopping envelope.", "Do not enable heading assist; visible drift must remain observable."],
        ["The car runs several nearby raw-servo values.", "IMU/odom choose the correction direction; LiDAR supplies the accepted fine fit."],
        ["Yaw changes sign across the measured grid.", "The fitted zero lies inside the bracket with acceptable R²/residuals."],
        ["Candidate servo centre", "LiDAR/IMU/odom comparison plot", "Temporary centre patch and rebuild"],
    ),
    "steering_centre_validation": _guide(
        "Independent straightness validation",
        "Tests the frozen centre on new outbound/inbound passes at different speeds without refitting it.",
        "motion", "C independent validation",
        ["Use the same clear lane but fresh passes.", "Be ready to reject a run if the car visibly moves laterally."],
        ["Three 0.5 m/s outbound and three 0.8 m/s inbound passes are requested.", "Observe physical straightness as well as the automatic gates."],
        ["Near-zero LiDAR yaw, lateral velocity and heading change.", "A visibly biased candidate fails even if odometry appears straight."],
        ["Per-pass straightness bars", "Frozen-candidate validation result and retry/REDO guidance"],
    ),
    "steering_endstops": _guide(
        "Human steering-limit survey",
        "Records mechanically free raw-servo limits and physically observed road-wheel angles before full steering sweeps.",
        "stationary", "Manual safety prerequisite",
        ["Secure the car on a stand with steering linkage visible and unloaded.", "Have an angle gauge/protractor ready; keep fingers clear."],
        ["Use the displayed single-key controls to approach each mechanical end carefully.", "Stop before binding, buzzing or linkage contact, then enter observed wheel angles."],
        ["Both safe limits enclose the validated centre.", "Observed angles remain within the reviewed room-turn bound."],
        ["Safe raw-servo range", "Human-observed left/right road-wheel limits"],
        data=["Servo command/echo/bus values", "Operator-confirmed mechanical limits and measured wheel angles"],
    ),
    "steering_static_training": _guide(
        "Servo-to-steering training",
        "Measures the effective road-wheel steering angle over both sides and sweep directions using rear-axle LiDAR motion.",
        "turning", "A/B training and candidate update",
        ["Use the clear central turning area and verify the end-stop stage passed.", "Tyres and floor condition should match later validation."],
        ["Settings that fit record one complete left/right circle; the shallowest non-fitting setting uses a room-limited arc.", "Reposition and accept/retry each requested condition."],
        ["Balanced coverage, repeatability and hysteresis within limits.", "The selected map is deployable; material nonlinearity stops the campaign for a code upgrade."],
        ["Servo-versus-effective-angle plot", "Linear/nonlinear model comparison", "Temporary steering-map patch and rebuild"],
    ),
    "steering_static_holdout": _guide(
        "Servo-to-steering hold-out",
        "Validates the frozen steering map at different, shuffled commands that were not used for fitting.",
        "turning", "C independent validation",
        ["Keep the same tyre, load and surface state as training.", "Clear the complete arc envelope."],
        ["Run the shuffled hold-out fractions on both steering sides; fitting conditions record a complete circle."],
        ["Low angle RMSE/bias with no training-condition reuse.", "A materially better unsupported nonlinear map blocks continuation."],
        ["Hold-out map overlay", "Frozen-map validation and model-selection result"],
    ),
    "steering_response": _guide(
        "Steering response characterisation",
        "Measures combined command-to-vehicle delay, rise and settling behavior over several speeds and deflections.",
        "turning", "A/B training and candidate update",
        ["Clear enough area for the commanded step and return at each speed.", "Check steering linkage and battery state."],
        ["The car holds centre, applies the step for one complete fitting circle, then returns to centre."],
        ["Enough valid transients for delay/time-constant estimates.", "LiDAR response follows command with bounded fit error."],
        ["Per-condition timing metrics", "Combined actuator/vehicle response plot and candidate"],
    ),
    "steering_response_validation": _guide(
        "Steering response hold-out",
        "Checks response predictions on different speed/step combinations without refitting.",
        "turning", "C independent validation",
        ["Keep hardware, tyres and surface unchanged from response training.", "Clear each step-and-return envelope."],
        ["Run the distinct validation step grid; each fitting step records a complete circle before returning."],
        ["Settling time and normalized prediction error remain within gates."],
        ["Independent timing/prediction validation", "Response coverage plot"],
    ),
    "motor_command_audit": _guide(
        "Motor command-chain audit",
        "Proves exclusive motor selector ownership and command/echo routing before any powered longitudinal test.",
        "stationary", "Safety prerequisite",
        ["Put the car securely on a stand with driven wheels completely clear.", "Stop teleoperation, planners and all other motor publishers."],
        ["Small controlled command changes are sent while stationary/on-stand."],
        ["Exactly one selector publisher and matching selected command echoes.", "No unexplained motor command reaches the bus."],
        ["Motor command ownership and echo audit", "Stationary coverage plot"],
    ),
    "longitudinal_observability": _guide(
        "Longitudinal sensor observability",
        "Checks LiDAR/IMU quality from stationary through low, medium and high straight passes before speed fitting.",
        "motion", "Sensor prerequisite",
        ["Use the full clear straight and mark start/stop points.", "Confirm the validated steering centre is active."],
        ["A stationary diagnostic and repeated straight probes are recorded; each speed gets the longest useful room-safe duration."],
        ["Reliable full-resolution moving LiDAR windows over the intended speed range.", "Stationary IMU values are diagnostic only, never reused as a later bias correction."],
        ["Longitudinal observability report", "LiDAR/IMU noise and coverage plot"],
    ),
    "low_speed_launch": _guide(
        "Low-speed launch characterisation",
        "Finds the first repeatable raw-ERPM command that creates measurable ground motion without fitting the full map.",
        "motion", "Characterisation prerequisite",
        ["Use the straight-line start with room for repeated low-speed launches."],
        ["Several low raw-ERPM targets are repeated; slow captures use nearly the full diagonal and can last up to 30 s."],
        ["Repeatable LiDAR ground motion and a defensible launch threshold."],
        ["First repeatable launch condition", "Low-speed coverage and variability"],
    ),
    "erpm_map_training": _guide(
        "ERPM-to-ground-speed training",
        "Fits wheel-odometry ERPM conversion from independent LiDAR ground speed over the room-limited speed grid.",
        "motion", "A/B training and candidate update",
        ["Use the full straight with stopping area and validated steering centre.", "Check battery voltage and tyre state."],
        ["Repeated raw-ERPM plateaus are collected; duration is longer at low speed and automatically shortened at high speed."],
        ["Strong speed coverage and a stable zero-intercept fit.", "If nonlinear mapping is materially better, the suite requests a supported model change."],
        ["ERPM-versus-LiDAR-speed plot", "Candidate speed/odometry map and reversible rebuild"],
    ),
    "erpm_map_holdout": _guide(
        "ERPM-to-ground-speed hold-out",
        "Validates the frozen speed map on distinct speed cells without refitting.",
        "motion", "C independent validation",
        ["Keep battery, gearing and tyres unchanged from map training.", "Use the same clear straight."],
        ["Run the independent raw-ERPM hold-out targets using their frozen room-derived durations."],
        ["Low ground-speed RMSE/bias across the hold-out range."],
        ["Frozen-map hold-out metrics", "Independent ERPM-speed plot"],
    ),
    "vel_to_erpm_audit": _guide(
        "Velocity-to-ERPM pipeline audit",
        "Checks that normal velocity commands traverse the rebuilt Ackermann-to-VESC path and realise the expected ERPM/ground speed.",
        "motion", "Applied-model audit",
        ["Use the clear straight and keep the validated speed candidate active through the suite."],
        ["Normal velocity commands—not raw ERPM—are issued over a fresh grid with speed-specific diagonal capture times."],
        ["Command routing and realised ground speed agree with the frozen map."],
        ["End-to-end VEL_TO_ERPM audit and coverage plot"],
    ),
    "erpm_response": _guide(
        "ERPM response characterisation",
        "Measures drivetrain/wheel-speed response timing for several raw-ERPM steps.",
        "dynamic", "A/B training and candidate update",
        ["Use the full straight and verify stopping/braking area is clear.", "Monitor battery and motor temperature."],
        ["The car establishes an initial speed, receives a raw-ERPM step, then stops."],
        ["Repeatable response delay/time constant and sufficient transient samples."],
        ["Step-response timing fit", "Per-condition response coverage"],
    ),
    "erpm_response_validation": _guide(
        "ERPM response hold-out",
        "Validates the frozen response model with different ERPM steps.",
        "dynamic", "C independent validation",
        ["Keep drivetrain and battery condition comparable to response training.", "Clear the full acceleration and stopping envelope."],
        ["Run the distinct response steps and review each transient."],
        ["Prediction errors and timing ratios pass without refitting."],
        ["Independent response validation and plot"],
    ),
    "coastdown": _guide(
        "Coast-down drag fit",
        "Fits effective constant, linear and quadratic resistance from zero-current speed decay trajectories.",
        "motion", "A/B training and candidate update",
        ["Use a level straight with consistent floor condition.", "Ensure the drivetrain rolls freely and steering is centred."],
        ["The car reaches each initial speed, motor current goes to zero, and LiDAR records the longest room-safe natural decay."],
        ["Monotonic usable coast-down trajectories and a well-conditioned resistance fit."],
        ["Measured/model speed trajectories", "Effective drag candidate and reversible rebuild"],
    ),
    "coastdown_validation": _guide(
        "Coast-down drag hold-out",
        "Tests the frozen drag curve from different initial speeds.",
        "motion", "C independent validation",
        ["Keep floor, tyres, gearing and vehicle load unchanged."],
        ["Run independent zero-current coast-down trajectories with speed-specific room-safe durations."],
        ["Frozen model predicts the speed decay within hold-out gates."],
        ["Hold-out trajectory overlay and drag validation"],
    ),
    "current_training": _guide(
        "Current-to-acceleration training",
        "Fits effective drive/brake current-to-ground-acceleration gains while diagnosing speed dependence and traction memory.",
        "dynamic", "A/B training and candidate update",
        ["Use the clearest straight and verify tyres, drivetrain, battery and VESC current limits.", "Be prepared for the strongest acceleration/braking pulses in the campaign."],
        ["Short controlled drive and brake current pulses are repeated at several initial speeds."],
        ["LiDAR acceleration is measurable, current echoes are correct and slip gates remain valid.", "Unsupported nonlinear or dynamic traction behavior stops the campaign for a model upgrade."],
        ["Current-versus-ground-acceleration plot", "Drive/brake gain candidate and nonlinear diagnostics"],
    ),
    "current_holdout": _guide(
        "Current-to-acceleration hold-out",
        "Validates frozen drive/brake gains at different speed/current cells.",
        "dynamic", "C independent validation",
        ["Keep battery SOC, tyres and floor comparable to current training.", "Clear acceleration and braking envelopes."],
        ["Run the distinct drive/brake hold-out pulses."],
        ["Low acceleration prediction error without material speed/current surface advantage."],
        ["Independent current-model validation and plot"],
    ),
    "accel_interface": _guide(
        "Acceleration-command interface audit",
        "Tests the rebuilt racing ACCEL_TO_CURRENT route from requested acceleration to current and measured ground acceleration.",
        "dynamic", "A/B applied-interface training",
        ["Use the clear straight and verify the current model passed hold-out.", "Confirm normal racing command ownership is otherwise stopped."],
        ["Ackermann acceleration commands are issued from several initial speeds."],
        ["Correct command routing, current sign/gain and measured ground acceleration."],
        ["Requested-versus-observed acceleration plot", "Applied interface audit and candidate evidence"],
    ),
    "accel_interface_validation": _guide(
        "Acceleration interface hold-out",
        "Validates the frozen ACCEL_TO_CURRENT route on distinct speeds and acceleration commands.",
        "dynamic", "C independent validation",
        ["Keep battery and traction state comparable to interface training.", "Clear the full acceleration/braking envelope."],
        ["Run the independent acceleration-command grid."],
        ["Observed ground acceleration agrees with the request within validation gates."],
        ["Independent acceleration-interface plot and validation"],
    ),
    "odometry_candidate_velocity_validation": _guide(
        "Selected odometry speed hold-out",
        "Runs the selected command map and causal shadow odometry on fresh speeds that were not used to choose the model.",
        "motion", "C independent model validation",
        ["Use the marked straight lane and leave the full braking margin clear.", "Keep battery, tyres and load comparable to the A/B longitudinal stages."],
        ["The shadow model—not production odometry—drives four new speed levels; LiDAR records each for its longest useful diagonal duration."],
        ["Every speed cell has complete coverage.", "Both commanded speed and shadow odometry pass LiDAR RMSE, bias and p95 gates."],
        ["Fresh speed-hold overlay", "Selected-model velocity validation report"],
    ),
    "odometry_candidate_accel_validation": _guide(
        "Selected odometry transient hold-out",
        "Tests the frozen causal odometry and acceleration mapper on fresh drive, hold-speed and brake pulses.",
        "dynamic", "C independent model validation",
        ["Use the longest clear straight and verify the active-stop envelope.", "Check battery voltage, motor temperature and tyre condition."],
        ["The shadow estimator runs distinct initial-speed/acceleration cells; no coefficient is refitted."],
        ["Ground-speed and acceleration errors pass in drive, braking and zero-acceleration hold regimes.", "A failure returns to fresh current training instead of repeating unchanged C data."],
        ["Dynamic shadow-versus-LiDAR plot", "Final causal-odometry validation report"],
    ),
    "lateral_stiffness_training": _guide(
        "Lateral model training",
        "Fits low-slip tyre stiffness, dynamic steering scale and cornering wheel-speed correction from balanced arcs.",
        "turning", "A/B training and candidate update",
        ["Use the clear central area with unchanged tyres, load and floor.", "Inspect tyres and steering linkage before repeated arcs."],
        ["Each balanced left/right speed-and-angle condition starts on its marked circle, settles, then records one complete revolution."],
        ["Quasi-steady force/slip and ERPM/IMU speed data cover both signs.", "If a nonlinear tyre or unsupported odometry model wins materially, promotion stops for implementation."],
        ["Front/rear force and cornering wheel-slip plots", "C_f/C_r, steering scale, wheel-slip coefficient with confidence intervals", "Reversible runtime rebuild"],
    ),
    "lateral_stiffness_validation": _guide(
        "Effective tyre-stiffness hold-out",
        "Tests the frozen dynamic bicycle and cornering-speed parameters on independent arcs.",
        "turning", "C independent validation",
        ["Keep vehicle load, tyres and surface unchanged from stiffness training.", "Clear every requested arc envelope."],
        ["Run one complete circle for every distinct speed/steering hold-out condition."],
        ["Frozen tyre, steering and wheel-slip values predict new force/speed behavior.", "Runtime odometry matches the frozen correction and no unsupported nonlinear tyre model wins."],
        ["Independent tyre-force and cornering-speed plot", "Final dynamic-model/runtime validation gate"],
    ),
}

# Observability deliberately uses an asymmetric-feature layout rather than the
# normal straight-lane illustration: repeatable windowed registration needs
# geometry on both sides and at different ranges.
STAGE_GUIDANCE["steering_observability"]["setup_figure"] = "observability"
STAGE_GUIDANCE["longitudinal_observability"]["setup_figure"] = "observability"


def guide_for(stage_key: str) -> dict[str, Any]:
    """Return a copy so callers cannot mutate the shared operator contract."""
    guide = STAGE_GUIDANCE.get(stage_key)
    if guide is None:
        return _guide(
            stage_key.replace("_", " ").title(),
            "Collect and analyse the stage-specific calibration evidence.",
            "motion", "Calibration stage",
            ["Read the live runner checklist before continuing."],
            ["Follow every reposition and trial-review prompt."],
            ["All automatic and operator gates pass."],
            ["Stage report, plot and cumulative LaTeX page"],
        )
    return {key: list(value) if isinstance(value, list) else value for key, value in guide.items()}
