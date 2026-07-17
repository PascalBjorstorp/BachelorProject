# Unified vehicle calibration

This is the canonical calibration entry point for the car. It replaces the old whole-ERPM / whole-steering workflow with a dependency-ordered, room-limited campaign. The legacy `steering/` and `ERPM/` directories remain the ROS implementation and analysis library; operators run this suite.

The design follows the supplied Gordiienko (2024) methodology where it is identifiable indoors: separate manoeuvres, windowed measurements, training-only fits, and independent validation on new conditions. The room profile uses the measured 14 × 14 m walls, keeps a 1 m exclusion band, and places long straight manoeuvres on the 45° diagonal of the clear central 12 × 12 m area. It extends the paper with direct metrology, steering calibration, ACCEL_TO_CURRENT ground-acceleration validation, nonlinear-model stop gates, and a final parameter inventory.

## A/B/C contract

Every parameter-producing component uses this sequence:

1. **A — capture:** record only the scoped training manoeuvre.
2. **B — analyse/update:** export data, make plots, fit only A data, gate the result, apply an accepted temporary candidate where a runtime parameter exists, and rebuild.
3. **C — validate:** collect new conditions with the frozen candidate active and evaluate them without refitting.

The source `vesc.yaml` bytes are restored after every invocation. The candidate is re-applied only inside the next reversible transaction, so C tests the candidate without permanently altering the source tree. Direct metrology updates frozen session analysis inputs and builds a reversible dynamic-model geometry patch; it never permanently changes the source tree. The static steering B patch explicitly uses an identity correction polynomial, so C cannot accidentally validate a stale nonlinear correction from an older campaign.

Audits and sensor preflights are not fitted models, but each has its own capture, analysis, report page, and downstream gate.

## Start and run one stage at a time

### Guided GUI (recommended)

From the main PC, use the one-command SSH launcher (the repository must also be present on the Jetson):

    python3 run_test 10.126.128.198

It logs in as `f1tenth`, starts the GUI on the Jetson, creates a private localhost SSH tunnel, and opens the page on the main PC. Keep that terminal open while testing and press Ctrl-C when finished. The supplied car password is used without placing it in the process arguments or terminal output; set `F1TENTH_SSH_PASSWORD` to override it. The launcher auto-detects `/home/f1tenth/BachelorProject` and `/home/f1tenth/Documents/GitHub/BachelorProject`; use `--remote-workspace` if the clone is elsewhere. It never exposes the GUI on the network.

For a GUI running directly on the current machine:

From a terminal with the workspace sourced:

    source install/setup.bash
    python3 f1tenth_parameters/vehicle_calibration/run_suite.py gui

This opens the localhost-only **F1TENTH Calibration Studio**. It does not replace or weaken the runner: every button launches the same single-stage CLI through a pseudo-terminal, so dependency gates, ROS recording, analysis, reversible builds and retry/REDO rules remain authoritative. The GUI adds:

- a preparation dashboard for ROS/build/disk/recovery readiness;
- an editable metrology form with measured SI values, kg-to-newton axle-load conversion and live closure errors; no standard-deviation input is requested;
- a complete 28-stage journey with setup, safety, expected behaviour, collected channels and expected outputs for each stage;
- explicit safety confirmations before every stage;
- live interactive `READY`, `ACCEPT`, `REDO`, `SKIP` and steering-end-stop single-key controls;
- a controlled interrupt button that sends `SIGINT`, allowing the runner's normal neutral/restore cleanup to execute;
- stage plots, headline fitted values, failure reasons, artifact links and the cumulative PDF report.

A session whose frozen stage order predates the current 28-stage contract is clearly marked **inspect only**. Its files and reports remain available, but it cannot be mixed with the current runner; create a new session to run the complete campaign.

The command-capable web server binds only to `127.0.0.1`; remote binding is intentionally rejected. Keep the launch terminal open. Closing it or pressing Ctrl+C while a stage is active requests a controlled stop and configuration restoration.

To resume a particular session or choose another local port:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py gui \
      --session f1tenth_parameters/vehicle_calibration/runs/<session> --port 8765

The CLI workflow remains available and is useful for automation or recovery:

From the repository root:

    source install/setup.bash
    python3 f1tenth_parameters/vehicle_calibration/run_suite.py new

`new` creates a session and copies the editable form:

    f1tenth_parameters/vehicle_calibration/runs/<session>/physical_measurements.yaml

Fill only the campaign-specific physical values before the second stage: race-ready mass, front/rear axle loads, and wheelbase. Do **not** re-enter the VESC IMU, LiDAR or rear-axle position. A new session imports those already-measured values from `f1tenth_system/f1tenth_stack/config/vesc.yaml` → `vehicle_geometry`, freezes them into both calibration snapshots, and records the source path in `physical_measurements.yaml`. Existing unfilled session sheets are populated the same way when opened by the canonical runner.

The imported values currently resolve to LiDAR `(x,y,z,yaw)=(0.265,0,0.05,0)`, VESC IMU/CoG `(0.160,0,0.0703,0)`, and rear axle `(0,0)`. Coordinates use the rear-axle `base_link`: IMU x/y are assumed coincident with the calculated CoG at `(0.160,0)`. The `z=0.0703 m` entry is the calculated CoG height used only to complete the static TF; it is not treated as a measured IMU lever arm and is unused by every planar calibration fit. Both standard launch files use this one source as their default static LiDAR/IMU transforms, and `vesc_to_odom` receives the IMU yaw. If the hardware is moved, update `vehicle_geometry` once; the next calibration session will inherit it automatically.

Run exactly one stage, inspect its plot/page, then continue:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py run \
      --session f1tenth_parameters/vehicle_calibration/runs/<session> --next

You may repeat a named, spoiled C capture:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py run \
      --session f1tenth_parameters/vehicle_calibration/runs/<session> \
      --stage steering_centre_validation

`--next` always returns after one complete stage. Review that stage's B result and C validation before spending more vehicle time; the canonical runner intentionally has no `--all` shortcut.

If power loss interrupts a temporary configuration transaction, recover before driving:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py recover \
      --workspace /home/akselmo/Documents/GitHub/BachelorProject

## Retry versus fresh recalibration

If C was spoiled by an obstacle, person, poor fixed features, or sensor dropout, repeat that C stage: it deliberately retains the same fitted candidate.

If C shows the fitted value is wrong, archive the branch and restart fresh A/B data:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py redo \
      --session f1tenth_parameters/vehicle_calibration/runs/<session> \
      --from steering_centre_validation

`redo` clears that candidate and every downstream stage, archives the old branch under `recalibration_history/`, and makes the next `--next` collect new A data. It cannot silently reuse a failed offset. The same contract applies to steering-map, steering-response, ERPM-map, ERPM-response, coast-down, current, acceleration-interface, and tyre-stiffness validation.

## LiDAR policy

The isolated steering and ERPM calibration launches explicitly request the complete 1080-sample, 270-degree scan at 40 Hz, and the ICP setting is `downsample: 1`. After each export, the unified runner hard-fails before fitting unless every scan has at least 1080 ranges and the median rate is at least 35 Hz. This does **not** alter the normal racing path: AMCL/planning continue to consume the validated 270-beam `/scan`. If full-resolution racing data must be recorded, launch `System_launch.py full_scan:=true`; it publishes 1080 beams on `/scan_full` while keeping the reduced `/scan` for AMCL. At 0.6 m/s, consecutive scans are only about 1.5 cm apart, so scan-to-scan ICP is never treated as an independent velocity or curvature observation.

Each raw registration instead uses a displacement-targeted baseline of about 12 cm. It uses IMU yaw only as a rotational seed; odometry position, velocity, and pose are never fed into ICP. The measured LiDAR-to-`base_link` transform converts registrations to base-frame motion. Multiple accepted registrations are then robustly aggregated in a 0.5 s window with 0.1 s stride. The robust window is the fitting product; raw registrations remain diagnostic evidence only.

The LiDAR quality check is therefore after direct metrology and before the steering-centre fit. It gates moving-window validity, speed, geometry/RMSE, and fixed environmental features. Stationary scan matching remains a noise diagnostic, not a false moving-sensor gate. The static raw-servo map translates LiDAR motion from `base_link` to the measured rear axle before deriving its equivalent steering angle from yaw and speed; IMU yaw is retained as a cross-check rather than becoming the map reference.

## Stage order

| # | Stage | Purpose |
| --- | --- | --- |
| 1 | `steering_command_audit` | Stationary steering command-chain safety audit. |
| 2 | `physical_metrology` | Mass/geometry/axle loads/LiDAR transform; updates frozen session geometry before motion. |
| 3 | `steering_observability` | Moving LiDAR-window quality preflight. |
| 4–5 | `steering_centre` → `steering_centre_validation` | A/B offset fit, then fresh straight C. |
| 6 | `steering_endstops` | Human mechanical limits and wheel-angle survey. |
| 7–8 | `steering_static_training` → `steering_static_holdout` | Servo-to-effective-angle A/B and shuffled C; complete circles where they fit. |
| 9–10 | `steering_response` → `steering_response_validation` | Combined steering/vehicle response A/B and distinct step C; complete step circles. |
| 11 | `motor_command_audit` | Stationary motor command-path audit. |
| 12 | `longitudinal_observability` | Session-local stationary diagnostic plus moving LiDAR/IMU check. |
| 13 | `low_speed_launch` | Low-speed observability and deadband data. |
| 14–15 | `erpm_map_training` → `erpm_map_holdout` | ERPM/odometry A/B and separate speed-cell C. |
| 16 | `vel_to_erpm_audit` | Full velocity-to-ERPM pipeline audit after C. |
| 17–18 | `erpm_response` → `erpm_response_validation` | Timing A/B and distinct response C. |
| 19–20 | `coastdown` → `coastdown_validation` | Integrated drag trajectory A/B and distinct-initial-speed C. |
| 21–22 | `current_training` → `current_holdout` | Current-to-ground-acceleration A/B and distinct speed/current C. |
| 23–24 | `accel_interface` → `accel_interface_validation` | ACCEL_TO_CURRENT routing and realised-ground-acceleration A, then C. |
| 25 | `odometry_candidate_velocity_validation` | Fresh steady-speed C for the selected wheel/IMU odometry model. |
| 26 | `odometry_candidate_accel_validation` | Fresh transient C for the selected wheel/IMU odometry model. |
| 27–28 | `lateral_stiffness_training` → `lateral_stiffness_validation` | Effective front/rear tyre stiffness, dynamic steering scale and cornering wheel-slip A/B; independent full-circle/runtime C. |

The preflight rotates each complete straight/finite-arc envelope—including startup, command-path settling, active stopping, conservative emergency braking and the vehicle body—onto the 45° lane and checks both projected axes against the clear central 12 × 12 m square. Its clear-room diagonal is 16.97 m and its footprint-aware straight-motion capacity is 16.10 m. Long steady captures deliberately target only 82% of that clearance-reduced room; the remaining 18% absorbs positioning and startup variation, while the separate 1 m wall exclusion remains untouched. Duration is calculated per speed: the slowest conditions can record for 30 s (about 1,200 native scans), while fast conditions are shortened to preserve the same envelope. Current/acceleration pulses are not lengthened because pulse duration is part of the excitation being identified.

Static steering, steering-response and tyre-stiffness conditions use the human-measured or commanded wheel angle to calculate their footprint-aware radius. Every fitting steady turn records one complete revolution. The shallow steering-map point that cannot fit a full circle instead records the longest bounded arc inside the same 82% target. The lateral tests establish speed and brake while retaining the commanded steering, so they remain on the marked circle. The room model uses a reviewed 0.50 rad road-wheel-angle bound and a conservative 1.5 m/s² braking floor; this is a planning assumption, not fabricated measured performance, and must be met during the low-speed brake check. The profile is limited to 3.0 m/s and rejects any manoeuvre that does not fit the 14 × 14 m room.

For ROS 2 smoke testing, `f1tenth_sim/config/sim.yaml` defaults to `calibration_room_map`, a 14 × 14 m enclosed map with a clear central 12 × 12 m area, a 45° initial heading, and asymmetric fixed wall features for LiDAR observability. It deliberately does not use an arbitrary racetrack. Track/benchmark launches may still override `map_path` and their initial pose explicitly.

The reproducible smoke test launches both isolated calibration ROS graphs with simulated VESC sensors and the full 1080-beam scan, exercises their exclusive selectors, and rejects a collision or hardware-driver start:

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 f1tenth_parameters/vehicle_calibration/open_room_smoke.py

## Steering centre and straight-driving policy

The centre stage starts from the deployed `vesc.yaml` offset as a useful prior, never as an answer. It uses three short raw-servo probes around that seed. IMU and odometry yaw slopes calculate the correction direction and a bounded provisional centre; a repeated fine LiDAR grid then supplies the actual fit. The near-centre increment is `0.005` servo, about 0.3° road-wheel angle with the archived gain.

This is directed use of the onboard IMU/odom, not random addition/subtraction. IMU and odometry cannot certify the centre: IMU bias can change with bringup and odometry contains the steering map being calibrated. The LiDAR fit must have expected sign, observed span, sufficient R², low residual, and a zero inside the measured bracket.

The C stage uses six new passes: three outbound at 0.5 m/s and three inbound at 0.8 m/s. It gates LiDAR yaw, lateral velocity, heading change, LiDAR quality, and the operator's observation. A visibly drifting car is a REDO even if odometry claims stability. This prevents the old approximately-0.53/0.55 false acceptance.

No heading PID is used for centre or static steering-map data. A bounded heading hold is allowed only for later longitudinal runs after centre C passes. Its trim is recorded and gated, so it cannot hide steering bias in longitudinal data.

There is no persistent ground-IMU-bias calibration stage. Bringup establishes its own epoch on each stack launch; stationary IMU samples are session-local diagnostics only, are never subtracted from a later stack's steering/lateral data, and are never written into the steering offset.

## Longitudinal control and nonlinear gates

The static ERPM map is retained for wheel odometry, calibration speed holds, and the legacy VEL_TO_ERPM path. It is **not** the racing throttle model. Racing remains `ACCEL_TO_CURRENT`; current, coast-down, and acceleration-interface stages test realised ground acceleration directly with LiDAR windows.

If C materially prefers a model the production stack cannot represent, the suite stops rather than silently flattening it:

- `nonlinear_steering_map_request.json`: the frozen piecewise steering map beats the deployed linear patch.
- ERPM speed validation flag `requires_full_stack_upgrade_for_selected_static_map`: a quadratic static map wins.
- `lateral_nonlinear_model_request.yaml`: nonlinear tyre force wins on independent arcs.
- `nonlinear_longitudinal_actuation_model_request.yaml`: a speed/current surface beats scalar ACCEL_TO_CURRENT on hold-out data.
- `dynamic_longitudinal_slip_model_request.yaml`: traction transients reject a memoryless scalar actuation model.
- `odometry_runtime_model_request.yaml`: fresh shadow C data select a command/wheel/IMU model that the production C++ path cannot reproduce exactly.

The failed stage still receives a Markdown page, plot when its analysed data is available, and a failed page in the cumulative LaTeX document. Implement the required bounded runtime model, then `redo` from that parameter's A stage.

## Tyre stiffness and other vehicle parameters

Tyre stiffness is estimated as an **effective low-slip cornering stiffness** for the measured tyre state, load, battery, and indoor surface. It is not claimed as a universal tyre constant. Every balanced speed/angle/sign condition settles and then records one complete circle, while still contributing one independent robust summary per manoeuvre:

    F_yf = l_r m a_y / (L cos(delta))
    F_yr = l_f m a_y / L
    alpha_f = delta - beta - l_f r / v_x
    alpha_r = -beta + l_r r / v_x

LiDAR motion is transformed first to `base_link` and then by the measured rigid-body offset to the CG before calculating `beta = atan2(v_y, v_x)`. IMU horizontal acceleration is rotated from its measured sensor yaw into `base_link` and shifted from the IMU origin to the CG before force balance. The calibration launch and `vesc_to_odom` use that same planar IMU transform, so a non-aligned IMU cannot silently look like lateral tyre force. The paper's `v_y ≈ -a_x/r` is a cross-check, not a replacement. The front fit jointly identifies the dynamic-bicycle `steering_model_scale` with `C_f`; this removes the previous hand-tuned scale from the model path. Validation evaluates the exact bounded runtime equation `F_y=C_alpha sin(shape atan(alpha/shape))`, not merely its linear tangent; the fixed shape is not presented as identified peak-tyre data. The same independent arc trials fit the causal cornering wheel-over-read coefficient from ERPM, IMU yaw and LiDAR ground speed. Trial bootstraps quantify all three fitted quantities, and distinct arc cells validate the frozen tyre, steering-scale and cornering-speed models plus the rebuilt runtime odometry.

`physical_measurements.yaml` also records vehicle dimensions, tracks, loaded wheel radius, CG height, and yaw inertia. It can derive yaw inertia from an optional bifilar pendulum measurement. The final `vehicle_parameter_inventory.yaml` explicitly separates direct measurements, validated effective parameters, model-selection flags, and values that cannot honestly be identified from this safe campaign.

Peak friction/full Pacejka shape, separated motor electrical R/L/back-EMF/gear efficiency, and separated servo/linkage/tyre-relaxation dynamics are deliberately reported as unidentifiable unless the necessary instrumentation or a separately approved limit test is available. The reported `C_f/C_r` are therefore effective values conditional on the LiDAR-derived steering map; a multi-point physical road-wheel-angle instrument is needed to turn them into component-level tyre constants. They are not guessed from unrelated data.

## Reports and promotion

Every completed, rejected, or redone stage updates its Markdown page, plot when data are available, cumulative LaTeX source, and PDF when `pdflatex` is installed. Each LaTeX page contains A/B/C role, status, fitted numbers, plot, statistical support, and raw-data link; the final document adds the parameter inventory. Every stage also writes `analysis/statistics/<stage>.yaml`: distributions use one aggregate per independent trial, never overlapping LiDAR windows as fake sample count. It records accepted/rejected rows, condition and repetition coverage, mean/sample spread/quantiles, a deterministic bootstrap median interval, and all fit/validation accuracy metrics emitted by that stage. Direct ruler/scale/metrology entries intentionally do not receive invented standard deviations or confidence intervals. Parameter-specific bootstrap intervals are additionally reported for steering centre, ERPM gain, current gains, coast-down, front/rear stiffness, dynamic steering scale, and the cornering wheel-slip coefficient.

To render the same plot code against a historical standalone steering session (without modifying its MCAP or analysis), run:

    python3 f1tenth_parameters/vehicle_calibration/render_historical_preview.py \
      /path/to/steering_session --output f1tenth_parameters/vehicle_calibration/preview/steering

All 28 stages have a dedicated plot rather than relying on a generic event-count chart. Audit pages compare requested commands with selector/bus/driver delivery; observability pages distinguish accepted and rejected robust LiDAR windows; fit pages show measured data and the candidate model; response pages show delay, rise time, time constant and fit error; and every C stage has its own hold-out figure. Multi-panel figures keep related diagnostics on the same one-page stage report. The GUI shows each figure both on its stage card and in the results gallery, while the LaTeX document embeds the same immutable PNG on that stage's page. A plot can only be absent when capture/analysis stopped before producing the data needed to calculate it; the failure page and terminal log remain available in that case.

Create the current or final document:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py report \
      --session f1tenth_parameters/vehicle_calibration/runs/<session>

This writes `analysis/vehicle_calibration_report.tex` and, when available, PDF; `vehicle_calibration_report.md`; `vehicle_parameters_candidate.yaml`; `vehicle_parameter_inventory.yaml`; and stage plots under `plots/stages/`.

Promotion is separate and blocked unless every required A/B/C gate passes:

    python3 f1tenth_parameters/vehicle_calibration/run_suite.py report \
      --session f1tenth_parameters/vehicle_calibration/runs/<session> --promote

Promotion carries both the accepted dynamic model values and the verified planar static geometry into the production VESC configuration. Restart the normal stack after promotion so its static transforms and odometry use the same reference geometry as the validation run.

## Campaign size and time

The session manifest calculates the selected grid. The default profile has 28 stages, two stationary capture blocks, two manual stages, and **504–507 nominal accepted driving passes**. Every fitted or validated condition has at least three independent repetitions; the small range is only whether the retained deployed steering seed falls outside the guided fine grid.

Plan **10–14 hours** of operator/vehicle time without REDO, normally split over two days. This assumes 45–60 seconds average setup/repositioning per accepted pass, 2.5–4 hours for direct metrology/builds/plot review/safety checks, and about one additional hour for the room-optimised steady captures and full circles. Spoiled captures, nonlinear implementation work, and explicit REDO are correctly extra time rather than hidden behind a false “full test complete” result.
