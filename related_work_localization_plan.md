# Related Work Plan: LiDAR and Odometry Localization

This file is a planning document for the thesis related-work section. It is not final thesis prose yet. It lists the localization methods to compare, the main claims to make, the likely advantages and disadvantages for an F1TENTH car on a known indoor racetrack, and the papers that can support each point.

Project context assumed here:

- Vehicle: F1TENTH-scale Ackermann car.
- Track: approximately 10 m by 30 m.
- Speed range: approximately 2 m/s to 10 m/s.
- Sensors for localization: 2D LiDAR and VESC-based odometry.
- Chosen localization approach: Adaptive Monte Carlo Localization (AMCL), using odometry as the motion proposal and LiDAR against a known occupancy map.

## 1. Section Goal

The related-work section should not read like a history of every localization paper. It should explain the main algorithm families that were realistic choices for this project, compare their assumptions, and lead naturally to AMCL.

Suggested final narrative:

1. Localization must estimate planar pose `(x, y, yaw)` fast enough for closed-loop racing.
2. VESC ERPM/steering odometry gives high-rate short-term motion but drifts because small systematic and non-systematic errors accumulate over distance [Borenstein1996OdometryErrors, Borenstein1997Positioning].
3. LiDAR can correct this drift by comparing range measurements with either previous scans, extracted map features, an NDT representation, or an occupancy grid likelihood model [Lu1997ScanMatching, Zhang2000LineSegments, Biber2003NDT, Dellaert1999MCL].
4. For this project, a known map and repeated laps make a map-based probabilistic method attractive.
5. Odometry and LiDAR alone do not uniquely imply AMCL: scan matching and NDT can also use odometry as the initial guess and LiDAR as the correction measurement. The reason to choose AMCL is instead its occupancy-map likelihood model, particle-based uncertainty representation, and tunable computation through particle count [Dellaert1999MCL, Thrun2001RobustMCL, Fox2003KLD].
6. The final pose estimate in this project is produced by an Extended Kalman Filter (EKF): odometry is used for high-rate prediction and AMCL pose is used as the map-based correction measurement [Kalman1960Filter, Ahmad2013EKFLocalization].

## 2. LiDAR Localization Algorithms

This LiDAR section should follow the F1TENTH Module C localization sequence:

1. Lecture 9 introduces LiDAR localization through scan matching, ICP, and fast correspondence search [F1TENTHLecture09].
2. Lecture 10 focuses further on fast correspondence search, which is the runtime-critical part of scan matching [F1TENTHLecture10].
3. Lecture 11 moves from single scan alignment to particle-filter localization using laser scan data, a control input, and a map to estimate pose and particle belief [F1TENTHLecture11].
4. NDT is added as an extra method from our own research because it is a relevant scan-registration alternative, even though it is not part of the three F1TENTH lectures [Biber2003NDT].

Use the lectures as the structure of the explanation. Use the papers below as the academic references in the final report.

### 2.1 Scan Matching and ICP

Core idea:

- Estimate the vehicle pose by aligning a 2D LiDAR scan with a previous scan or a map.
- Iterative Closest Point (ICP) alternates between finding point correspondences and solving for the rigid transform that minimizes alignment error [Besl1992ICP].
- In 2D LiDAR localization, scan matching is often used as a local pose update, usually initialized by odometry [Lu1997ScanMatching].
- The F1TENTH lectures introduce scan matching first because it is the direct geometric way to use LiDAR range measurements for localization [F1TENTHLecture09].

Points to explain:

- Scan matching is a local optimization problem.
- It normally needs a good initial pose estimate from odometry.
- Correspondence search is central: wrong correspondences cause bad pose updates.
- Point-to-line ICP improves the error metric for 2D laser scan matching by comparing scan points to local line structure rather than only point-to-point distances [Censi2008PLICP].

Pros for this project:

- Uses raw LiDAR geometry directly.
- High update rate is possible in 2D.
- Good for short-term local correction when odometry already gives a close initial pose.
- Easy to explain from the F1TENTH scan-matching lectures.

Cons for this project:

- Pure scan-to-scan matching can still drift because it estimates relative motion.
- Scan-to-map matching can fail if initialized too far from the true pose.
- Repeated straights and similar wall shapes on a small racetrack can create local minima.
- Dynamic obstacles and partial occlusions can corrupt correspondences.

Likely conclusion:

- Scan matching is a strong local LiDAR localization baseline, but it produces a single registration result and does not naturally represent uncertainty over multiple possible poses.

### 2.2 Fast Correspondence Search and Runtime

Core idea:

- The expensive part of ICP-style scan matching is often finding which points or map elements correspond to each other.
- Fast correspondence search is therefore a practical requirement, not just an implementation detail [F1TENTHLecture10].
- Efficient nearest-neighbor or spatial search methods reduce latency, which matters when the car moves 2-10 m/s.
- This project does not implement the Compressed Directional Distance Transform (CDDT). CDDT is a related ray-casting acceleration method, not the method used in the current AMCL sensor model [Walsh2017CDDT].

Points to explain:

- At 10 m/s, a localization delay of 50 ms corresponds to about 0.5 m of travel.
- Even a good scan-matching objective is not useful for racing if the correspondence search makes it too slow.
- The same runtime issue appears in particle-filter LiDAR sensor models, where many particles require many ray casts or likelihood evaluations.
- Course-referenced CDDT work shows why accelerating occupancy-grid ray casting is important for particle-filter localization on embedded hardware [Walsh2017CDDT].
- In this project, runtime is improved differently: the implemented sensor model uses a precomputed Euclidean distance transform (EDT), subsampled beams, and GPU-parallel likelihood evaluation.

Pros for this project:

- Makes local scan matching and particle-filter sensor updates feasible at higher rates.
- Reduces CPU/GPU budget pressure.
- Helps keep localization latency bounded.

Cons for this project:

- Approximate or discretized correspondence/ray-casting methods can introduce small measurement errors.
- More complex acceleration structures increase implementation complexity.
- This is usually an enabling technique, not a complete localization method by itself.

Likely conclusion:

- Fast correspondence and fast sensor-model evaluation should be discussed as practical LiDAR handling requirements. They support both scan matching and AMCL.
- CDDT should be described only as related work or a possible future ray-casting alternative. It should not be described as part of the implemented AMCL.

### 2.3 Particle Filters and AMCL

Core idea:

- Particle filters maintain a set of weighted pose hypotheses.
- The F1TENTH particle-filter lecture frames the localization problem as using laser scan data, a control input, and a map to output a pose and particle set [F1TENTHLecture11].
- In AMCL, odometry predicts particle motion, LiDAR/map likelihoods weight particles, and resampling keeps likely poses [Dellaert1999MCL, Fox1999EfficientMCL, Thrun2001RobustMCL].
- Adaptive sampling changes particle count based on uncertainty, improving the computation/robustness tradeoff [Fox2001KLDNIPS, Fox2003KLD].
- The implemented AMCL sensor model is a likelihood-field model: the occupancy map is preprocessed into an EDT, each measured beam endpoint is transformed into the map frame, and particle weight is computed from the distance between that endpoint and the nearest occupied cell.

Points to explain:

- MCL represents arbitrary/non-Gaussian belief distributions better than a single Gaussian estimator [Dellaert1999MCL].
- Markov localization and MCL are designed to maintain a probability density over robot pose [Fox1999MarkovLocalization, Dellaert1999MCL].
- In its full form, MCL can perform global localization because particles can cover multiple possible poses [Dellaert1999MCL, Fox1999EfficientMCL].
- In this project, AMCL is configured as a local localization method initialized near the expected pose. Therefore the report should not claim demonstrated global localization or global recovery unless that mode is actually tested.
- Robust variants improve behavior under localization failure and dynamic environments by modifying how samples are generated [Thrun2001RobustMCL].

Pros for this project:

- Internally combines VESC odometry prediction with 2D LiDAR map weighting.
- Works naturally with a known 2D occupancy map.
- Can correct odometry drift every LiDAR update.
- Can represent local pose uncertainty after ambiguous sections better than a single-pose scan matcher.
- Particle count gives a clear computation/accuracy tradeoff for the Jetson/CPU/GPU budget.
- Likelihood-field weighting avoids ray casting expected ranges for every particle and beam.
- GPU implementation parallelizes particle/beam likelihood evaluation.

Cons for this project:

- Requires a good map and reasonable LiDAR-map calibration.
- Bad motion-model covariance can cause lag, particle depletion, or poor recovery.
- More particles increase compute cost and latency.
- Symmetric tracks can still produce ambiguous hypotheses until distinctive geometry is seen.
- In the tested local configuration, AMCL does not provide a separate global relocalization mechanism.
- Likelihood-field models can be less physically exact than ray-casting beam models because they score measured endpoints by nearest-obstacle distance rather than comparing measured and expected range along each beam.

Why this leads to our choice:

- The sensor pair is not unique to AMCL. Scan matching and NDT can also use odometry plus LiDAR.
- AMCL was selected because it uses the existing 2D occupancy map directly and gives a particle-based probabilistic correction instead of only a single local scan-registration optimum.
- The environment is bounded and known, so a map-based method is reasonable.
- The car speed makes pure odometry too drift-prone, while pure local scan matching is fragile under ambiguity.
- AMCL gives the desired compromise: practical implementation, probabilistic local uncertainty, map-based drift correction, and tunable runtime.

Implementation note for report:

- Say: "The implemented AMCL uses a likelihood-field LiDAR model with a precomputed Euclidean distance transform and GPU-parallel particle weighting."
- Do not say: "The implemented AMCL uses CDDT."
- CDDT can remain in related work as a ray-casting acceleration method from the F1TENTH lecture references, but not as implemented system design.

### 2.4 Normal Distributions Transform (NDT)

Core idea:

- NDT is included as an extra research method outside the F1TENTH lecture sequence.
- Divide space into cells and represent points in each cell with a normal distribution.
- Register a scan by optimizing the likelihood of scan points under this continuous piecewise distribution.
- NDT avoids explicit point-to-point correspondences [Biber2003NDT].

Points to explain:

- NDT is a scan-matching method, but with a smoother map representation than raw point sets.
- It can be efficient and memory-conscious because each cell stores distribution parameters instead of all raw points [Biber2003NDT, Magnusson2007NDT].
- Like ICP, it is usually local optimization and still needs a reasonable initial pose estimate.
- Cell size affects performance: too coarse loses detail, too fine becomes sparse/noisy [Biber2003NDT].

Pros for this project:

- No explicit point-to-point correspondence search.
- Can be fast and stable when the map has enough geometric structure.
- Good candidate for scan-to-map localization if an NDT map is available.

Cons for this project:

- Needs tuning of grid/cell resolution.
- Still has local-minimum risk.
- Does not by itself maintain multiple pose hypotheses unless wrapped in a probabilistic filter.
- Less directly aligned with standard 2D occupancy-grid AMCL tooling.

Likely conclusion:

- NDT is a strong scan-registration alternative to ICP-style matching. It is useful to mention as extra research, but AMCL better matches the implemented occupancy-map particle-filter pipeline.

### 2.5 Feature-Based LiDAR Localization

Status in this plan:

- Feature-based localization is not part of the main F1TENTH lecture flow used here.
- It can be kept as a short optional paragraph if the thesis needs a broader taxonomy of LiDAR localization.

Core idea:

- Extract geometric features such as line segments, corners, or breakpoints from 2D LiDAR scans.
- Match extracted features to a feature map or to features from earlier scans.
- Estimate pose from the matched feature correspondences [Gonzalez1992Iconic, Zhang2000LineSegments, Borges2004LineExtraction, Nguyen2005LineExtraction].

Likely conclusion:

- Feature methods are compact and interpretable, but feature extraction and association add tuning risk on a small racetrack with repeated wall-like geometry. Keep this shorter than scan matching, AMCL, and NDT unless the examiner expects a full taxonomy.

## 3. Odometry / Motion Models

### 3.1 Simple Analytical Dead-Reckoning

Core idea:

- Integrate speed inferred from VESC electrical RPM (ERPM) together with steering/yaw information forward in time.
- This gives a high-rate pose estimate relative to the start pose.
- In practice, this is often the baseline odometry used before LiDAR correction.

Points to explain:

- Odometry is relative positioning, so error grows with travelled distance [Borenstein1997Positioning].
- Systematic errors include wheel radius, wheelbase, steering calibration, and ERPM-to-vehicle-speed calibration errors [Borenstein1996OdometryErrors].
- Non-systematic errors include slip, uneven ground, impacts, and fast maneuvers [Borenstein1996OdometryErrors, Borenstein1997Positioning].

Pros for this project:

- Very fast and simple.
- High update rate.
- Good short-term prediction between LiDAR scans.
- Does not require a map.

Cons for this project:

- Drift is unavoidable without absolute/map correction.
- At 10 m/s, small steering or speed errors quickly become large lateral/yaw errors.
- Wheel-speed-derived velocity can be biased during acceleration, braking, or tire slip.

Likely conclusion:

- Pure odometry is not enough for final localization, but it is useful as a high-rate EKF prediction source between LiDAR/map corrections.

### 3.2 Kinematic Bicycle Model

Core idea:

- Approximate the car as a single front wheel and a single rear wheel.
- Assume rolling without lateral slip.
- Use speed, steering angle, and wheelbase to predict planar motion.

Typical continuous-time form:

```text
xdot     = v cos(theta)
ydot     = v sin(theta)
thetaDot = v / L * tan(delta)
```

More detailed variants model the center-of-mass slip angle from front/rear wheelbase distances [Kong2015VehicleModels, Polack2017KinematicBicycle].

Points to explain:

- The kinematic bicycle model is simple and common in autonomous driving.
- It avoids tire-force parameters and is computationally cheap [Kong2015VehicleModels].
- Its assumptions become weaker as lateral acceleration and tire slip increase [Kong2015VehicleModels, Polack2017KinematicBicycle].

Pros for this project:

- Matches Ackermann steering geometry better than a differential-drive model.
- Low computational cost.
- Good default for odometry prediction at moderate speed and low slip.
- Few parameters: mainly wheelbase and steering calibration.

Cons for this project:

- Does not model lateral tire slip.
- Can understate uncertainty during fast cornering.
- Accuracy depends strongly on steering-angle calibration and delay.

Likely conclusion:

- Best practical motion model for AMCL unless the vehicle often operates in high-slip conditions. Use tuned covariance to admit uncertainty during aggressive turns.

### 3.3 Dynamic Bicycle Model

Core idea:

- Extend the bicycle model with lateral velocity, yaw rate, tire slip angles, and lateral tire forces.
- Often uses mass, yaw inertia, front/rear axle distances, and cornering stiffness parameters [Kong2015VehicleModels, Altche2017DynamicPlanning].

Points to explain:

- Dynamic models can represent high-speed lateral behavior better than pure kinematics.
- They need more parameters and measurements.
- Tire-force models can become difficult to identify accurately on a small vehicle and can vary with tire, floor, temperature, and speed.

Pros for this project:

- Better physical model when the car has measurable slip.
- Useful for high-speed prediction and controller design.
- Can explain why odometry errors grow during aggressive cornering.

Cons for this project:

- Requires parameters that may be hard to measure robustly on F1TENTH.
- More sensitive to calibration and model mismatch.
- May need IMU yaw-rate/lateral-acceleration data to be useful for localization.
- Higher complexity than needed if AMCL corrects pose frequently with LiDAR.

Likely conclusion:

- Dynamic bicycle odometry is theoretically attractive at 10 m/s, but for the localization related-work section it should be presented as a higher-complexity alternative. The implemented AMCL can instead use simpler odometry with realistic motion noise.

## 4. EKF Fusion Layer

Core idea:

- The Extended Kalman Filter estimates the state of a nonlinear system by alternating prediction and correction.
- Prediction propagates the current state and covariance through a local linearization of the motion model.
- Correction updates the state with a measurement and weights prediction vs measurement according to their covariances [Kalman1960Filter, Ahmad2013EKFLocalization].

How this applies in this project:

- AMCL already uses odometry internally for particle prediction and LiDAR for map-based particle weighting.
- The project then uses an EKF as a second-stage fusion/output layer.
- In the current implementation, `/odom_pose` is used for EKF prediction, `/amcl_pose` is used for EKF correction, and `/ekf_pose` is published as the fused output.
- Therefore, the report should describe the architecture as: odometry gives high-rate relative motion, AMCL gives lower-rate map-corrected pose from LiDAR, and EKF produces a temporally smooth fused pose estimate.

Pros for this project:

- Provides high-rate pose propagation between AMCL corrections.
- Uses covariance to balance odometry prediction against AMCL correction.
- Can reduce discontinuities if AMCL updates are noisier or lower-rate than odometry.
- Gives one pose topic for downstream control.

Cons for this project:

- EKF state is unimodal/Gaussian, so it cannot represent multiple possible poses like a particle filter.
- Poor covariance tuning can make the estimate either lag AMCL corrections or trust noisy jumps too much.
- If AMCL output is already highly filtered, EKF may add delay without much accuracy benefit.
- Since AMCL itself already uses odometry, the report should avoid implying that EKF is the only place where odometry and LiDAR are fused.

Likely conclusion:

- EKF is not an alternative to AMCL here. It is the final state-estimation layer that combines high-rate odometry propagation with AMCL pose corrections.

## 5. Comparison Table Draft

| Method | Main input | Output | Strength | Weakness | Fit for project |
|---|---|---|---|---|---|
| Simple odometry | VESC ERPM-derived speed, steering/yaw | Relative pose | Fast, high-rate | Unbounded drift | Good as EKF prediction only |
| Kinematic bicycle | Speed, steering, wheelbase | Relative pose | Simple Ackermann model | No lateral slip | Good EKF prediction model |
| Dynamic bicycle | Speed, steering, tire/vehicle params, maybe IMU | Relative pose/state | Better high-speed physics | Hard calibration | Possible future upgrade |
| Scan matching | Consecutive scans or scan + map | Relative/local pose | Uses raw geometry | Local minima, drift | Good baseline/local tracker |
| Fast correspondence / ray-casting acceleration | Scan/map queries | Faster LiDAR matching or weighting | Lower latency | Not localization by itself | Related support method; CDDT not implemented |
| NDT | Scan + NDT grid/map | Local pose | Smooth, no explicit correspondences | Needs cell tuning, local optimum | Strong alternative |
| AMCL | Odometry + LiDAR + occupancy map | Probabilistic local/map pose in this project | Likelihood-field EDT, particle uncertainty, map correction | Particle compute, map needed | Chosen LiDAR-map method |
| Feature-based | Extracted lines/corners | Pose from matched features | Compact/interpretable | Data association fragile | Optional short background |
| EKF | Odom pose + AMCL pose | Fused pose | High-rate prediction plus map correction | Gaussian/unimodal, covariance tuning | Final output layer |

## 6. Proposed Thesis Subsection Structure

### 6.1 Localization Problem

- Define localization as estimating planar pose on a known track.
- State why odometry alone is insufficient.
- State why LiDAR is the main absolute correction sensor.

Sources:

- F1TENTH platform context: [OKelly2020F1TENTH].
- Odometry drift and positioning categories: [Borenstein1997Positioning].

### 6.2 Odometry Models

- Start with VESC ERPM-based dead-reckoning odometry.
- Explain accumulated error.
- Compare simple analytical, kinematic bicycle, and dynamic bicycle.
- End by saying odometry is used as a short-term EKF prediction source, while LiDAR/map corrections provide the bounded map-relative pose.

Sources:

- Odometry error: [Borenstein1996OdometryErrors, Borenstein1997Positioning].
- Kinematic/dynamic vehicle models: [Kong2015VehicleModels, Polack2017KinematicBicycle, Altche2017DynamicPlanning].

### 6.3 LiDAR-Based Localization Families

- Follow the F1TENTH lecture flow first: scan matching/ICP, then fast correspondence search, then particle-filter localization.
- Present AMCL as the project implementation of the particle-filter/map-based branch.
- Add NDT as an extra scan-registration method from our own research.
- Keep feature-based localization as optional short background, not the main structure.

Sources:

- Course structure: [F1TENTHLecture09, F1TENTHLecture10, F1TENTHLecture11].
- Scan matching: [Besl1992ICP, Lu1997ScanMatching, Censi2008PLICP].
- Fast LiDAR sensor-model evaluation related work: [Walsh2017CDDT].
- NDT: [Biber2003NDT, Magnusson2007NDT].
- AMCL/MCL: [Dellaert1999MCL, Fox1999EfficientMCL, Fox1999MarkovLocalization, Thrun2001RobustMCL, Fox2001KLDNIPS, Fox2003KLD].
- Feature-based optional background: [Gonzalez1992Iconic, Zhang2000LineSegments, Borges2004LineExtraction, Nguyen2005LineExtraction].

### 6.4 EKF Fusion and Output Pose

- Explain that AMCL and EKF have different roles.
- AMCL performs LiDAR-map correction with a particle filter.
- EKF then predicts from odometry and corrects with AMCL pose to publish the fused pose.
- Mention the limitation: EKF assumes one approximately Gaussian pose belief.

Sources:

- Kalman filtering foundation: [Kalman1960Filter].
- EKF mobile robot localization example: [Ahmad2013EKFLocalization].

### 6.5 Method Selection

- Compare requirements:
  - real-time operation,
  - known 2D map,
  - odometry plus LiDAR available,
  - need to correct odometry drift,
  - need tunable compute cost.
- State why AMCL is selected as the LiDAR-map localization method.
- State why EKF is used as the final fusion/output layer.
- Mention expected limitations: map quality, particle count, covariance tuning, latency.

Sources:

- MCL efficiency and particle-based localization: [Dellaert1999MCL, Fox1999EfficientMCL].
- Robust/adaptive particle filtering concepts: [Thrun2001RobustMCL, Fox2003KLD].

## 7. Claims That Need Project-Specific Evidence Later

These claims should be backed by your own benchmark results, not only papers:

- AMCL latency is low enough for the control loop at 2-10 m/s.
- Particle count selected gives acceptable position/yaw error.
- Odometry drift over one lap is larger than AMCL error.
- Kinematic odometry covariance is sufficient for corners/chicanes.
- AMCL does not lag or snap after high-curvature sections.
- EKF smoothing/fusion improves output stability without adding unacceptable lag.

Planned project figures:

- Pure odometry vs OptiTrack drift over distance/lap.
- AMCL vs OptiTrack position error and yaw error.
- Particle count vs accuracy and computation time.
- LiDAR beam count vs accuracy and computation time.
- Example track plot showing AMCL correction after odometry drift.

## 8. Sources

Course guides are used to structure the LiDAR discussion. Final academic claims should cite the papers, proceedings, articles, or books below. No Wikipedia or informal forum sources.

### Course Guides Used for Structure

- [F1TENTHLecture09] F1TENTH Course Kit, "Lecture 9 - Scan Matching I," Module C: Mapping & Localization. Link: https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleC/lecture09.html
- [F1TENTHLecture10] F1TENTH Course Kit, "Lecture 10 - Scan Matching II," Module C: Mapping & Localization. Link: https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleC/lecture10.html
- [F1TENTHLecture11] F1TENTH Course Kit, "Lecture 11 - Particle Filters," Module C: Mapping & Localization. Link: https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleC/lecture11.html

### Platform / Context

- [OKelly2020F1TENTH] M. O'Kelly, H. Zheng, D. Karthik, and R. Mangharam, "F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning," Proceedings of Machine Learning Research, vol. 123, pp. 77-89, 2020. Link: https://proceedings.mlr.press/v123/o-kelly20a.html

### Odometry and Vehicle Models

- [Borenstein1996OdometryErrors] J. Borenstein and L. Feng, "Measurement and correction of systematic odometry errors in mobile robots," IEEE Transactions on Robotics and Automation, vol. 12, no. 6, pp. 869-880, 1996. DOI: https://doi.org/10.1109/70.544770
- [Borenstein1997Positioning] J. Borenstein, H. R. Everett, L. Feng, and D. Wehe, "Mobile robot positioning: Sensors and techniques," Journal of Robotic Systems, vol. 14, no. 4, pp. 231-249, 1997. DOI: https://doi.org/10.1002/(SICI)1097-4563(199704)14:4%3C231::AID-ROB2%3E3.0.CO;2-R
- [Kong2015VehicleModels] J. Kong, M. Pfeiffer, G. Schildbach, and F. Borrelli, "Kinematic and dynamic vehicle models for autonomous driving control design," 2015 IEEE Intelligent Vehicles Symposium (IV), 2015. DOI: https://doi.org/10.1109/IVS.2015.7225830
- [Polack2017KinematicBicycle] P. Polack, F. Altche, B. d'Andrea-Novel, and A. de La Fortelle, "The kinematic bicycle model: A consistent model for planning feasible trajectories for autonomous vehicles?," 2017 IEEE Intelligent Vehicles Symposium (IV), pp. 812-818, 2017. DOI: https://doi.org/10.1109/IVS.2017.7995816
- [Altche2017DynamicPlanning] F. Altche, P. Polack, and A. de La Fortelle, "High-speed trajectory planning for autonomous vehicles using a simple dynamic model," 2017 IEEE 20th International Conference on Intelligent Transportation Systems (ITSC), 2017. DOI: https://doi.org/10.1109/ITSC.2017.8317632

### Kalman / EKF Fusion

- [Kalman1960Filter] R. E. Kalman, "A new approach to linear filtering and prediction problems," Journal of Basic Engineering, vol. 82, no. 1, pp. 35-45, 1960. DOI: https://doi.org/10.1115/1.3662552
- [Ahmad2013EKFLocalization] H. Ahmad and T. Namerikawa, "Extended Kalman filter-based mobile robot localization with intermittent measurements," Systems Science & Control Engineering, vol. 1, no. 1, pp. 113-126, 2013. DOI: https://doi.org/10.1080/21642583.2013.864249

### Scan Matching

- [Besl1992ICP] P. J. Besl and N. D. McKay, "A method for registration of 3-D shapes," IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 14, no. 2, pp. 239-256, 1992. DOI: https://doi.org/10.1109/34.121791
- [Lu1997ScanMatching] F. Lu and E. Milios, "Robot pose estimation in unknown environments by matching 2D range scans," Journal of Intelligent and Robotic Systems, vol. 18, no. 3, pp. 249-275, 1997. DOI: https://doi.org/10.1023/A:1007957421070
- [Censi2008PLICP] A. Censi, "An ICP variant using a point-to-line metric," 2008 IEEE International Conference on Robotics and Automation (ICRA), 2008. DOI: https://doi.org/10.1109/ROBOT.2008.4543181

### Feature-Based LiDAR

- [Gonzalez1992Iconic] J. Gonzalez, A. Stentz, and A. Ollero, "An iconic position estimator for a 2D laser rangefinder," Proceedings of IEEE International Conference on Robotics and Automation (ICRA), vol. 3, pp. 2646-2651, 1992. Link: https://www.ri.cmu.edu/publications/an-iconic-position-estimator-for-a-2d-laser-rangefinder/
- [Zhang2000LineSegments] L. Zhang and B. K. Ghosh, "Line segment based map building and localization using 2D laser rangefinder," Proceedings 2000 IEEE International Conference on Robotics and Automation (ICRA), vol. 3, pp. 2538-2543, 2000. DOI: https://doi.org/10.1109/ROBOT.2000.846410
- [Borges2004LineExtraction] G. A. Borges and M.-J. Aldon, "Line extraction in 2D range images for mobile robotics," Journal of Intelligent and Robotic Systems, vol. 40, no. 3, pp. 267-297, 2004. DOI: https://doi.org/10.1023/B:JINT.0000038945.55712.65
- [Nguyen2005LineExtraction] V. Nguyen, A. Martinelli, N. Tomatis, and R. Siegwart, "A comparison of line extraction algorithms using 2D laser rangefinder for indoor mobile robotics," 2005 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 1929-1934, 2005. DOI: https://doi.org/10.1109/IROS.2005.1545234

### NDT

- [Biber2003NDT] P. Biber and W. Strasser, "The Normal Distributions Transform: A new approach to laser scan matching," Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), vol. 3, pp. 2743-2748, 2003. DOI: https://doi.org/10.1109/IROS.2003.1249285
- [Magnusson2007NDT] M. Magnusson, A. J. Lilienthal, and T. Duckett, "Scan registration for autonomous mining vehicles using 3D-NDT," Journal of Field Robotics, vol. 24, no. 10, pp. 803-827, 2007. DOI: https://doi.org/10.1002/rob.20204

### AMCL / Monte Carlo Localization

- [Dellaert1999MCL] F. Dellaert, D. Fox, W. Burgard, and S. Thrun, "Monte Carlo localization for mobile robots," Proceedings 1999 IEEE International Conference on Robotics and Automation (ICRA), vol. 2, pp. 1322-1328, 1999. DOI: https://doi.org/10.1109/ROBOT.1999.772544
- [Fox1999EfficientMCL] D. Fox, W. Burgard, F. Dellaert, and S. Thrun, "Monte Carlo Localization: Efficient Position Estimation for Mobile Robots," Proceedings of the Sixteenth National Conference on Artificial Intelligence (AAAI), pp. 343-349, 1999. Link: https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/
- [Fox1999MarkovLocalization] D. Fox, W. Burgard, and S. Thrun, "Markov localization for mobile robots in dynamic environments," Journal of Artificial Intelligence Research, vol. 11, pp. 391-427, 1999. DOI: https://doi.org/10.1613/jair.616
- [Thrun2001RobustMCL] S. Thrun, D. Fox, W. Burgard, and F. Dellaert, "Robust Monte Carlo localization for mobile robots," Artificial Intelligence, vol. 128, no. 1-2, pp. 99-141, 2001. DOI: https://doi.org/10.1016/S0004-3702(01)00069-8
- [Fox2001KLDNIPS] D. Fox, "KLD-Sampling: Adaptive Particle Filters," Advances in Neural Information Processing Systems 14 (NIPS 2001), pp. 713-720, MIT Press, 2001. Link: https://papers.nips.cc/paper_files/paper/2001/hash/c5b2cebf15b205503560c4e8e6d1ea78-Abstract.html
- [Fox2003KLD] D. Fox, "Adapting the sample size in particle filters through KLD-sampling," The International Journal of Robotics Research, vol. 22, no. 12, pp. 985-1003, 2003. DOI: https://doi.org/10.1177/0278364903022012001

### Course-Referenced Runtime / Ray-Casting Resource

- [Walsh2017CDDT] C. H. Walsh and S. Karaman, "CDDT: Fast Approximate 2D Ray Casting for Accelerated Localization," arXiv:1705.01167, 2017. Link: https://arxiv.org/abs/1705.01167

## 9. Next Writing Step

Next step should be one subsection at a time:

1. Write "Odometry as short-term motion prediction" in thesis prose.
2. Write "LiDAR scan matching, ICP, and fast correspondence search" using F1TENTH lectures 9-10 as the structure.
3. Write "Particle filters and AMCL" using F1TENTH lecture 11 as the structure.
4. Write "EKF fusion of odom prediction and AMCL correction".
5. Add "NDT as an extra scan-registration alternative".
6. Add equations for kinematic bicycle odometry, AMCL Bayes-filter update, and EKF prediction/correction.
7. Convert this source list to BibTeX entries.
