function reportPaths = writeOptiTrackQualityReports(results, outputDir, config, showPlot)
%WRITEOPTITRACKQUALITYREPORTS Check OptiTrack ground-truth self-consistency.

if nargin < 4 || isempty(showPlot)
    showPlot = false;
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

summaryRows = defaultQualitySummaryRow();
summaryRows(:) = [];
suspectRows = defaultSuspectSampleRow();
suspectRows(:) = [];

for i = 1:numel(results)
    [summaryRow, resultSuspects] = analyzeOptiTrackQualityResult(results(i), config);
    summaryRows(end + 1) = summaryRow; %#ok<AGROW>
    suspectRows = [suspectRows; resultSuspects(:)]; %#ok<AGROW>
end

summaryTable = struct2table(summaryRows);
suspectTable = struct2table(suspectRows);

reportPaths = struct();
reportPaths.summaryCsv = fullfile(outputDir, 'OptiTrack_Quality_Summary.csv');
reportPaths.suspectCsv = fullfile(outputDir, 'OptiTrack_Quality_SuspectSamples.csv');
reportPaths.trajectoryPlot = fullfile(outputDir, 'OptiTrack_Quality_Trajectory_Flags.png');

writetable(summaryTable, reportPaths.summaryCsv);
writetable(suspectTable, reportPaths.suspectCsv);
plotOptiTrackQualityTrajectoryFlags(results, suspectTable, outputDir, showPlot);

fprintf('\n=== OptiTrack Quality Check ===\n');
disp(summaryTable);
fprintf('Total suspect OptiTrack samples: %d\n', height(suspectTable));
end

function [row, suspectRows] = analyzeOptiTrackQualityResult(result, config)
cfg = readQualityConfig(config);

row = defaultQualitySummaryRow();
row.bagName = string(result.bagName);
row.sourceBagName = string(result.sourceBagName);
row.particle_count = result.particleCount;

suspectRows = defaultSuspectSampleRow();
suspectRows(:) = [];

n = numel(result.tRel);
if n < 1
    return;
end

tRel = result.tRel(:);
if isfield(result, 't') && numel(result.t) == n
    t = result.t(:);
else
    t = tRel;
end
xy = result.gtPos(:, 1:2);
yaw = result.gtYaw(:);
roll = getResultVector(result, 'gtRoll', n);
pitch = getResultVector(result, 'gtPitch', n);

qualityMask = isfinite(t) & all(isfinite(xy), 2) & isfinite(yaw);
if cfg.useMetricMask && isfield(result, 'metricMask') && numel(result.metricMask) == n
    qualityMask = qualityMask & result.metricMask(:);
end

dtAt = nan(n, 1);
stepAt = nan(n, 1);
speedAt = nan(n, 1);
accelAt = nan(n, 1);
yawStepAt = nan(n, 1);
yawRateAt = nan(n, 1);
rollStepAt = nan(n, 1);
pitchStepAt = nan(n, 1);
forwardAxisErrAt = nan(n, 1);

issueNames = ["time_order", "time_gap", "position_step", "speed", ...
    "acceleration", "yaw_step", "yaw_rate", "roll_abs", "pitch_abs", ...
    "roll_step", "pitch_step", "forward_axis"];
issueFlags = false(n, numel(issueNames));
enabledIssueMask = ismember(issueNames, cfg.enabledChecks);

runs = findQualityRuns(result, qualityMask, t);
for r = 1:size(runs, 1)
    idx = (runs(r, 1):runs(r, 2))';
    if numel(idx) < 2
        continue;
    end

    dt = diff(t(idx));
    dxy = diff(xy(idx, :), 1, 1);
    step = hypot(dxy(:, 1), dxy(:, 2));
    speed = nan(size(step));
    validDt = dt > eps;
    speed(validDt) = step(validDt) ./ dt(validDt);

    yawStep = wrapAnglePi(diff(yaw(idx)));
    yawRate = nan(size(yawStep));
    yawRate(validDt) = yawStep(validDt) ./ dt(validDt);

    rollStep = wrapAnglePi(diff(roll(idx)));
    pitchStep = wrapAnglePi(diff(pitch(idx)));

    stepTarget = idx(2:end);
    dtAt(stepTarget) = dt;
    stepAt(stepTarget) = step;
    speedAt(stepTarget) = speed;
    yawStepAt(stepTarget) = yawStep;
    yawRateAt(stepTarget) = yawRate;
    rollStepAt(stepTarget) = rollStep;
    pitchStepAt(stepTarget) = pitchStep;

    issueFlags(stepTarget, 1) = isfinite(dt) & dt <= eps;
    issueFlags(stepTarget, 2) = isfinite(dt) & dt > cfg.maxGapS;
    issueFlags(stepTarget, 3) = isfinite(step) & step > cfg.maxStepM;
    issueFlags(stepTarget, 4) = isfinite(speed) & speed > cfg.maxSpeedMps;
    issueFlags(stepTarget, 6) = isfinite(yawStep) & abs(yawStep) > cfg.maxYawStepRad;
    issueFlags(stepTarget, 7) = isfinite(yawRate) & abs(yawRate) > cfg.maxYawRateRadps;
    issueFlags(stepTarget, 10) = isfinite(rollStep) & abs(rollStep) > cfg.maxRollPitchStepRad;
    issueFlags(stepTarget, 11) = isfinite(pitchStep) & abs(pitchStep) > cfg.maxRollPitchStepRad;

    if cfg.forwardAxisEnabled
        forwardAxisErr = nan(size(step));
        moving = validDt & step >= cfg.minMotionStepM;
        motionHeading = atan2(dxy(:, 2), dxy(:, 1));
        targetYaw = yaw(stepTarget);
        forwardAxisErr(moving) = wrapAnglePi(motionHeading(moving) - targetYaw(moving));
        forwardAxisErrAt(stepTarget) = forwardAxisErr;
        issueFlags(stepTarget, 12) = isfinite(forwardAxisErr) & ...
            abs(forwardAxisErr) > cfg.maxForwardAxisErrorRad;
    end

    if numel(idx) >= 3
        accelTarget = idx(3:end);
        accelDt = diff(t(idx(2:end)));
        accel = nan(numel(accelTarget), 1);
        for k = 1:numel(accelTarget)
            if accelDt(k) > eps && isfinite(speed(k)) && isfinite(speed(k + 1))
                accel(k) = abs(speed(k + 1) - speed(k)) / accelDt(k);
            end
        end
        accelAt(accelTarget) = accel;
        issueFlags(accelTarget, 5) = isfinite(accel) & accel > cfg.maxAccelMps2;
    end
end

issueFlags(:, 8) = qualityMask & isfinite(roll) & abs(roll) > cfg.maxRollAbsRad;
issueFlags(:, 9) = qualityMask & isfinite(pitch) & abs(pitch) > cfg.maxPitchAbsRad;
issueFlags(:, ~enabledIssueMask) = false;

flagMask = any(issueFlags, 2);
flagIdx = find(flagMask);

row.n_quality_samples = nnz(qualityMask);
row.n_flagged_samples = numel(flagIdx);
row.quality_pass = row.n_quality_samples >= 2 && row.n_flagged_samples == 0;
row.duration_s = maxFinite(t(qualityMask)) - minFinite(t(qualityMask));
row.median_dt_ms = 1000 * medianFinite(dtAt(qualityMask));
row.max_dt_s = maxFinite(dtAt(qualityMask));
row.max_step_m = maxFinite(stepAt(qualityMask));
row.max_speed_mps = maxFinite(speedAt(qualityMask));
row.max_accel_mps2 = maxFinite(accelAt(qualityMask));
row.max_abs_yaw_step_deg = rad2deg(maxAbsFinite(yawStepAt(qualityMask)));
row.max_abs_yaw_rate_radps = maxAbsFinite(yawRateAt(qualityMask));
row.max_abs_roll_deg = rad2deg(maxAbsFinite(roll(qualityMask)));
row.max_abs_pitch_deg = rad2deg(maxAbsFinite(pitch(qualityMask)));
row.max_abs_roll_step_deg = rad2deg(maxAbsFinite(rollStepAt(qualityMask)));
row.max_abs_pitch_step_deg = rad2deg(maxAbsFinite(pitchStepAt(qualityMask)));
row.max_abs_forward_axis_error_deg = rad2deg(maxAbsFinite(forwardAxisErrAt(qualityMask)));

counts = sum(issueFlags, 1);
row.n_time_order_flags = counts(1);
row.n_gap_flags = counts(2);
row.n_position_step_flags = counts(3);
row.n_speed_flags = counts(4);
row.n_accel_flags = counts(5);
row.n_yaw_step_flags = counts(6);
row.n_yaw_rate_flags = counts(7);
row.n_roll_abs_flags = counts(8);
row.n_pitch_abs_flags = counts(9);
row.n_roll_step_flags = counts(10);
row.n_pitch_step_flags = counts(11);
row.n_forward_axis_flags = counts(12);

for k = 1:numel(flagIdx)
    sampleIdx = flagIdx(k);
    s = defaultSuspectSampleRow();
    s.bagName = string(result.bagName);
    s.sourceBagName = string(result.sourceBagName);
    s.particle_count = result.particleCount;
    s.sample_index = sampleIdx;
    s.t_rel_s = tRel(sampleIdx);
    s.x_m = xy(sampleIdx, 1);
    s.y_m = xy(sampleIdx, 2);
    s.reasons = strjoin(issueNames(issueFlags(sampleIdx, :)), ';');
    s.dt_s = dtAt(sampleIdx);
    s.step_m = stepAt(sampleIdx);
    s.speed_mps = speedAt(sampleIdx);
    s.accel_mps2 = accelAt(sampleIdx);
    s.yaw_deg = rad2deg(yaw(sampleIdx));
    s.roll_deg = rad2deg(roll(sampleIdx));
    s.pitch_deg = rad2deg(pitch(sampleIdx));
    s.yaw_step_deg = rad2deg(yawStepAt(sampleIdx));
    s.yaw_rate_radps = yawRateAt(sampleIdx);
    s.roll_step_deg = rad2deg(rollStepAt(sampleIdx));
    s.pitch_step_deg = rad2deg(pitchStepAt(sampleIdx));
    s.forward_axis_error_deg = rad2deg(forwardAxisErrAt(sampleIdx));
    suspectRows(end + 1) = s; %#ok<AGROW>
end
end

function plotOptiTrackQualityTrajectoryFlags(results, suspectTable, outputDir, showPlot)
fig = makePlotFigure('OptiTrack Quality Trajectory Flags', showPlot);
n = max(numel(results), 1);
nCols = ceil(sqrt(n));
nRows = ceil(n / nCols);
tiledlayout(nRows, nCols, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:numel(results)
    nexttile;
    result = results(i);
    xy = result.gtPos(:, 1:2);
    usedMask = true(size(xy, 1), 1);
    if isfield(result, 'metricMask') && numel(result.metricMask) == size(xy, 1)
        usedMask = result.metricMask(:);
    end

    plot(xy(:, 1), xy(:, 2), '-', 'Color', [0.80 0.80 0.80], 'LineWidth', 0.8);
    hold on;
    plot(xy(usedMask, 1), xy(usedMask, 2), '.', 'Color', [0.00 0.45 0.74], 'MarkerSize', 4);

    flagIdx = [];
    if height(suspectTable) > 0
        tableMask = suspectTable.bagName == string(result.bagName);
        flagIdx = suspectTable.sample_index(tableMask);
        flagIdx = flagIdx(isfinite(flagIdx) & flagIdx >= 1 & flagIdx <= size(xy, 1));
    end
    if ~isempty(flagIdx)
        scatter(xy(flagIdx, 1), xy(flagIdx, 2), 26, 'r', 'filled');
    end

    axis equal;
    grid on;
    title(sprintf('%s (%d flags)', result.bagName, numel(unique(flagIdx))), ...
        'Interpreter', 'none');
    xlabel('map x [m]');
    ylabel('map y [m]');
end

savePlotFigure(fig, outputDir, 'OptiTrack_Quality_Trajectory_Flags', showPlot);
end

function row = defaultQualitySummaryRow()
row = struct( ...
    'bagName', "", ...
    'sourceBagName', "", ...
    'particle_count', NaN, ...
    'quality_pass', false, ...
    'duration_s', NaN, ...
    'n_quality_samples', 0, ...
    'n_flagged_samples', 0, ...
    'median_dt_ms', NaN, ...
    'max_dt_s', NaN, ...
    'max_step_m', NaN, ...
    'max_speed_mps', NaN, ...
    'max_accel_mps2', NaN, ...
    'max_abs_yaw_step_deg', NaN, ...
    'max_abs_yaw_rate_radps', NaN, ...
    'max_abs_roll_deg', NaN, ...
    'max_abs_pitch_deg', NaN, ...
    'max_abs_roll_step_deg', NaN, ...
    'max_abs_pitch_step_deg', NaN, ...
    'max_abs_forward_axis_error_deg', NaN, ...
    'n_time_order_flags', 0, ...
    'n_gap_flags', 0, ...
    'n_position_step_flags', 0, ...
    'n_speed_flags', 0, ...
    'n_accel_flags', 0, ...
    'n_yaw_step_flags', 0, ...
    'n_yaw_rate_flags', 0, ...
    'n_roll_abs_flags', 0, ...
    'n_pitch_abs_flags', 0, ...
    'n_roll_step_flags', 0, ...
    'n_pitch_step_flags', 0, ...
    'n_forward_axis_flags', 0);
end

function row = defaultSuspectSampleRow()
row = struct( ...
    'bagName', "", ...
    'sourceBagName', "", ...
    'particle_count', NaN, ...
    'sample_index', NaN, ...
    't_rel_s', NaN, ...
    'x_m', NaN, ...
    'y_m', NaN, ...
    'reasons', "", ...
    'dt_s', NaN, ...
    'step_m', NaN, ...
    'speed_mps', NaN, ...
    'accel_mps2', NaN, ...
    'yaw_deg', NaN, ...
    'roll_deg', NaN, ...
    'pitch_deg', NaN, ...
    'yaw_step_deg', NaN, ...
    'yaw_rate_radps', NaN, ...
    'roll_step_deg', NaN, ...
    'pitch_step_deg', NaN, ...
    'forward_axis_error_deg', NaN);
end

function cfg = readQualityConfig(config)
cfg = struct();
cfg.useMetricMask = getConfigLogicalLocal(config, 'optitrackQualityUseMetricMask', true);
cfg.enabledChecks = getConfigStringVectorLocal(config, 'optitrackQualityEnabledChecks', ...
    ["position_step", "roll_step", "forward_axis"]);
cfg.maxGapS = getConfigScalarLocal(config, 'optitrackQualityMaxGapS', 0.05);
cfg.maxStepM = getConfigScalarLocal(config, 'optitrackQualityMaxStepM', 0.20);
cfg.maxSpeedMps = getConfigScalarLocal(config, 'optitrackQualityMaxSpeedMps', ...
    getConfigScalarLocal(config, 'optitrackMaxSpeedMps', inf));
cfg.maxAccelMps2 = getConfigScalarLocal(config, 'optitrackQualityMaxAccelMps2', 40.0);
cfg.maxYawStepRad = getConfigScalarLocal(config, 'optitrackQualityMaxYawStepRad', 30 * pi / 180);
cfg.maxYawRateRadps = getConfigScalarLocal(config, 'optitrackQualityMaxYawRateRadps', 12.0);
cfg.maxRollAbsRad = getConfigScalarLocal(config, 'optitrackQualityMaxRollAbsRad', 15 * pi / 180);
cfg.maxPitchAbsRad = getConfigScalarLocal(config, 'optitrackQualityMaxPitchAbsRad', 15 * pi / 180);
cfg.maxRollPitchStepRad = getConfigScalarLocal(config, 'optitrackQualityMaxRollPitchStepRad', 10 * pi / 180);
cfg.forwardAxisEnabled = getConfigLogicalLocal(config, 'optitrackQualityForwardAxisEnabled', true);
cfg.minMotionStepM = getConfigScalarLocal(config, 'optitrackQualityMinMotionStepM', 0.01);
cfg.maxForwardAxisErrorRad = getConfigScalarLocal(config, ...
    'optitrackQualityMaxForwardAxisErrorRad', 75 * pi / 180);
end

function runs = findQualityRuns(result, qualityMask, t)
runs = [];
if isfield(result, 'laps') && ~isempty(result.laps)
    for i = 1:numel(result.laps)
        lap = result.laps(i);
        if ~isfield(lap, 't') || numel(lap.t) < 2
            continue;
        end
        lapStart = min(lap.t);
        lapEnd = max(lap.t);
        lapMask = qualityMask & t >= lapStart & t <= lapEnd;
        runs = [runs; findTrueRuns(lapMask)]; %#ok<AGROW>
    end
end

if isempty(runs)
    runs = findTrueRuns(qualityMask);
else
    runs = unique(runs, 'rows', 'stable');
end
end

function runs = findTrueRuns(mask)
mask = mask(:);
starts = find(diff([false; mask]) == 1);
ends = find(diff([mask; false]) == -1);
runs = [starts, ends];
end

function values = getResultVector(result, fieldName, n)
if isfield(result, fieldName) && numel(result.(fieldName)) == n
    values = result.(fieldName)(:);
else
    values = nan(n, 1);
end
end

function value = maxFinite(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = max(x);
end
end

function value = minFinite(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = min(x);
end
end

function value = medianFinite(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = median(x);
end
end

function value = maxAbsFinite(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = max(abs(x));
end
end

function value = getConfigScalarLocal(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if isnumeric(candidate) && isscalar(candidate)
        value = double(candidate);
    end
end
end

function value = getConfigLogicalLocal(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if islogical(candidate) && isscalar(candidate)
        value = candidate;
    elseif isnumeric(candidate) && isscalar(candidate)
        value = candidate ~= 0;
    end
end
end

function value = getConfigStringVectorLocal(config, fieldName, defaultValue)
value = string(defaultValue);
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if isstring(candidate) || ischar(candidate) || iscellstr(candidate)
        value = string(candidate);
        value = value(strlength(value) > 0);
    end
end
end

function a = wrapAnglePi(a)
a = atan2(sin(a), cos(a));
end
