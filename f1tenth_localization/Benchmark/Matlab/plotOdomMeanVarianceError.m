clc;
close all;

% Generate the combined odometry drift plot used in the report.

matlabRootDir = '/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/Matlab';
plotFunctionsDir = fullfile(matlabRootDir, 'matlab plotting functions');
if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    error('Could not find plotting functions directory: %s', plotFunctionsDir);
end

odomBagRootDir = fullfile(matlabRootDir, '..', 'bags', 'OptitrackBags', 'ODOM');
odomCsvRootDir = fullfile(matlabRootDir, 'csv', 'Odom');
outputDir = fullfile(matlabRootDir, 'plots', 'OptiTrackBenchmark', 'ODOM_full_initial_aligned');
reportImageDir = '/home/pascal/Documents/BachelorProject/Report/Sections/Localization/Odom/Images';
showPlot = false;

config = struct();
config.ekfTopic = '/odom_pose';
config.optitrackTopic = '/vrpn_mocap/Car2/pose';
config.staticTfTopic = '/tf_static';
config.mapTopic = '/map';
config.mapFrame = 'map';
config.optitrackFrame = 'world';
config.skipStartupAndIncompleteLaps = false;
config.optitrackExpectedHz = 240;
config.ekfExpectedHz = 240;
config.localMotionIntervalS = 0.025;
config.localMotionBaseMessageCount = 5;
config.localMotionMessageCounts = [5, 10, 15, 20];
config.localMotionMaxSampleGapS = config.localMotionIntervalS / 2;

config.lapCloseRadiusM = 0.45;
config.lapRearmRadiusM = 0.90;
config.minLapDurationS = 2.0;
config.minLapDistanceM = 2.0;

config.optitrackDropoutZeroRadiusM = 1e-6;
config.optitrackFreezeDistanceM = 1e-6;
config.optitrackMaxSpeedMps = 12.0;

config.startCalibrationEnabled = false;
config.startCalibrationDurationS = 3.0;
config.startCalibrationMinSamples = 50;
config.startCalibrationMaxStdM = 0.03;
config.startCalibrationMaxTravelM = 0.05;

config.optitrackFreezeZoneExcludeEnabled = false;
config.optitrackFreezeZoneWindowS = 0.05;
config.optitrackFreezeZoneMaxGtTravelM = 0.02;
config.optitrackFreezeZoneMinEkfTravelM = 0.10;
config.optitrackFreezeZonePaddingS = 0.05;

config.excludeYawOutlierLaps = false;
config.yawOutlierLapThresholdRad = pi / 2;
config.yawOutlierLapParticleCounts = [];
config.yawIsolatedOutlierFilterEnabled = false;
config.yawIsolatedOutlierThresholdRad = pi / 2;
config.yawIsolatedOutlierMaxRunLength = 2;

config.metricExcludeXLessThanM = -Inf;
config.optitrackQualityEnabled = false;
config.bagStartEpochSeconds = readOdomCsvStartEpochSeconds(odomCsvRootDir);
config.bagStartEpochToleranceS = 2.0;
config.driveCommandTopics = {'/drive', '/ackermann_cmd'};
config.driveCommandSpeedThresholdMps = 0.05;
config.driveCommandAccelerationThreshold = 0.05;
config.driveWindowPreStartS = 0.2;
config.driveWindowHelperScript = fullfile(matlabRootDir, 'read_drive_window.py');
config.crashYawDropDetectionEnabled = true;
config.crashYawDropThresholdDeg = 20.0;
config.crashYawDropMinBeforeDeg = 30.0;
config.odomOptitrackQualityFilterEnabled = true;
config.odomOptitrackMaxYawRateRadps = 3.0;
config.odomOptitrackYawRateWindowS = 0.10;
config.odomOptitrackYawJumpThresholdDeg = 45.0;
config.odomOptitrackStallWindowS = 0.05;
config.odomOptitrackStallMaxTravelM = 0.02;
config.odomOptitrackStallMinOdomTravelM = 0.06;
config.odomOptitrackMotionWindowS = 0.20;
config.odomOptitrackMinMotionTravelM = 0.08;
config.odomOptitrackMaxBackwardM = -0.04;
config.odomOptitrackMinSidewaysM = 0.06;
config.odomOptitrackSidewaysRatio = 1.2;
config.odomOptitrackJumpWindowS = 0.05;
config.odomOptitrackJumpMinTravelM = 0.40;
config.odomOptitrackIgnoreBeforeMotion = true;
config.odomOptitrackMotionStartTravelM = 0.05;
config.odomOptitrackMaxYawErrorArtifactDeg = 90.0;

fprintf('ODOM bag root : %s\n', odomBagRootDir);
fprintf('ODOM CSV runs : %d\n', numel(config.bagStartEpochSeconds));

results = loadOptiTrackBenchmarkBags(odomBagRootDir, config);
results = alignOdomResultsToGroundTruthStart(results);
results = applyOdomOptiTrackQualityFilter(results, config);
if isempty(results)
    error('No ODOM benchmark bags were loaded from %s', odomBagRootDir);
end

[trajectorySummaryTable] = plotOdomTrajectoryComparisonForReport( ...
    results, outputDir, reportImageDir, showPlot);
[summaryTable, aggregateTable] = plotOdomMeanVariance(results, outputDir, reportImageDir, showPlot, config);
[localMotionSampleTable, localMotionSummaryTable] = plotOdomLocalMotionDrift( ...
    results, outputDir, reportImageDir, showPlot, config);
writetable(trajectorySummaryTable, fullfile(outputDir, 'Odom_Trajectory_Plot_Summary.csv'));
writetable(summaryTable, fullfile(outputDir, 'Odom_Run_Error_Summary.csv'));
writetable(aggregateTable, fullfile(outputDir, 'Odom_Mean_Variance_Summary.csv'));
writetable(localMotionSampleTable, fullfile(outputDir, 'Odom_Local_Motion_Drift_Samples.csv'));
writetable(localMotionSummaryTable, fullfile(outputDir, 'Odom_Local_Motion_Drift_Summary.csv'));

fprintf('Report image saved to %s\n', ...
    fullfile(reportImageDir, 'odom_position_error_mean_std.png'));
fprintf('Local motion image saved to %s\n', ...
    fullfile(reportImageDir, 'odom_local_motion_drift_boxplot.png'));

function epochSeconds = readOdomCsvStartEpochSeconds(odomCsvRootDir)
files = dir(fullfile(odomCsvRootDir, 'Bag*', 'pipeline_latency_*.csv'));
epochSeconds = nan(numel(files), 1);
for i = 1:numel(files)
    token = regexp(files(i).name, 'pipeline_latency_(\d+)\.csv', 'tokens', 'once');
    if ~isempty(token)
        epochSeconds(i) = str2double(token{1});
    end
end
epochSeconds = unique(epochSeconds(isfinite(epochSeconds)), 'stable');
end

function results = alignOdomResultsToGroundTruthStart(results)
for i = 1:numel(results)
    if isempty(results(i).gtPos) || isempty(results(i).ekfPos)
        continue;
    end

    gt0 = results(i).gtPos(1, 1:2);
    odom0 = results(i).ekfPos(1, 1:2);
    if isfield(results(i), 'gtYaw') && isfield(results(i), 'ekfYaw') && ...
            ~isempty(results(i).gtYaw) && ~isempty(results(i).ekfYaw)
        yawOffset = results(i).gtYaw(1) - results(i).ekfYaw(1);
    else
        yawOffset = 0;
    end

    results(i) = alignSingleOdomResult(results(i), gt0, odom0, yawOffset);
end
end

function result = alignSingleOdomResult(result, gt0, odom0, yawOffset)
R = [cos(yawOffset), -sin(yawOffset); sin(yawOffset), cos(yawOffset)];
result.ekfPos(:, 1:2) = ((R * (result.ekfPos(:, 1:2) - odom0)')' + gt0);
if isfield(result, 'ekfYaw') && ~isempty(result.ekfYaw)
    result.ekfYaw = wrapAnglePiLocal(result.ekfYaw + yawOffset);
end
result = recomputePositionErrors(result);
end

function result = recomputePositionErrors(result)
result.xError = result.ekfPos(:, 1) - result.gtPos(:, 1);
result.yError = result.ekfPos(:, 2) - result.gtPos(:, 2);
result.xyError = hypot(result.xError, result.yError);
end

function results = applyOdomOptiTrackQualityFilter(results, config)
if ~getConfigLogicalValue(config, 'odomOptitrackQualityFilterEnabled', true)
    return;
end

for i = 1:numel(results)
    n = numel(results(i).t);
    if n < 2
        continue;
    end

    issueMasks = detectOdomOptiTrackFaultMasks(results(i), config);
    positionBad = issueMasks.stall | issueMasks.nonholonomic | issueMasks.positionJump;
    yawBad = issueMasks.yawRate | issueMasks.yawJump | issueMasks.nonholonomic;
    highYawError = false(n, 1);
    maxYawErrorArtifactDeg = getConfigScalarValue(config, ...
        'odomOptitrackMaxYawErrorArtifactDeg', Inf);
    if isfinite(maxYawErrorArtifactDeg)
        highYawError = abs(rad2deg(results(i).yawError(:))) > maxYawErrorArtifactDeg;
        yawBad = yawBad | highYawError;
    end
    positionBad = positionBad(:);
    yawBad = yawBad(:);

    results(i).xError(positionBad) = NaN;
    results(i).yError(positionBad) = NaN;
    results(i).xyError(positionBad) = NaN;
    results(i).yawError(yawBad) = NaN;
    results(i).odomOptitrackPositionValidMask = ~positionBad;
    results(i).odomOptitrackYawValidMask = ~yawBad;
    results(i).odomOptitrackPositionRejectedSamples = nnz(positionBad);
    results(i).odomOptitrackYawRejectedSamples = nnz(yawBad);

    if isfield(results(i), 'metricMask') && numel(results(i).metricMask) == n
        results(i).metricMask = results(i).metricMask(:) & ~positionBad;
    end

    fprintf(['ODOM OptiTrack filter %s: rejected position %d/%d, yaw %d/%d samples ', ...
        '(stall %d, yaw-rate %d, yaw-jump %d, non-holonomic %d, jump %d, high-yaw %d)\n'], ...
        results(i).bagName, nnz(positionBad), n, nnz(yawBad), n, ...
        nnz(issueMasks.stall), nnz(issueMasks.yawRate), nnz(issueMasks.yawJump), ...
        nnz(issueMasks.nonholonomic), nnz(issueMasks.positionJump), nnz(highYawError));
end
end

function issueMasks = detectOdomOptiTrackFaultMasks(result, config)
n = numel(result.t);
t = result.t(:);
gtXY = result.gtPos(:, 1:2);
odomXY = result.ekfPos(:, 1:2);
gtYaw = result.gtYaw(:);

issueMasks = struct();
issueMasks.stall = false(n, 1);
issueMasks.yawRate = false(n, 1);
issueMasks.yawJump = false(n, 1);
issueMasks.nonholonomic = false(n, 1);
issueMasks.positionJump = false(n, 1);
if n < 2
    return;
end

issueMasks.stall = slidingWindowTravelMask(t, gtXY, odomXY, ...
    getConfigScalarValue(config, 'odomOptitrackStallWindowS', 0.05), ...
    getConfigScalarValue(config, 'odomOptitrackStallMaxTravelM', 0.02), ...
    getConfigScalarValue(config, 'odomOptitrackStallMinOdomTravelM', 0.06));
issueMasks.yawRate = yawRateWindowMask(t, gtYaw, ...
    getConfigScalarValue(config, 'odomOptitrackYawRateWindowS', 0.10), ...
    getConfigScalarValue(config, 'odomOptitrackMaxYawRateRadps', 3.0));

yawJumpThreshold = deg2rad(getConfigScalarValue(config, ...
    'odomOptitrackYawJumpThresholdDeg', 45.0));
dyaw = wrapAnglePiLocal(diff(gtYaw));
badYawJumpTarget = find(abs(dyaw) > yawJumpThreshold) + 1;
issueMasks.yawJump(badYawJumpTarget) = true;

issueMasks.nonholonomic = nonholonomicMotionMask(t, gtXY, gtYaw, config);
issueMasks.positionJump = positionJumpMask(t, gtXY, ...
    getConfigScalarValue(config, 'odomOptitrackJumpWindowS', 0.05), ...
    getConfigScalarValue(config, 'odomOptitrackJumpMinTravelM', 0.40));

motionMask = odomOptitrackAnalysisMotionMask(t, odomXY, config);
issueMasks.stall = issueMasks.stall & motionMask;
issueMasks.yawRate = issueMasks.yawRate & motionMask;
issueMasks.yawJump = issueMasks.yawJump & motionMask;
issueMasks.nonholonomic = issueMasks.nonholonomic & motionMask;
issueMasks.positionJump = issueMasks.positionJump & motionMask;
end

function mask = yawRateWindowMask(t, yaw, windowS, maxYawRateRadps)
n = numel(t);
mask = false(n, 1);
j = 1;
for i = 2:n
    while j < i && t(i) - t(j) > windowS
        j = j + 1;
    end
    if j >= i || ~isfinite(yaw(i)) || ~isfinite(yaw(j))
        continue;
    end

    dt = t(i) - t(j);
    if dt <= eps
        continue;
    end

    yawRate = abs(wrapAnglePiLocal(yaw(i) - yaw(j))) / dt;
    if yawRate > maxYawRateRadps
        mask(i) = true;
    end
end
end

function mask = slidingWindowTravelMask(t, optiXY, odomXY, windowS, maxOptiTravelM, minOdomTravelM)
n = numel(t);
mask = false(n, 1);
j = 1;
for i = 2:n
    while j < i && t(i) - t(j) > windowS
        j = j + 1;
    end
    if j >= i || ~all(isfinite(optiXY([j i], :)), 'all') || ...
            ~all(isfinite(odomXY([j i], :)), 'all')
        continue;
    end

    optiTravel = norm(optiXY(i, :) - optiXY(j, :));
    odomTravel = norm(odomXY(i, :) - odomXY(j, :));
    if optiTravel <= maxOptiTravelM && odomTravel >= minOdomTravelM
        mask(j:i) = true;
    end
end
end

function mask = nonholonomicMotionMask(t, xy, yaw, config)
n = numel(t);
mask = false(n, 1);
j = 1;
windowS = getConfigScalarValue(config, 'odomOptitrackMotionWindowS', 0.20);
minTravelM = getConfigScalarValue(config, 'odomOptitrackMinMotionTravelM', 0.08);
maxBackwardM = getConfigScalarValue(config, 'odomOptitrackMaxBackwardM', -0.04);
minSidewaysM = getConfigScalarValue(config, 'odomOptitrackMinSidewaysM', 0.06);
sidewaysRatio = getConfigScalarValue(config, 'odomOptitrackSidewaysRatio', 1.2);
for i = 2:n
    while j < i && t(i) - t(j) > windowS
        j = j + 1;
    end
    if j >= i || ~all(isfinite(xy([j i], :)), 'all') || ~isfinite(yaw(i))
        continue;
    end

    dxy = xy(i, :) - xy(j, :);
    travel = norm(dxy);
    if travel < minTravelM
        continue;
    end

    forward = [cos(yaw(i)), sin(yaw(i))];
    longitudinal = dot(dxy, forward);
    lateral = dxy(1) * (-forward(2)) + dxy(2) * forward(1);
    sideways = abs(lateral) >= minSidewaysM && ...
        abs(lateral) >= sidewaysRatio * max(abs(longitudinal), eps);
    backwards = longitudinal <= maxBackwardM;
    if sideways || backwards
        mask(i) = true;
    end
end
end

function mask = positionJumpMask(t, optiXY, windowS, minOptiTravelM)
n = numel(t);
mask = false(n, 1);
j = 1;
for i = 2:n
    while j < i && t(i) - t(j) > windowS
        j = j + 1;
    end
    if j >= i || ~all(isfinite(optiXY([j i], :)), 'all')
        continue;
    end

    optiTravel = norm(optiXY(i, :) - optiXY(j, :));
    if optiTravel >= minOptiTravelM
        mask(j:i) = true;
    end
end
end

function mask = odomOptitrackAnalysisMotionMask(t, odomXY, config)
n = numel(t);
mask = true(n, 1);
if ~getConfigLogicalValue(config, 'odomOptitrackIgnoreBeforeMotion', true) || n < 2
    return;
end

valid = isfinite(t) & all(isfinite(odomXY), 2);
if ~any(valid)
    return;
end

idx0 = find(valid, 1, 'first');
travelThresholdM = getConfigScalarValue(config, 'odomOptitrackMotionStartTravelM', 0.05);
dxyFromStart = odomXY - odomXY(idx0, :);
travelFromStart = hypot(dxyFromStart(:, 1), dxyFromStart(:, 2));
idxStart = find(valid & travelFromStart >= travelThresholdM, 1, 'first');
if isempty(idxStart)
    mask(:) = false;
    return;
end

mask = t >= t(idxStart);
end

function value = getConfigScalarValue(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if isnumeric(candidate) && isscalar(candidate)
        value = double(candidate);
    end
end
end

function value = getConfigLogicalValue(config, fieldName, defaultValue)
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

function summaryTable = plotOdomTrajectoryComparisonForReport(results, outputDir, reportImageDir, showPlot)
fig = makePlotFigure('OptiTrack vs Odom Trajectory', showPlot);
set(fig, 'Position', [100, 100, 1300, 900]);
ax = axes(fig);
plotOdomTrajectoryAxes(ax, results, [], NaN, NaN, 25, ...
    '', true);
savePlotFigure(fig, outputDir, 'Odom_Trajectory_vs_OptiTrack', showPlot);

if ~exist(reportImageDir, 'dir')
    mkdir(reportImageDir);
end
copyfile(fullfile(outputDir, 'Odom_Trajectory_vs_OptiTrack.png'), ...
    fullfile(reportImageDir, 'odom_trajectory_vs_optitrack.png'));

summaryTable = table(numel(results), ...
    'VariableNames', {'runs'});
end

function plotOdomTrajectoryAxes(ax, results, windowInfo, startRelS, endRelS, fontSize, titleText, showLegend)
axes(ax);
hold(ax, 'on');
drawOccupancyMapBackground(getFirstMapData(results));

gtColor = [0.05 0.24 0.55];
odomColor = [0.72 0.23 0.12];
for i = 1:numel(results)
    mask = true(size(results(i).gtPos, 1), 1);
    if ~isempty(windowInfo) && isfinite(startRelS) && isfinite(endRelS)
        startAbsS = max(windowInfo(i).firstDriveAbsS + startRelS, windowInfo(i).startAbsS);
        endAbsS = min(windowInfo(i).firstDriveAbsS + endRelS, windowInfo(i).endAbsS);
        mask = isfinite(results(i).t(:)) & results(i).t(:) >= startAbsS & results(i).t(:) <= endAbsS;
    end
    if isfield(results(i), 'odomOptitrackPositionValidMask') && ...
            numel(results(i).odomOptitrackPositionValidMask) == numel(mask)
        mask = mask & results(i).odomOptitrackPositionValidMask(:);
    end
    if nnz(mask) < 2
        continue;
    end

    if i == 1
        gtVisibility = 'on';
        odomVisibility = 'on';
    else
        gtVisibility = 'off';
        odomVisibility = 'off';
    end

    gtX = results(i).gtPos(:, 1);
    gtY = results(i).gtPos(:, 2);
    odomX = results(i).ekfPos(:, 1);
    odomY = results(i).ekfPos(:, 2);
    gtX(~mask) = NaN;
    gtY(~mask) = NaN;
    odomX(~mask) = NaN;
    odomY(~mask) = NaN;

    plot(ax, gtX, gtY, '-', ...
        'Color', gtColor, 'LineWidth', 2.7, ...
        'DisplayName', 'OptiTrack', 'HandleVisibility', gtVisibility);
    plot(ax, odomX, odomY, '--', ...
        'Color', odomColor, 'LineWidth', 2.7, ...
        'DisplayName', 'Odometry', 'HandleVisibility', odomVisibility);
end

axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'LineWidth', 1.9, 'FontSize', fontSize, ...
    'GridAlpha', 0.22, 'MinorGridAlpha', 0.12, 'Layer', 'top');
xlabel(ax, 'map x [m]', 'FontSize', fontSize + 4);
ylabel(ax, 'map y [m]', 'FontSize', fontSize + 4);
if strlength(string(titleText)) > 0
    title(ax, titleText, 'FontSize', fontSize + 2);
end
if showLegend
    legend(ax, 'Location', 'northeast', 'LineWidth', 1.2, 'FontSize', fontSize);
end
end

function [summaryTable, aggregateTable] = plotOdomMeanVariance(results, outputDir, reportImageDir, showPlot, config)
windowInfo = repmat(emptyDriveWindowInfo(), numel(results), 1);
valid = false(numel(results), 1);
for i = 1:numel(results)
    windowInfo(i) = findDriveAlignedWindow(results(i), config);
    inWindow = results(i).t(:) >= windowInfo(i).startAbsS & ...
        results(i).t(:) <= windowInfo(i).endAbsS;
    valid(i) = numel(results(i).tRel) >= 2 && ...
        windowInfo(i).ok && ...
        nnz(isfinite(results(i).xyError) & inWindow) >= 2 && ...
        nnz(isfinite(results(i).yawError) & inWindow) >= 2;
end
results = results(valid);
windowInfo = windowInfo(valid);
if isempty(results)
    error('No valid odom error traces available.');
end

commonStartRelS = -config.driveWindowPreStartS;
commonEndRelS = max([windowInfo.endAbsS] - [windowInfo.firstDriveAbsS]);
if isfield(config, 'driveWindowMaxEndRelS') && isfinite(config.driveWindowMaxEndRelS)
    commonEndRelS = min(commonEndRelS, config.driveWindowMaxEndRelS);
end
if commonEndRelS <= commonStartRelS
    error('Drive-aligned odom window is empty.');
end

dt = median(cell2mat(arrayfun(@(r) median(diff(r.tRel(isfinite(r.tRel)))), results, ...
    'UniformOutput', false)), 'omitnan');
if ~isfinite(dt) || dt <= 0
    dt = (commonEndRelS - commonStartRelS) / 1000;
end
tRelGrid = (commonStartRelS:dt:commonEndRelS)';
if numel(tRelGrid) < 200
    tRelGrid = linspace(commonStartRelS, commonEndRelS, 500)';
end

posGrid = nan(numel(tRelGrid), numel(results));
yawGridDeg = nan(numel(tRelGrid), numel(results));
runMeanPos = nan(numel(results), 1);
runRmsePos = nan(numel(results), 1);
runStdPos = nan(numel(results), 1);
runP95Pos = nan(numel(results), 1);
runMaxPos = nan(numel(results), 1);
runMeanYawDeg = nan(numel(results), 1);
runRmseYawDeg = nan(numel(results), 1);
runStdYawDeg = nan(numel(results), 1);
runP95YawDeg = nan(numel(results), 1);
runMaxYawDeg = nan(numel(results), 1);
runDuration = nan(numel(results), 1);
runName = strings(numel(results), 1);
firstDriveAbsS = nan(numel(results), 1);
analysisStartAbsS = nan(numel(results), 1);
analysisEndAbsS = nan(numel(results), 1);
rejectedPositionSamples = nan(numel(results), 1);
rejectedYawSamples = nan(numel(results), 1);

for i = 1:numel(results)
    tAbs = results(i).t(:);
    posErr = results(i).xyError(:);
    yawErrDeg = rad2deg(abs(results(i).yawError(:)));
    posMask = isfinite(tAbs) & isfinite(posErr);
    yawMask = isfinite(tAbs) & isfinite(yawErrDeg);

    [tAbsPosUnique, posUniqueIdx] = unique(tAbs(posMask), 'stable');
    posUnique = posErr(posMask);
    posUnique = posUnique(posUniqueIdx);

    [tAbsYawUnique, yawUniqueIdx] = unique(tAbs(yawMask), 'stable');
    yawUniqueDeg = yawErrDeg(yawMask);
    yawUniqueDeg = yawUniqueDeg(yawUniqueIdx);

    tAbsGrid = windowInfo(i).firstDriveAbsS + tRelGrid;
    beforeWindowMask = tAbsGrid < windowInfo(i).startAbsS;
    posInterp = interp1(tAbsPosUnique, posUnique, tAbsGrid, 'linear', NaN);
    yawInterpDeg = interp1(tAbsYawUnique, yawUniqueDeg, tAbsGrid, 'linear', NaN);
    posInterp(beforeWindowMask) = NaN;
    yawInterpDeg(beforeWindowMask) = NaN;

    effectiveEndAbsS = windowInfo(i).endAbsS;
    crashHoldIdx = detectYawCrashHoldIndex(tAbsGrid, yawInterpDeg, windowInfo(i), config);
    if ~isempty(crashHoldIdx)
        effectiveEndAbsS = min(effectiveEndAbsS, tAbsGrid(crashHoldIdx));
    end
    afterWindowMask = tAbsGrid > effectiveEndAbsS;

    lastPosGridIdx = find(tAbsGrid <= effectiveEndAbsS & isfinite(posInterp), 1, 'last');
    if ~isempty(lastPosGridIdx)
        posInterp(afterWindowMask) = posInterp(lastPosGridIdx);
    else
        posInterp(afterWindowMask) = NaN;
    end

    lastYawGridIdx = find(tAbsGrid <= effectiveEndAbsS & isfinite(yawInterpDeg), 1, 'last');
    if ~isempty(lastYawGridIdx)
        yawInterpDeg(afterWindowMask) = yawInterpDeg(lastYawGridIdx);
    else
        yawInterpDeg(afterWindowMask) = NaN;
    end
    posGrid(:, i) = posInterp;
    yawGridDeg(:, i) = yawInterpDeg;

    posMetricMask = tAbsPosUnique >= windowInfo(i).startAbsS & ...
        tAbsPosUnique <= effectiveEndAbsS;
    yawMetricMask = tAbsYawUnique >= windowInfo(i).startAbsS & ...
        tAbsYawUnique <= effectiveEndAbsS;
    posMetric = posUnique(posMetricMask);
    yawMetricDeg = yawUniqueDeg(yawMetricMask);

    runName(i) = string(results(i).bagName);
    runMeanPos(i) = mean(posMetric, 'omitnan');
    runRmsePos(i) = sqrt(mean(posMetric .^ 2, 'omitnan'));
    runStdPos(i) = std(posMetric, 0, 'omitnan');
    runP95Pos(i) = prctile(posMetric, 95);
    runMaxPos(i) = max(posMetric, [], 'omitnan');
    runMeanYawDeg(i) = mean(yawMetricDeg, 'omitnan');
    runRmseYawDeg(i) = sqrt(mean(yawMetricDeg .^ 2, 'omitnan'));
    runStdYawDeg(i) = std(yawMetricDeg, 0, 'omitnan');
    runP95YawDeg(i) = prctile(yawMetricDeg, 95);
    runMaxYawDeg(i) = max(yawMetricDeg, [], 'omitnan');
    runDuration(i) = effectiveEndAbsS - windowInfo(i).startAbsS;
    firstDriveAbsS(i) = windowInfo(i).firstDriveAbsS;
    analysisStartAbsS(i) = windowInfo(i).startAbsS;
    analysisEndAbsS(i) = effectiveEndAbsS;
    rejectedPositionSamples(i) = getResultScalar(results(i), ...
        'odomOptitrackPositionRejectedSamples', NaN);
    rejectedYawSamples(i) = getResultScalar(results(i), ...
        'odomOptitrackYawRejectedSamples', NaN);
end

meanPos = mean(posGrid, 2, 'omitnan');
stdPos = std(posGrid, 0, 2, 'omitnan');
varPos = var(posGrid, 0, 2, 'omitnan');
meanYawDeg = mean(yawGridDeg, 2, 'omitnan');
stdYawDeg = std(yawGridDeg, 0, 2, 'omitnan');
varYawDeg = var(yawGridDeg, 0, 2, 'omitnan');

fig = makePlotFigure('Odom Trajectory, Position Error, and Yaw Error', showPlot);
set(fig, 'Position', [100, 100, 3200, 950]);
layout = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

ax0 = nexttile(layout, 1);
plotOdomTrajectoryAxes(ax0, results, windowInfo, commonStartRelS, commonEndRelS, 26, ...
    '', true);

ax1 = nexttile(layout, 2);
plotMeanStdBand(ax1, tRelGrid, meanPos, stdPos, [0.20 0.45 0.80], [0.05 0.24 0.55], ...
    'mean position error');
ylabel(ax1, 'position error [m]', 'FontSize', 30);

ax2 = nexttile(layout, 3);
plotMeanStdBand(ax2, tRelGrid, meanYawDeg, stdYawDeg, [0.86 0.44 0.25], [0.68 0.19 0.11], ...
    'mean yaw error');
ylabel(ax2, 'yaw error [deg]', 'FontSize', 30);

axesList = [ax1, ax2];
for ax = axesList
    grid(ax, 'on');
    driveLine = xline(ax, 0, '--', 'first drive', 'Color', [0.25 0.25 0.25], ...
        'LineWidth', 1.7, 'LabelVerticalAlignment', 'bottom', 'FontSize', 22);
    driveLine.HandleVisibility = 'off';
    set(ax, 'LineWidth', 1.9, 'FontSize', 26, ...
        'GridAlpha', 0.22, 'MinorGridAlpha', 0.12);
    xlabel(ax, 'time relative to first drive command [s]', 'FontSize', 30);
    xlim(ax, [commonStartRelS, commonEndRelS]);
end
legend(ax1, 'Location', 'northwest', 'LineWidth', 1.2, 'FontSize', 23);
legend(ax2, 'Location', 'northwest', 'LineWidth', 1.2, 'FontSize', 23);
linkaxes(axesList, 'x');
savePlotFigure(fig, outputDir, 'Odom_Position_Error_Mean_Std', showPlot);

if ~exist(reportImageDir, 'dir')
    mkdir(reportImageDir);
end
copyfile(fullfile(outputDir, 'Odom_Position_Error_Mean_Std.png'), ...
    fullfile(reportImageDir, 'odom_position_error_mean_std.png'));

summaryTable = table(runName, runDuration, firstDriveAbsS, analysisStartAbsS, analysisEndAbsS, ...
    rejectedPositionSamples, rejectedYawSamples, ...
    runMeanPos, runStdPos, runRmsePos, runP95Pos, runMaxPos, ...
    runMeanYawDeg, runStdYawDeg, runRmseYawDeg, runP95YawDeg, runMaxYawDeg, ...
    'VariableNames', {'run', 'duration_s', 'first_drive_abs_s', ...
    'analysis_start_abs_s', 'analysis_end_abs_s', ...
    'rejected_position_samples', 'rejected_yaw_samples', 'mean_position_error_m', ...
    'std_position_error_m', 'rmse_position_error_m', 'p95_position_error_m', ...
    'max_position_error_m', 'mean_yaw_error_deg', 'std_yaw_error_deg', ...
    'rmse_yaw_error_deg', 'p95_yaw_error_deg', 'max_yaw_error_deg'});

aggregateTable = table(numel(results), commonEndRelS - commonStartRelS, ...
    commonStartRelS, commonEndRelS, ...
    mean(runMeanPos, 'omitnan'), std(runMeanPos, 0, 'omitnan'), ...
    mean(runRmsePos, 'omitnan'), mean(runP95Pos, 'omitnan'), ...
    mean(runMaxPos, 'omitnan'), max(meanPos, [], 'omitnan'), max(varPos, [], 'omitnan'), ...
    mean(runMeanYawDeg, 'omitnan'), std(runMeanYawDeg, 0, 'omitnan'), ...
    mean(runRmseYawDeg, 'omitnan'), mean(runP95YawDeg, 'omitnan'), ...
    mean(runMaxYawDeg, 'omitnan'), max(meanYawDeg, [], 'omitnan'), max(varYawDeg, [], 'omitnan'), ...
    'VariableNames', {'runs', 'common_duration_s', 'analysis_start_rel_drive_s', ...
    'analysis_end_rel_drive_s', 'mean_of_run_mean_position_error_m', ...
    'std_of_run_mean_position_error_m', 'mean_run_rmse_position_error_m', ...
    'mean_run_p95_position_error_m', 'mean_run_max_position_error_m', ...
    'max_mean_position_error_m', 'max_time_variance_position_m2', ...
    'mean_of_run_mean_yaw_error_deg', 'std_of_run_mean_yaw_error_deg', ...
    'mean_run_rmse_yaw_error_deg', 'mean_run_p95_yaw_error_deg', ...
    'mean_run_max_yaw_error_deg', 'max_mean_yaw_error_deg', ...
    'max_time_variance_yaw_deg2'});
end

function plotMeanStdBand(ax, x, meanValue, stdValue, bandColor, lineColor, lineName)
upper = meanValue + stdValue;
lower = max(meanValue - stdValue, 0);
hold(ax, 'on');
fill(ax, [x; flipud(x)], [upper; flipud(lower)], bandColor, ...
    'FaceAlpha', 0.24, 'EdgeColor', 'none', ...
    'DisplayName', 'run-to-run standard deviation');
plot(ax, x, meanValue, '-', 'Color', lineColor, 'LineWidth', 3.4, ...
    'DisplayName', lineName);
end

function [sampleTable, summaryTable] = plotOdomLocalMotionDrift( ...
    results, outputDir, reportImageDir, showPlot, config)
baseIntervalS = getConfigScalarValue(config, 'localMotionIntervalS', 0.025);
baseMessageCount = getConfigScalarValue(config, 'localMotionBaseMessageCount', 5);
if ~isfinite(baseIntervalS) || baseIntervalS <= 0
    error('localMotionIntervalS must be positive.');
end
if ~isfinite(baseMessageCount) || baseMessageCount <= 0
    error('localMotionBaseMessageCount must be positive.');
end

intervalMessageCounts = baseMessageCount;
if isfield(config, 'localMotionMessageCounts') && ~isempty(config.localMotionMessageCounts)
    intervalMessageCounts = double(config.localMotionMessageCounts(:)');
end
intervalMessageCounts = unique(intervalMessageCounts( ...
    isfinite(intervalMessageCounts) & intervalMessageCounts > 0), 'stable');
if isempty(intervalMessageCounts)
    error('localMotionMessageCounts must contain at least one positive value.');
end

run = strings(0, 1);
runIndex = zeros(0, 1);
intervalMessageCount = zeros(0, 1);
intervalSColumn = zeros(0, 1);
intervalStartRelDriveS = zeros(0, 1);
intervalEndRelDriveS = zeros(0, 1);
positionDriftM = zeros(0, 1);
yawDriftDeg = zeros(0, 1);
gtTravelM = zeros(0, 1);
odomTravelM = zeros(0, 1);
gtYawDeltaDeg = zeros(0, 1);
odomYawDeltaDeg = zeros(0, 1);

summaryRun = strings(0, 1);
summaryRunIndex = zeros(0, 1);
summaryIntervalMessageCount = zeros(0, 1);
summaryIntervalS = zeros(0, 1);
summaryPositionSamples = zeros(0, 1);
summaryYawSamples = zeros(0, 1);
summaryPositionMeanM = zeros(0, 1);
summaryPositionMedianM = zeros(0, 1);
summaryPositionP95M = zeros(0, 1);
summaryPositionMaxM = zeros(0, 1);
summaryYawMeanDeg = zeros(0, 1);
summaryYawMedianDeg = zeros(0, 1);
summaryYawP95Deg = zeros(0, 1);
summaryYawMaxDeg = zeros(0, 1);

plotRunCount = 0;
for i = 1:numel(results)
    windowInfo = findDriveAlignedWindow(results(i), config);
    if ~windowInfo.ok
        continue;
    end

    runAccepted = false;
    currentRunIndex = NaN;
    for intervalIdx = 1:numel(intervalMessageCounts)
        messageCount = intervalMessageCounts(intervalIdx);
        intervalS = baseIntervalS * messageCount / baseMessageCount;
        [startRelS, endRelS, posDriftM, yawDriftDegRun, gtTravelRunM, ...
            odomTravelRunM, gtYawDeltaRunDeg, odomYawDeltaRunDeg] = ...
            computeOdomLocalMotionDrift(results(i), windowInfo, intervalS, config);
        if isempty(startRelS) || ...
                (~any(isfinite(posDriftM)) && ~any(isfinite(yawDriftDegRun)))
            continue;
        end

        if ~runAccepted
            plotRunCount = plotRunCount + 1;
            currentRunIndex = plotRunCount;
            runAccepted = true;
        end

        n = numel(startRelS);
        run = [run; repmat(string(results(i).bagName), n, 1)]; %#ok<AGROW>
        runIndex = [runIndex; repmat(currentRunIndex, n, 1)]; %#ok<AGROW>
        intervalMessageCount = [intervalMessageCount; repmat(messageCount, n, 1)]; %#ok<AGROW>
        intervalSColumn = [intervalSColumn; repmat(intervalS, n, 1)]; %#ok<AGROW>
        intervalStartRelDriveS = [intervalStartRelDriveS; startRelS]; %#ok<AGROW>
        intervalEndRelDriveS = [intervalEndRelDriveS; endRelS]; %#ok<AGROW>
        positionDriftM = [positionDriftM; posDriftM]; %#ok<AGROW>
        yawDriftDeg = [yawDriftDeg; yawDriftDegRun]; %#ok<AGROW>
        gtTravelM = [gtTravelM; gtTravelRunM]; %#ok<AGROW>
        odomTravelM = [odomTravelM; odomTravelRunM]; %#ok<AGROW>
        gtYawDeltaDeg = [gtYawDeltaDeg; gtYawDeltaRunDeg]; %#ok<AGROW>
        odomYawDeltaDeg = [odomYawDeltaDeg; odomYawDeltaRunDeg]; %#ok<AGROW>

        summaryRun(end + 1, 1) = string(results(i).bagName); %#ok<AGROW>
        summaryRunIndex(end + 1, 1) = currentRunIndex; %#ok<AGROW>
        summaryIntervalMessageCount(end + 1, 1) = messageCount; %#ok<AGROW>
        summaryIntervalS(end + 1, 1) = intervalS; %#ok<AGROW>
        summaryPositionSamples(end + 1, 1) = nnz(isfinite(posDriftM)); %#ok<AGROW>
        summaryYawSamples(end + 1, 1) = nnz(isfinite(yawDriftDegRun)); %#ok<AGROW>
        summaryPositionMeanM(end + 1, 1) = meanFinite(posDriftM); %#ok<AGROW>
        summaryPositionMedianM(end + 1, 1) = prctileFinite(posDriftM, 50); %#ok<AGROW>
        summaryPositionP95M(end + 1, 1) = prctileFinite(posDriftM, 95); %#ok<AGROW>
        summaryPositionMaxM(end + 1, 1) = maxFinite(posDriftM); %#ok<AGROW>
        summaryYawMeanDeg(end + 1, 1) = meanFinite(yawDriftDegRun); %#ok<AGROW>
        summaryYawMedianDeg(end + 1, 1) = prctileFinite(yawDriftDegRun, 50); %#ok<AGROW>
        summaryYawP95Deg(end + 1, 1) = prctileFinite(yawDriftDegRun, 95); %#ok<AGROW>
        summaryYawMaxDeg(end + 1, 1) = maxFinite(yawDriftDegRun); %#ok<AGROW>
    end
end

sampleTable = table(run, runIndex, intervalMessageCount, intervalSColumn, ...
    intervalStartRelDriveS, intervalEndRelDriveS, ...
    positionDriftM, yawDriftDeg, gtTravelM, odomTravelM, ...
    gtYawDeltaDeg, odomYawDeltaDeg, ...
    'VariableNames', {'run', 'run_index', 'interval_message_count', 'interval_s', ...
    'interval_start_rel_drive_s', ...
    'interval_end_rel_drive_s', 'position_drift_m', 'yaw_drift_deg', ...
    'gt_travel_m', 'odom_travel_m', 'gt_yaw_delta_deg', 'odom_yaw_delta_deg'});

summaryTable = table(summaryRun, summaryRunIndex, summaryIntervalMessageCount, summaryIntervalS, ...
    summaryPositionSamples, summaryYawSamples, ...
    summaryPositionMeanM, summaryPositionMedianM, summaryPositionP95M, summaryPositionMaxM, ...
    summaryYawMeanDeg, summaryYawMedianDeg, summaryYawP95Deg, summaryYawMaxDeg, ...
    'VariableNames', {'run', 'run_index', 'interval_message_count', 'interval_s', ...
    'position_sample_count', 'yaw_sample_count', ...
    'mean_position_drift_m', 'median_position_drift_m', ...
    'p95_position_drift_m', 'max_position_drift_m', ...
    'mean_yaw_drift_deg', 'median_yaw_drift_deg', ...
    'p95_yaw_drift_deg', 'max_yaw_drift_deg'});

if height(sampleTable) == 0 || ...
        (~any(isfinite(sampleTable.position_drift_m)) && ~any(isfinite(sampleTable.yaw_drift_deg)))
    warning('No finite local motion drift samples available; skipping local motion figure.');
    return;
end

fig = makePlotFigure('Odom Local Motion Drift by Message Interval', showPlot);
set(fig, 'Position', [100, 100, 2200, 900]);

ax1 = axes(fig, 'Position', [0.08, 0.14, 0.39, 0.76]);
plotLocalMotionBoxplot(ax1, sampleTable.position_drift_m * 100.0, ...
    sampleTable.interval_message_count, {}, 'position drift [cm]', ...
    'odometry-message interval');

ax2 = axes(fig, 'Position', [0.58, 0.14, 0.39, 0.76]);
plotLocalMotionBoxplot(ax2, sampleTable.yaw_drift_deg, ...
    sampleTable.interval_message_count, {}, 'yaw drift [deg]', ...
    'odometry-message interval');

savePlotFigure(fig, outputDir, 'Odom_Local_Motion_Drift_Boxplot', showPlot);

if ~exist(reportImageDir, 'dir')
    mkdir(reportImageDir);
end
copyfile(fullfile(outputDir, 'Odom_Local_Motion_Drift_Boxplot.png'), ...
    fullfile(reportImageDir, 'odom_local_motion_drift_boxplot.png'));
end

function [intervalStartRelDriveS, intervalEndRelDriveS, positionDriftM, ...
    yawDriftDeg, gtTravelM, odomTravelM, gtYawDeltaDeg, odomYawDeltaDeg] = ...
    computeOdomLocalMotionDrift(result, windowInfo, intervalS, config)
intervalStartRelDriveS = zeros(0, 1);
intervalEndRelDriveS = zeros(0, 1);
positionDriftM = zeros(0, 1);
yawDriftDeg = zeros(0, 1);
gtTravelM = zeros(0, 1);
odomTravelM = zeros(0, 1);
gtYawDeltaDeg = zeros(0, 1);
odomYawDeltaDeg = zeros(0, 1);

t = result.t(:);
if numel(t) < 2
    return;
end

startAbsS = max(windowInfo.firstDriveAbsS, min(t, [], 'omitnan'));
endAbsS = min(windowInfo.endAbsS, max(t, [], 'omitnan'));
if ~isfinite(startAbsS) || ~isfinite(endAbsS) || endAbsS - startAbsS < intervalS
    return;
end

intervalStartAbsS = (startAbsS:intervalS:(endAbsS - intervalS))';
intervalEndAbsS = intervalStartAbsS + intervalS;
if isempty(intervalStartAbsS)
    return;
end

positionRawValid = isfinite(t) & ...
    all(isfinite(result.gtPos(:, 1:2)), 2) & all(isfinite(result.ekfPos(:, 1:2)), 2);
if isfield(result, 'odomOptitrackPositionValidMask') && ...
        numel(result.odomOptitrackPositionValidMask) == numel(t)
    positionRawValid = positionRawValid & result.odomOptitrackPositionValidMask(:);
end

yawRawValid = isfinite(t) & isfinite(result.gtYaw(:)) & isfinite(result.ekfYaw(:));
if isfield(result, 'odomOptitrackYawValidMask') && ...
        numel(result.odomOptitrackYawValidMask) == numel(t)
    yawRawValid = yawRawValid & result.odomOptitrackYawValidMask(:);
end

maxSampleGapS = getConfigScalarValue(config, 'localMotionMaxSampleGapS', intervalS / 2);
positionIntervalValid = validateLocalMotionIntervals( ...
    t, intervalStartAbsS, intervalEndAbsS, positionRawValid, maxSampleGapS);
yawIntervalValid = validateLocalMotionIntervals( ...
    t, intervalStartAbsS, intervalEndAbsS, yawRawValid, maxSampleGapS);

positionDriftM = nan(numel(intervalStartAbsS), 1);
gtTravelM = nan(numel(intervalStartAbsS), 1);
odomTravelM = nan(numel(intervalStartAbsS), 1);
if nnz(positionRawValid) >= 2
    [tPos, posUniqueIdx] = unique(t(positionRawValid), 'stable');
    gtXYValid = result.gtPos(positionRawValid, 1:2);
    odomXYValid = result.ekfPos(positionRawValid, 1:2);
    gtXYValid = gtXYValid(posUniqueIdx, :);
    odomXYValid = odomXYValid(posUniqueIdx, :);

    gtXY0 = interp1(tPos, gtXYValid, intervalStartAbsS, 'linear', NaN);
    gtXY1 = interp1(tPos, gtXYValid, intervalEndAbsS, 'linear', NaN);
    odomXY0 = interp1(tPos, odomXYValid, intervalStartAbsS, 'linear', NaN);
    odomXY1 = interp1(tPos, odomXYValid, intervalEndAbsS, 'linear', NaN);

    gtDeltaXY = gtXY1 - gtXY0;
    odomDeltaXY = odomXY1 - odomXY0;
    deltaDriftXY = odomDeltaXY - gtDeltaXY;
    positionDriftM = hypot(deltaDriftXY(:, 1), deltaDriftXY(:, 2));
    gtTravelM = hypot(gtDeltaXY(:, 1), gtDeltaXY(:, 2));
    odomTravelM = hypot(odomDeltaXY(:, 1), odomDeltaXY(:, 2));
    positionDriftM(~positionIntervalValid) = NaN;
    gtTravelM(~positionIntervalValid) = NaN;
    odomTravelM(~positionIntervalValid) = NaN;
end

yawDriftDeg = nan(numel(intervalStartAbsS), 1);
gtYawDeltaDeg = nan(numel(intervalStartAbsS), 1);
odomYawDeltaDeg = nan(numel(intervalStartAbsS), 1);
if nnz(yawRawValid) >= 2
    [tYaw, yawUniqueIdx] = unique(t(yawRawValid), 'stable');
    gtYawValid = unwrap(result.gtYaw(yawRawValid));
    odomYawValid = unwrap(result.ekfYaw(yawRawValid));
    gtYawValid = gtYawValid(yawUniqueIdx);
    odomYawValid = odomYawValid(yawUniqueIdx);

    gtYaw0 = interp1(tYaw, gtYawValid, intervalStartAbsS, 'linear', NaN);
    gtYaw1 = interp1(tYaw, gtYawValid, intervalEndAbsS, 'linear', NaN);
    odomYaw0 = interp1(tYaw, odomYawValid, intervalStartAbsS, 'linear', NaN);
    odomYaw1 = interp1(tYaw, odomYawValid, intervalEndAbsS, 'linear', NaN);

    gtYawDelta = wrapAnglePiLocal(gtYaw1 - gtYaw0);
    odomYawDelta = wrapAnglePiLocal(odomYaw1 - odomYaw0);
    yawDriftDeg = rad2deg(abs(wrapAnglePiLocal(odomYawDelta - gtYawDelta)));
    gtYawDeltaDeg = rad2deg(gtYawDelta);
    odomYawDeltaDeg = rad2deg(odomYawDelta);
    yawDriftDeg(~yawIntervalValid) = NaN;
    gtYawDeltaDeg(~yawIntervalValid) = NaN;
    odomYawDeltaDeg(~yawIntervalValid) = NaN;
end

intervalStartRelDriveS = intervalStartAbsS - windowInfo.firstDriveAbsS;
intervalEndRelDriveS = intervalEndAbsS - windowInfo.firstDriveAbsS;
end

function ok = validateLocalMotionIntervals(t, intervalStartAbsS, intervalEndAbsS, rawValid, maxSampleGapS)
ok = false(numel(intervalStartAbsS), 1);
t = t(:);
rawValid = rawValid(:);
for i = 1:numel(intervalStartAbsS)
    inInterval = t >= intervalStartAbsS(i) & t <= intervalEndAbsS(i);
    if nnz(inInterval) < 2 || ~all(rawValid(inInterval))
        continue;
    end
    if isfinite(maxSampleGapS) && maxSampleGapS > 0 && ...
            any(diff(t(inInterval)) > maxSampleGapS)
        continue;
    end
    ok(i) = true;
end
end

function plotLocalMotionBoxplot(ax, values, groups, labels, yLabelText, xLabelText)
if nargin < 6 || isempty(xLabelText)
    xLabelText = 'run';
end
finite = isfinite(values) & isfinite(groups);
values = values(finite);
groups = groups(finite);
if isempty(values)
    text(ax, 0.5, 0.5, 'No finite samples', ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 20);
    axis(ax, 'off');
    return;
end

presentGroups = unique(groups, 'stable');
groupIndex = nan(size(groups));
for i = 1:numel(presentGroups)
    groupIndex(groups == presentGroups(i)) = i;
end
if isempty(labels)
    presentLabels = arrayfun(@(group) sprintf('%g', group), presentGroups, ...
        'UniformOutput', false);
elseif numel(labels) >= max(presentGroups) && all(presentGroups == round(presentGroups))
    presentLabels = labels(presentGroups);
elseif numel(labels) == numel(presentGroups)
    presentLabels = labels;
else
    presentLabels = arrayfun(@(group) sprintf('%g', group), presentGroups, ...
        'UniformOutput', false);
end

axes(ax);
boxplot(values, groupIndex, 'Labels', presentLabels, 'Symbol', 'k+', ...
    'Whisker', 1.5, 'Widths', 0.55, 'Colors', [0.10, 0.10, 0.10]);
grid(ax, 'on');
set(ax, 'LineWidth', 1.7, 'FontSize', 22, ...
    'GridAlpha', 0.22, 'MinorGridAlpha', 0.12);
xlabel(ax, xLabelText, 'FontSize', 26);
ylabel(ax, yLabelText, 'FontSize', 26);
ylimCurrent = ylim(ax);
if ylimCurrent(1) < 0
    ylim(ax, [0, ylimCurrent(2)]);
end

boxes = findobj(ax, 'Tag', 'Box');
medians = findobj(ax, 'Tag', 'Median');
whiskers = findobj(ax, 'Tag', 'Whisker');
set([boxes; medians; whiskers], 'LineWidth', 1.4);
end

function value = meanFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = prctileFinite(values, percentileValue)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = prctile(values, percentileValue);
end
end

function value = maxFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = getResultScalar(result, fieldName, defaultValue)
value = defaultValue;
if isfield(result, fieldName)
    candidate = result.(fieldName);
    if isnumeric(candidate) && isscalar(candidate)
        value = double(candidate);
    end
end
end

function holdIdx = detectYawCrashHoldIndex(tAbsGrid, yawInterpDeg, windowInfo, config)
holdIdx = [];
if ~isfield(config, 'crashYawDropDetectionEnabled') || ~config.crashYawDropDetectionEnabled
    return;
end

inWindow = tAbsGrid >= windowInfo.startAbsS & tAbsGrid <= windowInfo.endAbsS & ...
    isfinite(yawInterpDeg);
gridIdx = find(inWindow);
if numel(gridIdx) < 3
    return;
end

yawWindowDeg = yawInterpDeg(gridIdx);
yawDeltaDeg = diff(yawWindowDeg);
dropIdx = find(yawDeltaDeg <= -config.crashYawDropThresholdDeg & ...
    yawWindowDeg(1:end-1) >= config.crashYawDropMinBeforeDeg, 1, 'first');
if isempty(dropIdx)
    return;
end

holdIdx = gridIdx(dropIdx);
end

function info = emptyDriveWindowInfo()
info = struct( ...
    'ok', false, ...
    'topic', "", ...
    'firstDriveAbsS', NaN, ...
    'startAbsS', NaN, ...
    'endAbsS', NaN);
end

function info = findDriveAlignedWindow(result, config)
info = emptyDriveWindowInfo();
if ~isfield(result, 'bagPath') || isempty(result.bagPath) || ~isfolder(result.bagPath)
    warning('No bag path available for %s; skipping drive-aligned window.', result.bagName);
    return;
end

[pythonOk, firstDriveAbsS, endAbsS, driveTopic] = readDriveWindowWithPythonLocal(result.bagPath, config);
if pythonOk
    startAbsS = firstDriveAbsS - config.driveWindowPreStartS;
    if isfield(result, 't') && ~isempty(result.t)
        startAbsS = max(startAbsS, min(result.t(:), [], 'omitnan'));
        endAbsS = min(endAbsS, max(result.t(:), [], 'omitnan'));
    end
    if isfinite(startAbsS) && isfinite(endAbsS) && endAbsS > startAbsS
        info.ok = true;
        info.topic = string(driveTopic);
        info.firstDriveAbsS = firstDriveAbsS;
        info.startAbsS = startAbsS;
        info.endAbsS = endAbsS;
        return;
    end
end

try
    bag = ros2bagreader(result.bagPath);
catch ME
    warning('Could not open bag for drive window in %s: %s', result.bagName, ME.message);
    return;
end

topics = getTopicNamesLocal(bag.AvailableTopics);
driveTopic = resolveDriveTopicLocal(topics, config.driveCommandTopics);
if isempty(driveTopic)
    warning('No drive command topic found in %s.', result.bagName);
    return;
end

try
    driveSel = select(bag, 'Topic', driveTopic);
    driveMsgs = readMessages(driveSel);
catch ME
    warning('Could not read %s in %s: %s', driveTopic, result.bagName, ME.message);
    return;
end

if isempty(driveMsgs)
    warning('No drive command messages found in %s.', result.bagName);
    return;
end

tOverride = getSelectionTimesLocal(driveSel, numel(driveMsgs));
[tDrive, speed, acceleration] = extractDriveCommandSeriesLocal(driveMsgs, tOverride);
active = (isfinite(speed) & abs(speed) > config.driveCommandSpeedThresholdMps) | ...
    (isfinite(acceleration) & abs(acceleration) > config.driveCommandAccelerationThreshold);
idxFirst = find(active & isfinite(tDrive), 1, 'first');
idxLast = find(active & isfinite(tDrive), 1, 'last');
if isempty(idxFirst) || isempty(idxLast)
    warning('No active drive command found in %s.', result.bagName);
    return;
end

firstDriveAbsS = tDrive(idxFirst);
startAbsS = firstDriveAbsS - config.driveWindowPreStartS;
endAbsS = tDrive(idxLast);
if isfield(result, 't') && ~isempty(result.t)
    startAbsS = max(startAbsS, min(result.t(:), [], 'omitnan'));
    endAbsS = min(endAbsS, max(result.t(:), [], 'omitnan'));
end
if ~isfinite(startAbsS) || ~isfinite(endAbsS) || endAbsS <= startAbsS
    warning('Invalid drive-aligned window in %s.', result.bagName);
    return;
end

info.ok = true;
info.topic = string(driveTopic);
info.firstDriveAbsS = firstDriveAbsS;
info.startAbsS = startAbsS;
info.endAbsS = endAbsS;
end

function [ok, firstDriveAbsS, lastDriveAbsS, driveTopic] = readDriveWindowWithPythonLocal(bagPath, config)
ok = false;
firstDriveAbsS = NaN;
lastDriveAbsS = NaN;
driveTopic = "";

if isfield(config, 'driveWindowHelperScript') && ~isempty(config.driveWindowHelperScript)
    scriptPath = config.driveWindowHelperScript;
else
    scriptPath = fullfile(fileparts(mfilename('fullpath')), 'read_drive_window.py');
end
if ~isfile(scriptPath)
    return;
end

cmd = sprintf('env LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python3 %s %s %.17g %.17g', ...
    shellQuoteLocal(scriptPath), shellQuoteLocal(bagPath), ...
    config.driveCommandSpeedThresholdMps, ...
    config.driveCommandAccelerationThreshold);
[status, output] = system(cmd);
if status ~= 0 || strlength(strtrim(string(output))) == 0
    return;
end

lines = splitlines(strtrim(string(output)));
jsonLine = "";
for i = numel(lines):-1:1
    if startsWith(strtrim(lines(i)), "{")
        jsonLine = strtrim(lines(i));
        break;
    end
end
if strlength(jsonLine) == 0
    return;
end

try
    decoded = jsondecode(char(jsonLine));
catch
    return;
end
if ~isfield(decoded, 'ok') || ~decoded.ok || ~isfield(decoded, 'first') || ~isfield(decoded, 'last')
    return;
end

firstDriveAbsS = double(decoded.first);
lastDriveAbsS = double(decoded.last);
if isfield(decoded, 'topic')
    driveTopic = string(decoded.topic);
end
ok = isfinite(firstDriveAbsS) && isfinite(lastDriveAbsS) && lastDriveAbsS > firstDriveAbsS;
end

function quoted = shellQuoteLocal(textValue)
textValue = char(string(textValue));
quoted = ['''', strrep(textValue, '''', '''"''"'''), ''''];
end

function driveTopic = resolveDriveTopicLocal(topics, candidates)
driveTopic = '';
for i = 1:numel(candidates)
    candidate = candidates{i};
    if any(strcmp(topics, candidate))
        driveTopic = candidate;
        return;
    end
end
idx = find(contains(topics, 'drive') | contains(topics, 'ackermann'), 1, 'first');
if ~isempty(idx)
    driveTopic = topics{idx};
end
end

function topics = getTopicNamesLocal(availableTopics)
if istable(availableTopics)
    names = availableTopics.Properties.RowNames;
    if isempty(names) && ismember('TopicName', availableTopics.Properties.VariableNames)
        names = availableTopics.TopicName;
    end
elseif isstruct(availableTopics) && isfield(availableTopics, 'Name')
    names = {availableTopics.Name};
elseif iscell(availableTopics)
    names = availableTopics;
else
    names = {};
end
topics = cellstr(string(names));
end

function t = getSelectionTimesLocal(selection, n)
t = nan(n, 1);
try
    raw = selection.MessageList.Time;
    if isduration(raw)
        raw = seconds(raw);
    elseif isdatetime(raw)
        raw = posixtime(raw);
    else
        raw = double(raw);
    end
    raw = raw(:);
    count = min(n, numel(raw));
    t(1:count) = raw(1:count);
catch
end
end

function [t, speed, acceleration] = extractDriveCommandSeriesLocal(msgs, tOverride)
if nargin < 2
    tOverride = [];
end
n = numel(msgs);
t = nan(n, 1);
speed = nan(n, 1);
acceleration = nan(n, 1);

for k = 1:n
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end
    t(k) = extractHeaderTimeLocal(m);
    if ~isfinite(t(k)) && numel(tOverride) == n && isfinite(tOverride(k))
        t(k) = tOverride(k);
    end

    command = m;
    [drive, hasDrive] = getFieldIgnoreCaseLocal(m, 'drive');
    if hasDrive && isstruct(drive)
        command = drive;
    end

    [speedValue, hasSpeed] = getNumericFieldIgnoreCaseLocal(command, 'speed');
    if hasSpeed
        speed(k) = double(speedValue);
    end
    [accelValue, hasAccel] = getNumericFieldIgnoreCaseLocal(command, 'acceleration');
    if hasAccel
        acceleration(k) = double(accelValue);
    end
end

valid = isfinite(t);
t = t(valid);
speed = speed(valid);
acceleration = acceleration(valid);
[t, order] = sort(t);
speed = speed(order);
acceleration = acceleration(order);
[t, uniqueIdx] = unique(t, 'stable');
speed = speed(uniqueIdx);
acceleration = acceleration(uniqueIdx);
end

function t = extractHeaderTimeLocal(m)
t = NaN;
[header, hasHeader] = getFieldIgnoreCaseLocal(m, 'header');
if ~hasHeader || ~isstruct(header)
    return;
end
[stamp, hasStamp] = getFieldIgnoreCaseLocal(header, 'stamp');
if ~hasStamp || ~isstruct(stamp)
    return;
end
[secVal, hasSec] = getNumericFieldIgnoreCaseLocal(stamp, 'sec');
[nsecVal, hasNSec] = getNumericFieldIgnoreCaseLocal(stamp, 'nanosec');
if ~hasNSec
    [nsecVal, hasNSec] = getNumericFieldIgnoreCaseLocal(stamp, 'nsec');
end
if hasSec && hasNSec
    t = double(secVal) + double(nsecVal) * 1e-9;
elseif hasSec
    t = double(secVal);
end
end

function [value, hasField] = getFieldIgnoreCaseLocal(s, fieldName)
value = [];
hasField = false;
if ~isstruct(s)
    return;
end
fields = fieldnames(s);
idx = find(strcmpi(fields, fieldName), 1, 'first');
if isempty(idx)
    return;
end
value = s.(fields{idx});
hasField = true;
end

function [value, hasValue] = getNumericFieldIgnoreCaseLocal(s, fieldName)
value = NaN;
hasValue = false;
[rawValue, hasField] = getFieldIgnoreCaseLocal(s, fieldName);
if hasField && isnumeric(rawValue) && isscalar(rawValue)
    value = double(rawValue);
    hasValue = isfinite(value);
end
end

function a = wrapAnglePiLocal(a)
a = atan2(sin(a), cos(a));
end
