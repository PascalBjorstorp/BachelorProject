clc;
close all;

% Plot OptiTrack self-consistency failures from AMCL and ODOM benchmark bags.
% The figure is used to show why OptiTrack was not used as the main AMCL
% real-world ground-truth source.

matlabRootDir = '/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/Matlab';
plotFunctionsDir = fullfile(matlabRootDir, 'matlab plotting functions');
if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    error('Could not find plotting functions directory: %s', plotFunctionsDir);
end

amclBagRootDir = fullfile(matlabRootDir, '..', 'bags', 'OptitrackBags', 'AMCL');
odomBagRootDir = fullfile(matlabRootDir, '..', 'bags', 'OptitrackBags', 'ODOM');
outputDir = fullfile(matlabRootDir, 'plots', 'OptiTrackFailureMap');
reportImageDir = '/home/pascal/Documents/BachelorProject/Report/Sections/Localization/Images';
showPlot = false;

config = struct();
config.ekfTopic = '/ego_racecar/odom';
config.optitrackTopic = '/vrpn_mocap/Car2/pose';
config.staticTfTopic = '/tf_static';
config.mapTopic = '/map';
config.mapFrame = 'map';
config.optitrackFrame = 'world';
config.skipStartupAndIncompleteLaps = false;
config.lapCloseRadiusM = 0.45;
config.lapRearmRadiusM = 0.90;
config.minLapDurationS = 2.0;
config.minLapDistanceM = 2.0;
config.optitrackDropoutZeroRadiusM = 1e-6;
config.optitrackFreezeDistanceM = -1;
config.optitrackMaxSpeedMps = inf;
config.startCalibrationEnabled = false;
config.optitrackFreezeZoneExcludeEnabled = false;
config.excludeYawOutlierLaps = false;
config.yawIsolatedOutlierFilterEnabled = false;
config.metricExcludeXLessThanM = -Inf;

% Plausibility thresholds. The yaw-rate limit is the expected physical limit
% of the car; the other thresholds suppress small marker jitter.
quality = struct();
quality.maxYawRateRadps = 3.0;
quality.yawRateWindowS = 0.10;
quality.yawJumpThresholdRad = deg2rad(45.0);
quality.stallWindowS = 0.05;
quality.stallMaxOptiTravelM = 0.02;
quality.stallMinOdomTravelM = 0.06;
quality.motionWindowS = 0.20;
quality.minMotionTravelM = 0.08;
quality.maxBackwardM = -0.04;
quality.minSidewaysM = 0.06;
quality.sidewaysRatio = 1.2;
quality.jumpWindowS = 0.05;
quality.jumpMinOptiTravelM = 0.40;
quality.ignoreBeforeMotion = true;
quality.motionStartTravelM = 0.05;

fprintf('Loading AMCL OptiTrack bags from %s\n', amclBagRootDir);
amclResults = loadOptiTrackBenchmarkBags(amclBagRootDir, config);
fprintf('Loading ODOM OptiTrack bags from %s\n', odomBagRootDir);
odomResults = loadOptiTrackBenchmarkBags(odomBagRootDir, config);

allResults = [tagResultGroup(amclResults, "AMCL"), tagResultGroup(odomResults, "ODOM")];
if isempty(allResults)
    error('No OptiTrack bags were loaded.');
end

allIssues = emptyIssueTable();
summaryRows = emptySummaryTable();
for i = 1:numel(allResults)
    [issues, summary] = analyzeOptiTrackFailureResult(allResults(i), quality);
    allIssues = [allIssues; issues]; %#ok<AGROW>
    summaryRows = [summaryRows; summary]; %#ok<AGROW>
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
if ~exist(reportImageDir, 'dir')
    mkdir(reportImageDir);
end

writetable(summaryRows, fullfile(outputDir, 'OptiTrack_Failure_Map_Summary.csv'));
writetable(allIssues, fullfile(outputDir, 'OptiTrack_Failure_Map_Samples.csv'));

renderOptiTrackFailureMap(allResults, allIssues, outputDir, showPlot);
copyfile(fullfile(outputDir, 'OptiTrack_Failure_Map.png'), ...
    fullfile(reportImageDir, 'optitrack_failure_map.png'));

fprintf('Failure map saved to %s\n', fullfile(reportImageDir, 'optitrack_failure_map.png'));
fprintf('Summary saved to %s\n', fullfile(outputDir, 'OptiTrack_Failure_Map_Summary.csv'));

function results = tagResultGroup(results, groupName)
for i = 1:numel(results)
    results(i).resultGroup = groupName;
end
end

function [issues, summary] = analyzeOptiTrackFailureResult(result, quality)
t = result.t(:);
xy = result.gtPos(:, 1:2);
yaw = result.gtYaw(:);
odomXY = result.ekfPos(:, 1:2);

n = numel(t);
yawRateMask = false(n, 1);
yawJumpMask = false(n, 1);
stallMask = false(n, 1);
nonholonomicMask = false(n, 1);
jumpMask = false(n, 1);

if n >= 2
    yawRateMask = yawRateWindowMask(t, yaw, quality);
    dyaw = wrapAnglePiLocal(diff(yaw));
    badYawJumpTarget = find(abs(dyaw) > quality.yawJumpThresholdRad) + 1;
    yawJumpMask(badYawJumpTarget) = true;

    stallMask = slidingWindowTravelMask(t, xy, odomXY, ...
        quality.stallWindowS, quality.stallMaxOptiTravelM, quality.stallMinOdomTravelM);
    nonholonomicMask = nonholonomicMotionMask(t, xy, yaw, quality);
    jumpMask = positionJumpMask(t, xy, quality);
end

motionMask = analysisMotionMask(t, odomXY, quality);
yawRateMask = yawRateMask & motionMask;
yawJumpMask = yawJumpMask & motionMask;
stallMask = stallMask & motionMask;
nonholonomicMask = nonholonomicMask & motionMask;
jumpMask = jumpMask & motionMask;

categories = [ ...
    makeIssueRows(result, stallMask, "OptiTrack stall"), ...
    makeIssueRows(result, yawRateMask, "Yaw rate > 3 rad/s"), ...
    makeIssueRows(result, yawJumpMask, "Sudden yaw jump"), ...
    makeIssueRows(result, nonholonomicMask, "Sideways/backwards motion"), ...
    makeIssueRows(result, jumpMask, "Position jump")];
categories = categories(:);

if isempty(categories)
    issues = emptyIssueTable();
else
    issues = struct2table(categories);
end

summary = table(string(result.resultGroup), string(result.bagName), n, ...
    nnz(stallMask), nnz(yawRateMask), nnz(yawJumpMask), ...
    nnz(nonholonomicMask), nnz(jumpMask), ...
    'VariableNames', {'group', 'bag', 'samples', 'stall_samples', ...
    'yaw_rate_samples', 'yaw_jump_samples', 'nonholonomic_samples', ...
    'position_jump_samples'});
end

function mask = yawRateWindowMask(t, yaw, quality)
n = numel(t);
mask = false(n, 1);
j = 1;
for i = 2:n
    while j < i && t(i) - t(j) > quality.yawRateWindowS
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
    if yawRate > quality.maxYawRateRadps
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

function mask = nonholonomicMotionMask(t, xy, yaw, quality)
n = numel(t);
mask = false(n, 1);
j = 1;
for i = 2:n
    while j < i && t(i) - t(j) > quality.motionWindowS
        j = j + 1;
    end
    if j >= i || ~all(isfinite(xy([j i], :)), 'all') || ~isfinite(yaw(i))
        continue;
    end

    dxy = xy(i, :) - xy(j, :);
    travel = norm(dxy);
    if travel < quality.minMotionTravelM
        continue;
    end

    forward = [cos(yaw(i)), sin(yaw(i))];
    longitudinal = dot(dxy, forward);
    lateral = dxy(1) * (-forward(2)) + dxy(2) * forward(1);
    sideways = abs(lateral) >= quality.minSidewaysM && ...
        abs(lateral) >= quality.sidewaysRatio * max(abs(longitudinal), eps);
    backwards = longitudinal <= quality.maxBackwardM;
    if sideways || backwards
        mask(i) = true;
    end
end
end

function mask = positionJumpMask(t, optiXY, quality)
n = numel(t);
mask = false(n, 1);
j = 1;
for i = 2:n
    while j < i && t(i) - t(j) > quality.jumpWindowS
        j = j + 1;
    end
    if j >= i || ~all(isfinite(optiXY([j i], :)), 'all')
        continue;
    end

    optiTravel = norm(optiXY(i, :) - optiXY(j, :));
    if optiTravel >= quality.jumpMinOptiTravelM
        mask(j:i) = true;
    end
end
end

function mask = analysisMotionMask(t, odomXY, quality)
n = numel(t);
mask = true(n, 1);
if ~isfield(quality, 'ignoreBeforeMotion') || ~quality.ignoreBeforeMotion || n < 2
    return;
end

valid = isfinite(t) & all(isfinite(odomXY), 2);
if ~any(valid)
    return;
end

idx0 = find(valid, 1, 'first');
travelThresholdM = quality.motionStartTravelM;
dxyFromStart = odomXY - odomXY(idx0, :);
travelFromStart = hypot(dxyFromStart(:, 1), dxyFromStart(:, 2));
idxStart = find(valid & travelFromStart >= travelThresholdM, 1, 'first');
if isempty(idxStart)
    mask(:) = false;
    return;
end

mask = t >= t(idxStart);
end

function rows = makeIssueRows(result, mask, category)
idx = find(mask(:));
rows = defaultIssueRow();
rows(:) = [];
for k = 1:numel(idx)
    i = idx(k);
    row = defaultIssueRow();
    row.group = string(result.resultGroup);
    row.bag = string(result.bagName);
    row.category = string(category);
    row.t_rel_s = result.tRel(i);
    row.x_m = result.gtPos(i, 1);
    row.y_m = result.gtPos(i, 2);
    row.yaw_deg = rad2deg(result.gtYaw(i));
    rows(end + 1) = row; %#ok<AGROW>
end
end

function renderOptiTrackFailureMap(results, issues, outputDir, showPlot)
fig = makePlotFigure('OptiTrack failure map', showPlot);
set(fig, 'Position', [100, 100, 1800, 1200]);
ax = axes(fig);
hold(ax, 'on');
drawOccupancyMapBackground(getFirstMapData(results));

for i = 1:numel(results)
    stride = max(floor(numel(results(i).t) / 1800), 1);
    idx = 1:stride:numel(results(i).t);
    plot(ax, results(i).gtPos(idx, 1), results(i).gtPos(idx, 2), '.', ...
        'Color', [0.70 0.70 0.70], 'MarkerSize', 4, 'HandleVisibility', 'off');
end

plotCategory(ax, issues, "OptiTrack stall", [0.12 0.43 0.84], 'o');
plotCategory(ax, issues, "Yaw rate > 3 rad/s", [0.82 0.13 0.13], '^');
plotCategory(ax, issues, "Sudden yaw jump", [0.95 0.45 0.10], 'v');
plotCategory(ax, issues, "Sideways/backwards motion", [0.47 0.18 0.70], 's');
plotCategory(ax, issues, "Position jump", [0.10 0.55 0.22], 'd');

axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'LineWidth', 1.8, 'FontSize', 22, ...
    'GridAlpha', 0.20, 'MinorGridAlpha', 0.12, 'Layer', 'top');
xlabel(ax, 'map x [m]');
ylabel(ax, 'map y [m]');
set(ax, 'Position', [0.08, 0.09, 0.88, 0.84]);
lgd = legend(ax, 'Location', 'northeast', ...
    'LineWidth', 1.1, 'FontSize', 20);
lgd.ItemTokenSize = [20, 17];
savePlotFigure(fig, outputDir, 'OptiTrack_Failure_Map', showPlot);
end

function plotCategory(ax, issues, category, color, marker)
if isempty(issues) || height(issues) == 0
    return;
end
mask = issues.category == category;
if ~any(mask)
    return;
end

x = issues.x_m(mask);
y = issues.y_m(mask);
maxPoints = 2500;
if numel(x) > maxPoints
    idx = round(linspace(1, numel(x), maxPoints));
    x = x(idx);
    y = y(idx);
end
scatter(ax, x, y, 64, marker, 'filled', ...
    'MarkerFaceColor', color, 'MarkerEdgeColor', 'w', ...
    'LineWidth', 0.6, 'DisplayName', sprintf('%s (%d)', category, nnz(mask)));
end

function tableOut = emptyIssueTable()
tableOut = table(strings(0, 1), strings(0, 1), strings(0, 1), ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    'VariableNames', {'group', 'bag', 'category', 't_rel_s', ...
    'x_m', 'y_m', 'yaw_deg'});
end

function tableOut = emptySummaryTable()
tableOut = table(strings(0, 1), strings(0, 1), zeros(0, 1), ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    'VariableNames', {'group', 'bag', 'samples', 'stall_samples', ...
    'yaw_rate_samples', 'yaw_jump_samples', 'nonholonomic_samples', ...
    'position_jump_samples'});
end

function row = defaultIssueRow()
row = struct('group', "", 'bag', "", 'category', "", ...
    't_rel_s', NaN, 'x_m', NaN, 'y_m', NaN, 'yaw_deg', NaN);
end

function a = wrapAnglePiLocal(a)
a = atan2(sin(a), cos(a));
end
