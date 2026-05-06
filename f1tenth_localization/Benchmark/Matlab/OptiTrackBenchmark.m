clc;
close all;
% Main driver for OptiTrack-vs-EKF benchmark bags.
%
% Run this file directly from MATLAB. All settings are edited below; there
% are no command-line inputs or launch arguments.

% ===================== USER SETTINGS =====================
skipStartupAndIncompleteLaps = true;
showPlots = false   ;

matlabRootDir = '/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/Matlab';
bagRootDir = fullfile(matlabRootDir, '..', 'bags', 'OptitrackBags', 'AMCL');
odomBagRootDir = fullfile(matlabRootDir, '..', 'bags', 'OptitrackBags', 'ODOM');
odomCsvRootDir = fullfile(matlabRootDir, 'csv', 'Odom');
plotsRootDir = fullfile(matlabRootDir, 'plots', 'OptiTrackBenchmark');

config = struct();
config.ekfTopic = '/ekf_pose';
config.optitrackTopic = '/vrpn_mocap/Car2/pose';
config.staticTfTopic = '/tf_static';
config.mapTopic = '/map';
config.mapFrame = 'map';
config.optitrackFrame = 'world';
config.skipStartupAndIncompleteLaps = skipStartupAndIncompleteLaps;
config.optitrackExpectedHz = 240;
config.ekfExpectedHz = 200;

% Lap detection settings. Tune only if lap trimming misses crossings.
config.lapCloseRadiusM = 0.45;
config.lapRearmRadiusM = 0.90;
config.minLapDurationS = 2.0;
config.minLapDistanceM = 2.0;

% OptiTrack dropout/outlier filtering.
config.optitrackDropoutZeroRadiusM = 1e-6;
config.optitrackFreezeDistanceM = 1e-6;
config.optitrackMaxSpeedMps = 12.0;

% Drop known bad yaw-outlier laps from selected particle-count runs.
config.excludeYawOutlierLaps = true;
config.yawOutlierLapThresholdRad = pi / 2;
config.yawOutlierLapParticleCounts = 100;

% Ignore isolated 1-2 sample yaw spikes in yaw metrics.
config.yawIsolatedOutlierFilterEnabled = true;
config.yawIsolatedOutlierThresholdRad = pi / 2;
config.yawIsolatedOutlierMaxRunLength = 2;

% Exclude known bad OptiTrack area from error metrics. Excluded samples are
% still drawn grey in trajectory/heatmap plots.
config.metricExcludeXLessThanM = 2.0;
% =================== END USER SETTINGS ===================

plotFunctionsDir = fullfile(matlabRootDir, 'matlab plotting functions');
if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    error('Could not find plotting functions directory: %s', plotFunctionsDir);
end

if ~isfolder(bagRootDir)
    error('Bag root does not exist: %s', bagRootDir);
end

fprintf('Bag root        : %s\n', bagRootDir);
fprintf('Skip lap trim   : %s\n', mat2str(skipStartupAndIncompleteLaps));
fprintf('EKF topic       : %s\n', config.ekfTopic);
fprintf('OptiTrack topic : %s\n', config.optitrackTopic);
fprintf('Expected rates  : OptiTrack %.0f Hz | EKF %.0f Hz\n', ...
    config.optitrackExpectedHz, config.ekfExpectedHz);

results = loadOptiTrackBenchmarkBags(bagRootDir, config);
if isempty(results)
    error('No valid OptiTrack benchmark bags were loaded from %s', bagRootDir);
end

% ===================== ODOM LOADING =====================
odomConfig = config;
odomConfig.ekfTopic = '/odom_pose';
odomConfig.skipStartupAndIncompleteLaps = false;
odomConfig.bagStartEpochSeconds = readOdomCsvStartEpochSeconds(odomCsvRootDir);
odomConfig.bagStartEpochToleranceS = 2.0;
fprintf('\nOdom CSV folder : %s\n', odomCsvRootDir);
fprintf('Odom bag root   : %s\n', odomBagRootDir);
fprintf('Odom topic      : %s\n', odomConfig.ekfTopic);
fprintf('Odom CSV runs   : %d\n', numel(odomConfig.bagStartEpochSeconds));

odomResults = [];
if isfolder(odomBagRootDir)
    odomResults = loadOptiTrackBenchmarkBags(odomBagRootDir, odomConfig);
    odomResults = alignOdomResultsToGroundTruthStart(odomResults);
    if isempty(odomResults)
        warning('No valid ODOM benchmark bags were loaded from %s', odomBagRootDir);
    end
else
    warning('ODOM bag root does not exist: %s', odomBagRootDir);
end

[~, runName] = fileparts(bagRootDir);
if config.skipStartupAndIncompleteLaps
    runName = [runName, '_trimmed'];
else
    runName = [runName, '_full'];
end
[runName, outputDir] = prepareOutputDirectory(runName, plotsRootDir);
fprintf('Output plot folder: %s\n', outputDir);

odomOutputDir = '';
if ~isempty(odomResults)
    [~, odomRunName] = fileparts(odomBagRootDir);
    odomRunName = [odomRunName, '_full_initial_aligned'];
    [~, odomOutputDir] = prepareOutputDirectory(odomRunName, plotsRootDir);
    fprintf('ODOM output plot folder: %s\n', odomOutputDir);
end

plotOptiTrackTrajectoryComparison(results, outputDir, showPlots);
plotOptiTrackXYErrorScatter(results, outputDir, showPlots);
plotOptiTrackPositionErrorBins(results, outputDir, showPlots);
plotOptiTrackBenchmarkStatistics(results, outputDir, showPlots);
plotOptiTrackErrorHeatmaps(results, outputDir, showPlots);

if ~isempty(odomResults)
    plotOdomTrajectoryComparison(odomResults, odomOutputDir, showPlots);
    plotOdomPositionErrorOverTime(odomResults, odomOutputDir, showPlots);
end

summaryTable = printOptiTrackBenchmarkSummary(results);
summaryPath = fullfile(outputDir, 'OptiTrackBenchmark_Summary.csv');
writetable(summaryTable, summaryPath);

lapTimeTable = printOptiTrackLapTimeSummary(results);
lapTimePath = fullfile(outputDir, 'OptiTrack_LapTime_Summary.csv');
writetable(lapTimeTable, lapTimePath);

fprintf('\nSummary CSV saved to %s\n', summaryPath);
fprintf('Lap-time CSV saved to %s\n', lapTimePath);
fprintf('Plots saved to %s\n', outputDir);
if ~isempty(odomOutputDir)
    fprintf('ODOM plots saved to %s\n', odomOutputDir);
end

function results = alignOdomResultsToGroundTruthStart(results)
%ALIGNODOMRESULTSTOGROUNDTRUTHSTART Remove initial frame offset from odom.
%
% The pure odom pose is relative to its odom frame. Aligning the first odom
% sample to OptiTrack makes the plotted error show drift over time instead
% of a constant start-frame offset.

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

if isfield(result, 'laps') && ~isempty(result.laps)
    for j = 1:numel(result.laps)
        result.laps(j).ekfPos(:, 1:2) = ((R * (result.laps(j).ekfPos(:, 1:2) - odom0)')' + gt0);
        if isfield(result.laps(j), 'ekfYaw') && ~isempty(result.laps(j).ekfYaw)
            result.laps(j).ekfYaw = wrapAnglePiLocal(result.laps(j).ekfYaw + yawOffset);
        end
        result.laps(j) = recomputePositionErrors(result.laps(j));
    end
end
end

function result = recomputePositionErrors(result)
result.xError = result.ekfPos(:, 1) - result.gtPos(:, 1);
result.yError = result.ekfPos(:, 2) - result.gtPos(:, 2);
result.xyError = hypot(result.xError, result.yError);
end

function plotOdomTrajectoryComparison(results, outputDir, showPlot)
fig = makePlotFigure('OptiTrack vs Odom Trajectory', showPlot);
hold on;
colors = lines(max(numel(results), 1));

drawOccupancyMapBackground(getFirstMapData(results));

for i = 1:numel(results)
    c = colors(i, :);
    plot(results(i).gtPos(:, 1), results(i).gtPos(:, 2), '-', ...
        'Color', c, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('%s OptiTrack', results(i).bagName));
    plot(results(i).ekfPos(:, 1), results(i).ekfPos(:, 2), '--', ...
        'Color', c, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('%s Odom', results(i).bagName));
end

axis equal;
grid on;
xlabel('map x [m]');
ylabel('map y [m]');
title('OptiTrack Ground Truth vs Initial-Aligned Odom Trajectory');
legend('Location', 'bestoutside', 'Interpreter', 'none');
savePlotFigure(fig, outputDir, 'Odom_Trajectory_vs_OptiTrack', showPlot);
end

function plotOdomPositionErrorOverTime(results, outputDir, showPlot)
fig = makePlotFigure('Odom Position Error Over Time', showPlot);
hold on;
colors = lines(max(numel(results), 1));

for i = 1:numel(results)
    plot(results(i).tRel, results(i).xyError, 'Color', colors(i, :), ...
        'LineWidth', 1.1, 'DisplayName', results(i).bagName);
end

grid on;
xlabel('time after analysis window start [s]');
ylabel('position error [m]');
title('Initial-Aligned Odom Position Error Over Time');
legend('Location', 'bestoutside', 'Interpreter', 'none');
savePlotFigure(fig, outputDir, 'Odom_Position_Error_Over_Time', showPlot);
end

function a = wrapAnglePiLocal(a)
a = atan2(sin(a), cos(a));
end

function epochSeconds = readOdomCsvStartEpochSeconds(odomCsvRootDir)
if ~isfolder(odomCsvRootDir)
    warning('ODOM CSV root does not exist: %s', odomCsvRootDir);
    epochSeconds = [];
    return;
end

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
