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

fprintf('ODOM bag root : %s\n', odomBagRootDir);
fprintf('ODOM CSV runs : %d\n', numel(config.bagStartEpochSeconds));

results = loadOptiTrackBenchmarkBags(odomBagRootDir, config);
results = alignOdomResultsToGroundTruthStart(results);
if isempty(results)
    error('No ODOM benchmark bags were loaded from %s', odomBagRootDir);
end

[trajectorySummaryTable] = plotOdomTrajectoryComparisonForReport( ...
    results, outputDir, reportImageDir, showPlot);
[summaryTable, aggregateTable] = plotOdomMeanVariance(results, outputDir, reportImageDir, showPlot);
writetable(trajectorySummaryTable, fullfile(outputDir, 'Odom_Trajectory_Plot_Summary.csv'));
writetable(summaryTable, fullfile(outputDir, 'Odom_Run_Error_Summary.csv'));
writetable(aggregateTable, fullfile(outputDir, 'Odom_Mean_Variance_Summary.csv'));

fprintf('Report image saved to %s\n', ...
    fullfile(reportImageDir, 'odom_position_error_mean_std.png'));

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

function summaryTable = plotOdomTrajectoryComparisonForReport(results, outputDir, reportImageDir, showPlot)
fig = makePlotFigure('OptiTrack vs Odom Trajectory', showPlot);
hold on;

drawOccupancyMapBackground(getFirstMapData(results));

gtColor = [0.05 0.24 0.55];
odomColor = [0.72 0.23 0.12];
for i = 1:numel(results)
    if i == 1
        gtVisibility = 'on';
        odomVisibility = 'on';
    else
        gtVisibility = 'off';
        odomVisibility = 'off';
    end

    plot(results(i).gtPos(:, 1), results(i).gtPos(:, 2), '-', ...
        'Color', gtColor, 'LineWidth', 2.6, ...
        'DisplayName', 'OptiTrack', 'HandleVisibility', gtVisibility);
    plot(results(i).ekfPos(:, 1), results(i).ekfPos(:, 2), '--', ...
        'Color', odomColor, 'LineWidth', 2.6, ...
        'DisplayName', 'Odometry', 'HandleVisibility', odomVisibility);
end

axis equal;
grid on;
set(gca, 'LineWidth', 1.6, 'FontSize', 15, 'GridAlpha', 0.22, 'MinorGridAlpha', 0.12);
xlabel('map x [m]');
ylabel('map y [m]');
title('OptiTrack Ground Truth vs Initial-Aligned Odom Trajectory');
legend('Location', 'northeast', 'LineWidth', 1.2);
savePlotFigure(fig, outputDir, 'Odom_Trajectory_vs_OptiTrack', showPlot);

if ~exist(reportImageDir, 'dir')
    mkdir(reportImageDir);
end
copyfile(fullfile(outputDir, 'Odom_Trajectory_vs_OptiTrack.png'), ...
    fullfile(reportImageDir, 'odom_trajectory_vs_optitrack.png'));

summaryTable = table(numel(results), ...
    'VariableNames', {'runs'});
end

function [summaryTable, aggregateTable] = plotOdomMeanVariance(results, outputDir, reportImageDir, showPlot)
analysisStartS = 3.0;
valid = false(numel(results), 1);
for i = 1:numel(results)
    valid(i) = numel(results(i).tRel) >= 2 && ...
        nnz(isfinite(results(i).xyError) & results(i).tRel(:) >= analysisStartS) >= 2;
end
results = results(valid);
if isempty(results)
    error('No valid odom error traces available.');
end

commonEndS = min(arrayfun(@(r) max(r.tRel(isfinite(r.tRel))), results));
dt = median(cell2mat(arrayfun(@(r) median(diff(r.tRel(isfinite(r.tRel)))), results, ...
    'UniformOutput', false)), 'omitnan');
if ~isfinite(dt) || dt <= 0
    dt = (commonEndS - analysisStartS) / 1000;
end
tGrid = (analysisStartS:dt:commonEndS)';
if numel(tGrid) < 200
    tGrid = linspace(analysisStartS, commonEndS, 500)';
end
xPlot = tGrid - analysisStartS;

errGrid = nan(numel(tGrid), numel(results));
runMean = nan(numel(results), 1);
runRmse = nan(numel(results), 1);
runStd = nan(numel(results), 1);
runP95 = nan(numel(results), 1);
runMax = nan(numel(results), 1);
runDuration = nan(numel(results), 1);
runName = strings(numel(results), 1);

for i = 1:numel(results)
    t = results(i).tRel(:);
    e = results(i).xyError(:);
    mask = isfinite(t) & isfinite(e);
    [tUnique, uniqueIdx] = unique(t(mask), 'stable');
    eUnique = e(mask);
    eUnique = eUnique(uniqueIdx);
    errGrid(:, i) = interp1(tUnique, eUnique, tGrid, 'linear', NaN);
    metricMask = tUnique >= analysisStartS;
    eMetric = eUnique(metricMask);
    tMetric = tUnique(metricMask);

    runName(i) = string(results(i).bagName);
    runMean(i) = mean(eMetric, 'omitnan');
    runRmse(i) = sqrt(mean(eMetric .^ 2, 'omitnan'));
    runStd(i) = std(eMetric, 0, 'omitnan');
    runP95(i) = prctile(eMetric, 95);
    runMax(i) = max(eMetric, [], 'omitnan');
    runDuration(i) = max(tMetric) - min(tMetric);
end

meanErr = mean(errGrid, 2, 'omitnan');
stdErr = std(errGrid, 0, 2, 'omitnan');
varErr = var(errGrid, 0, 2, 'omitnan');
upper = meanErr + stdErr;
lower = max(meanErr - stdErr, 0);

fig = makePlotFigure('Odom Mean Position Error and Standard Deviation', showPlot);
hold on;
fill([xPlot; flipud(xPlot)], [upper; flipud(lower)], [0.20 0.45 0.80], ...
    'FaceAlpha', 0.24, 'EdgeColor', 'none', ...
    'DisplayName', 'run-to-run variation');
plot(xPlot, meanErr, '-', 'Color', [0.05 0.24 0.55], 'LineWidth', 3.2, ...
    'DisplayName', 'mean position error');
grid on;
set(gca, 'LineWidth', 1.6, 'FontSize', 15, 'GridAlpha', 0.22, 'MinorGridAlpha', 0.12);
xlabel('time after motion segment start [s]');
ylabel('position error [m]');
title(sprintf('Initial-Aligned Odom Position Error, n=%d Runs', numel(results)));
legend('Location', 'northwest', 'LineWidth', 1.2);
savePlotFigure(fig, outputDir, 'Odom_Position_Error_Mean_Std', showPlot);

if ~exist(reportImageDir, 'dir')
    mkdir(reportImageDir);
end
copyfile(fullfile(outputDir, 'Odom_Position_Error_Mean_Std.png'), ...
    fullfile(reportImageDir, 'odom_position_error_mean_std.png'));

summaryTable = table(runName, runDuration, runMean, runStd, runRmse, runP95, runMax, ...
    'VariableNames', {'run', 'duration_s', 'mean_error_m', 'std_error_m', ...
    'rmse_error_m', 'p95_error_m', 'max_error_m'});

aggregateTable = table(numel(results), commonEndS - analysisStartS, analysisStartS, ...
    mean(runMean, 'omitnan'), ...
    std(runMean, 0, 'omitnan'), mean(runRmse, 'omitnan'), mean(runP95, 'omitnan'), ...
    mean(runMax, 'omitnan'), max(meanErr, [], 'omitnan'), max(varErr, [], 'omitnan'), ...
    'VariableNames', {'runs', 'common_duration_s', 'analysis_start_s', 'mean_of_run_mean_error_m', ...
    'std_of_run_mean_error_m', 'mean_run_rmse_error_m', 'mean_run_p95_error_m', ...
    'mean_run_max_error_m', 'max_mean_error_m', 'max_time_variance_m2'});
end

function a = wrapAnglePiLocal(a)
a = atan2(sin(a), cos(a));
end
