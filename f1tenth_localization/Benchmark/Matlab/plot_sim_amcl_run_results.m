clc;
clear all
close all;
% Plot all CSV outputs from one simulated AMCL benchmark run.
%
% Optional workspace inputs before pressing Run:
%   runRootDir   - one sim_benchmark/<timestamp> folder, or one localizer folder
%   plotsRootDir - output root for generated PNG/CSV reports
%   showPlots    - true: save and show figures, false: save only
%   localizers   - optional cell/string list, for example {'gpu', 'nav2'}
%   skipFirstLaps - number of detected laps to exclude from metrics/plots
%   skipLastLaps  - number of detected final laps to exclude

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

plotFunctionsDir = fullfile(scriptDir, 'matlab plotting functions');
if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    error('Could not find plotting functions directory: %s', plotFunctionsDir);
end

if ~exist('runRootDir', 'var') || isempty(runRootDir)
    runRootDir = latestSimBenchmarkRun(fullfile(scriptDir, 'sim_benchmark'));
end
if ~exist('plotsRootDir', 'var') || isempty(plotsRootDir)
    plotsRootDir = fullfile(scriptDir, 'plots', 'SimAmclBenchmark');
end
if ~exist('showPlots', 'var') || isempty(showPlots)
    showPlots = false;
end
if ~exist('localizers', 'var')
    localizers = {};
end
if ~exist('skipFirstLaps', 'var') || isempty(skipFirstLaps)
    skipFirstLaps = 1;
end
if ~exist('skipLastLaps', 'var') || isempty(skipLastLaps)
    skipLastLaps = 0;
end

if ~isfolder(runRootDir)
    error('Run root does not exist: %s', runRootDir);
end
if ~isfolder(plotsRootDir)
    mkdir(plotsRootDir);
end

localizerDirs = discoverSimLocalizerDirs(runRootDir, localizers);
if isempty(localizerDirs)
    error('No localizer folders with groundtruth_at_ekf.csv found under %s', runRootDir);
end

[~, defaultRunName] = fileparts(stripTrailingSeparator(char(runRootDir)));
[runName, runOutputDir] = prepareOutputDirectory(defaultRunName, plotsRootDir);

fprintf('Input run folder : %s\n', runRootDir);
fprintf('Localizer folders: %d\n', numel(localizerDirs));
fprintf('Output folder    : %s\n', runOutputDir);
fprintf('Show plots       : %s\n', mat2str(logical(showPlots)));
fprintf('Skip first laps  : %d\n', skipFirstLaps);
fprintf('Skip last laps   : %d\n', skipLastLaps);

runInfos = struct('name', {}, 'dir', {}, 'result', {}, 'pipelineData', {});
failedRuns = {};

for i = 1:numel(localizerDirs)
    loc = localizerDirs(i);
    fprintf('\n[%d/%d] Localizer: %s\n', i, numel(localizerDirs), loc.name);
    fprintf('[%d/%d] Folder   : %s\n', i, numel(localizerDirs), loc.dir);

    try
        result = loadSimGroundTruthResult(loc.dir, loc.name, skipFirstLaps, skipLastLaps);
        localOutputDir = fullfile(runOutputDir, sanitizeFileName(loc.name));
        ensureDirectory(localOutputDir);
        fprintf('[%d/%d] Plots    : %s\n', i, numel(localizerDirs), localOutputDir);

        % Localizer folders keep local-only diagnostics. Comparison plots are
        % written once at the run root with all localizers together.
        plotOptiTrackErrorHeatmaps(result, localOutputDir, showPlots);
        plotSimCovariance(result, localOutputDir, showPlots);
        plotMpcOutputs(loc.dir, localOutputDir, showPlots);

        localSummaryPath = writeSimGroundTruthSummary(result, localOutputDir, 'Sim_Localizer_Summary');
        fprintf('[%d/%d] Summary  : %s\n', i, numel(localizerDirs), localSummaryPath);

        pipelineData = [];
        try
            pipelineData = loadSimPipelineMonitorData(loc.dir);
            if hasAnyPipelineMonitorData(pipelineData)
                pipelineOutputDir = fullfile(localOutputDir, 'pipeline_monitor');
                plotPipelineMonitorSuite(pipelineData, pipelineOutputDir, showPlots);
                writePipelineSummary(pipelineData, pipelineOutputDir, loc.name);
                fprintf('[%d/%d] Pipeline : %s\n', i, numel(localizerDirs), pipelineOutputDir);
            else
                fprintf('[%d/%d] Pipeline : no pipeline/system CSVs found\n', i, numel(localizerDirs));
            end
        catch ME
            warning('Pipeline monitor plots failed for %s: %s', loc.name, ME.message);
        end

        runInfos(end + 1).name = loc.name; %#ok<SAGROW>
        runInfos(end).dir = loc.dir;
        runInfos(end).result = result;
        runInfos(end).pipelineData = pipelineData;
    catch ME
        warning('Skipping localizer %s: %s', loc.name, ME.message);
        failedRuns(end + 1, :) = {loc.name, ME.message}; %#ok<SAGROW>
    end
end

if ~isempty(runInfos)
    allResults = [runInfos.result];
    plotSimRunComparison(runInfos, runOutputDir, showPlots);
    plotOptiTrackTrajectoryComparison(allResults, runOutputDir, showPlots);
    plotOptiTrackXYErrorOverTime(allResults, runOutputDir, showPlots);
    plotOptiTrackYawErrorOverTime(allResults, runOutputDir, showPlots);
    plotOptiTrackXYErrorScatter(allResults, runOutputDir, showPlots);
    plotOptiTrackPositionErrorBins(allResults, runOutputDir, showPlots);
    plotOptiTrackBenchmarkStatistics(allResults, runOutputDir, showPlots);
    plotOptiTrackErrorHeatmaps(allResults, runOutputDir, showPlots);
    summaryPath = writeSimGroundTruthSummary(allResults, runOutputDir, 'Sim_Run_Summary');
    pipelineSummaryPath = writeRunPipelineSummary(runInfos, runOutputDir);
    fprintf('\nRun summary saved to %s\n', summaryPath);
    if ~isempty(pipelineSummaryPath)
        fprintf('Pipeline summary saved to %s\n', pipelineSummaryPath);
    end
end

fprintf('\nBatch complete. Processed %d localizer folder(s).\n', numel(runInfos));
if ~isempty(failedRuns)
    fprintf('Failed localizer folder(s):\n');
    for i = 1:size(failedRuns, 1)
        fprintf('  %s: %s\n', failedRuns{i, 1}, failedRuns{i, 2});
    end
end
fprintf('Plots saved to %s\n', runOutputDir);

function runRootDir = latestSimBenchmarkRun(simRootDir)
if ~isfolder(simRootDir)
    error('Default sim benchmark root does not exist: %s', simRootDir);
end

entries = dir(simRootDir);
entries = entries([entries.isdir]);
entries = entries(~ismember({entries.name}, {'.', '..'}));

valid = false(numel(entries), 1);
for i = 1:numel(entries)
    candidate = fullfile(simRootDir, entries(i).name);
    valid(i) = ~isempty(discoverSimLocalizerDirs(candidate, {}));
end
entries = entries(valid);

if isempty(entries)
    error('No sim benchmark run folders found under %s', simRootDir);
end

[~, idx] = sort({entries.name});
entries = entries(idx);
runRootDir = fullfile(simRootDir, entries(end).name);
end

function localizerDirs = discoverSimLocalizerDirs(runRootDir, requestedLocalizers)
localizerDirs = struct('name', {}, 'dir', {});

if isfile(fullfile(runRootDir, 'groundtruth_at_ekf.csv'))
    [~, locName] = fileparts(stripTrailingSeparator(char(runRootDir)));
    localizerDirs(1).name = locName;
    localizerDirs(1).dir = runRootDir;
    return;
end

requested = normalizeRequestedLocalizers(requestedLocalizers);
entries = dir(runRootDir);
entries = entries([entries.isdir]);
entries = entries(~ismember({entries.name}, {'.', '..'}));

for i = 1:numel(entries)
    name = entries(i).name;
    if ~isempty(requested) && ~any(strcmp(string(name), requested))
        continue;
    end

    candidate = fullfile(runRootDir, name);
    if isfile(fullfile(candidate, 'groundtruth_at_ekf.csv'))
        localizerDirs(end + 1).name = name; %#ok<AGROW>
        localizerDirs(end).dir = candidate;
    end
end

if ~isempty(localizerDirs)
    [~, order] = sort({localizerDirs.name});
    localizerDirs = localizerDirs(order);
end
end

function requested = normalizeRequestedLocalizers(localizers)
if isempty(localizers)
    requested = strings(0, 1);
elseif ischar(localizers)
    requested = string({localizers});
elseif isstring(localizers)
    requested = localizers(:);
elseif iscell(localizers)
    requested = string(localizers(:));
else
    error('localizers must be a char, string, cell array, or empty');
end
requested = requested(strlength(requested) > 0);
end

function result = loadSimGroundTruthResult(localizerDir, localizerName, skipFirstLaps, skipLastLaps)
if nargin < 3 || isempty(skipFirstLaps)
    skipFirstLaps = 0;
end
if nargin < 4 || isempty(skipLastLaps)
    skipLastLaps = 0;
end

csvPath = fullfile(localizerDir, 'groundtruth_at_ekf.csv');
T = readtable(csvPath, 'VariableNamingRule', 'preserve');
assertColumnsLocal(T, {'wall_time_ns', 'ekf_x', 'ekf_y', 'ekf_yaw', ...
    'gt_x', 'gt_y', 'gt_yaw', 'err_x', 'err_y', 'err_xy', 'err_yaw'}, ...
    'groundtruth_at_ekf.csv');

if height(T) == 0
    error('Ground-truth CSV is empty: %s', csvPath);
end

tRel = relativeTimeSeconds(T.wall_time_ns);
gtPos = [double(T.gt_x), double(T.gt_y), zeros(height(T), 1)];
ekfPos = [double(T.ekf_x), double(T.ekf_y), zeros(height(T), 1)];
gtYaw = double(T.gt_yaw);
ekfYaw = double(T.ekf_yaw);

if ismember('amcl_x', T.Properties.VariableNames) && ismember('amcl_y', T.Properties.VariableNames)
    amclPos = [double(T.amcl_x), double(T.amcl_y), zeros(height(T), 1)];
else
    amclPos = NaN(height(T), 3);
end
if ismember('amcl_yaw', T.Properties.VariableNames)
    amclYaw = double(T.amcl_yaw);
else
    amclYaw = NaN(height(T), 1);
end

xError = double(T.err_x);
yError = double(T.err_y);
xyError = double(T.err_xy);
yawError = wrapAnglePiLocal(double(T.err_yaw));
amclXError = amclPos(:, 1) - gtPos(:, 1);
amclYError = amclPos(:, 2) - gtPos(:, 2);
amclXYError = hypot(amclXError, amclYError);
amclYawError = wrapAnglePiLocal(amclYaw - gtYaw);

collision = optionalNumericColumn(T, 'collision', zeros(height(T), 1));
metricMask = isfinite(xError) & isfinite(yError) & isfinite(xyError) & isfinite(yawError) & ...
    all(isfinite(gtPos(:, 1:2)), 2) & all(isfinite(ekfPos(:, 1:2)), 2) & collision == 0;
amclMetricMask = isfinite(amclXError) & isfinite(amclYError) & isfinite(amclXYError) & ...
    isfinite(amclYawError) & all(isfinite(gtPos(:, 1:2)), 2) & collision == 0;

result = struct();
result.bagName = char(localizerName);
result.sourceBagName = char(localizerName);
result.localizerName = char(localizerName);
result.runDir = localizerDir;
result.csvPath = csvPath;
result.particleCount = NaN;
result.tRel = tRel;
result.gtPos = gtPos;
result.ekfPos = ekfPos;
result.amclPos = amclPos;
result.gtYaw = gtYaw;
result.ekfYaw = ekfYaw;
result.amclYaw = amclYaw;
result.xError = xError;
result.yError = yError;
result.xyError = xyError;
result.yawError = yawError;
result.amclXError = amclXError;
result.amclYError = amclYError;
result.amclXYError = amclXYError;
result.amclYawError = amclYawError;
result.metricMask = metricMask;
result.amclMetricMask = amclMetricMask;
result.collision = collision;
result.progressS = optionalNumericColumn(T, 'progress_s', NaN(height(T), 1));
result.progressRatio = optionalNumericColumn(T, 'progress_ratio', NaN(height(T), 1));
result.lapCount = optionalNumericColumn(T, 'lap_count', NaN(height(T), 1));
result.gtVx = optionalNumericColumn(T, 'gt_vx', NaN(height(T), 1));
result.gtVy = optionalNumericColumn(T, 'gt_vy', NaN(height(T), 1));
result.gtWz = optionalNumericColumn(T, 'gt_wz', NaN(height(T), 1));
result.ekfCovX = optionalNumericColumn(T, 'ekf_cov_x', NaN(height(T), 1));
result.ekfCovY = optionalNumericColumn(T, 'ekf_cov_y', NaN(height(T), 1));
result.ekfCovYaw = optionalNumericColumn(T, 'ekf_cov_yaw', NaN(height(T), 1));
result.mapData = [];
result.trimApplied = false;
result.optitrackSamplesRaw = height(T);
result.nYawIsolatedSamplesRemoved = 0;
result.nYawOutlierLapsExcluded = 0;
result.yawOutlierLapsExcluded = [];

result.laps = buildSimLapSegments(result);
result.lapsAll = result.laps;
result.nLapsRaw = numel(result.laps);
[result.laps, lapTrimMask] = filterSimLaps( ...
    result.laps, skipFirstLaps, skipLastLaps, numel(result.metricMask));
if any(lapTrimMask)
    result.trimApplied = true;
    result.metricMask = result.metricMask & lapTrimMask;
    result.amclMetricMask = result.amclMetricMask & lapTrimMask;
end
result.nLaps = numel(result.laps);
if isempty(result.laps)
    result.lapDurationsS = maxFiniteLocal(result.tRel) - minFiniteLocal(result.tRel);
else
    result.lapDurationsS = [result.laps.durationS];
end
result.optitrackSamplesValid = nnz(result.metricMask);
result.optitrackSamplesRejected = height(T) - nnz(result.metricMask);
result = addSimSummaryFields(result);
end

function result = addSimSummaryFields(result)
mask = result.metricMask(:);
result.nSamples = numel(result.xyError);
result.nMetricSamples = nnz(mask);
result.nExcludedSamples = result.nSamples - result.nMetricSamples;
if isfield(result, 'lapDurationsS') && ~isempty(result.lapDurationsS)
    result.durationS = sum(result.lapDurationsS, 'omitnan');
else
    result.durationS = maxFiniteLocal(result.tRel) - minFiniteLocal(result.tRel);
end
result.meanXError = mean(result.xError(mask), 'omitnan');
result.stdXError = std(result.xError(mask), 0, 'omitnan');
result.meanYError = mean(result.yError(mask), 'omitnan');
result.stdYError = std(result.yError(mask), 0, 'omitnan');
result.meanXYError = mean(result.xyError(mask), 'omitnan');
result.stdXYError = std(result.xyError(mask), 0, 'omitnan');
result.rmseXYError = sqrt(mean(result.xyError(mask).^2, 'omitnan'));
result.stdRmseXYError = NaN;
result.maxXYError = maxFiniteLocal(result.xyError(mask));
result.meanAbsYawError = mean(abs(result.yawError(mask)), 'omitnan');
result.stdYawError = std(abs(result.yawError(mask)), 0, 'omitnan');
result.rmseYawError = sqrt(mean(result.yawError(mask).^2, 'omitnan'));
result.stdRmseYawError = NaN;
result.maxAbsYawError = maxFiniteLocal(abs(result.yawError(mask)));
end

function laps = buildSimLapSegments(result)
laps = struct('bagName', {}, 'sourceBagName', {}, 'localizerName', {}, ...
    'sourceIndex', {}, 'lapNumber', {}, 'tRel', {}, 'gtPos', {}, 'ekfPos', {}, ...
    'amclPos', {}, 'gtYaw', {}, 'ekfYaw', {}, 'amclYaw', {}, 'xError', {}, ...
    'yError', {}, 'xyError', {}, 'yawError', {}, 'amclXError', {}, ...
    'amclYError', {}, 'amclXYError', {}, 'amclYawError', {}, 'metricMask', {}, ...
    'amclMetricMask', {}, 'durationS', {});

lapCount = result.lapCount(:);
if ~any(isfinite(lapCount))
    idx = (1:numel(result.tRel))';
    laps = makeSimLapSegment(result, idx, 1);
    return;
end

lapValues = unique(lapCount(isfinite(lapCount)), 'stable');
for i = 1:numel(lapValues)
    idx = find(lapCount == lapValues(i));
    if numel(idx) < 5
        continue;
    end
    if isfield(result, 'progressRatio')
        progress = result.progressRatio(idx);
        progress = progress(isfinite(progress));
        if numel(progress) >= 2 && (max(progress) - min(progress)) < 0.85
            continue;
        end
    end
    laps(end + 1) = makeSimLapSegment(result, idx, lapValues(i)); %#ok<AGROW>
end

if isempty(laps)
    idx = (1:numel(result.tRel))';
    laps = makeSimLapSegment(result, idx, 1);
end
end

function lap = makeSimLapSegment(result, idx, lapNumber)
lap = struct();
lap.bagName = sprintf('%s lap %g', result.bagName, lapNumber);
lap.sourceBagName = result.sourceBagName;
lap.localizerName = result.localizerName;
lap.sourceIndex = idx(:);
lap.lapNumber = lapNumber;
lap.tRel = result.tRel(idx);
lap.gtPos = result.gtPos(idx, :);
lap.ekfPos = result.ekfPos(idx, :);
lap.amclPos = result.amclPos(idx, :);
lap.gtYaw = result.gtYaw(idx);
lap.ekfYaw = result.ekfYaw(idx);
lap.amclYaw = result.amclYaw(idx);
lap.xError = result.xError(idx);
lap.yError = result.yError(idx);
lap.xyError = result.xyError(idx);
lap.yawError = result.yawError(idx);
lap.amclXError = result.amclXError(idx);
lap.amclYError = result.amclYError(idx);
lap.amclXYError = result.amclXYError(idx);
lap.amclYawError = result.amclYawError(idx);
lap.metricMask = result.metricMask(idx);
lap.amclMetricMask = result.amclMetricMask(idx);
lap.durationS = maxFiniteLocal(lap.tRel) - minFiniteLocal(lap.tRel);
end

function [lapsOut, keepMask] = filterSimLaps(laps, skipFirstLaps, skipLastLaps, nSamples)
if nargin < 4 || isempty(nSamples)
    nSamples = 0;
end

if isempty(laps)
    lapsOut = laps;
    keepMask = false(nSamples, 1);
    return;
end

n = numel(laps);
firstIdx = max(1, 1 + max(0, round(skipFirstLaps)));
lastIdx = min(n, n - max(0, round(skipLastLaps)));
if firstIdx > lastIdx
    warning('Lap filter removed all %d laps. Keeping all laps instead.', n);
    firstIdx = 1;
    lastIdx = n;
end

selected = firstIdx:lastIdx;
lapsOut = laps(selected);
keepMask = false(nSamples, 1);
for i = selected
    idx = laps(i).sourceIndex;
    idx = idx(idx >= 1 & idx <= nSamples);
    keepMask(idx) = true;
end
end

function plotSimTrajectoryComparison(result, outputDir, showPlot)
fig = makePlotFigure('Simulation GT, AMCL, and EKF Trajectory', showPlot);
hold on;
metricMask = result.metricMask(:);

plotMaskedXY(result.gtPos(:, 1:2), ~metricMask, '-', [0.70 0.70 0.70], 1.1, 'off', '');
plotMaskedXY(result.ekfPos(:, 1:2), ~metricMask, '--', [0.70 0.70 0.70], 0.9, 'off', '');
plotMaskedXY(result.gtPos(:, 1:2), metricMask, '-', [0.05 0.05 0.05], 2.0, 'on', 'ground truth');
plotMaskedXY(result.ekfPos(:, 1:2), metricMask, '--', [0.00 0.45 0.74], 1.4, 'on', 'EKF');
plotMaskedXY(result.amclPos(:, 1:2), metricMask, ':', [0.85 0.33 0.10], 1.6, 'on', 'AMCL');

collisionMask = result.collision(:) ~= 0 & all(isfinite(result.gtPos(:, 1:2)), 2);
if any(collisionMask)
    scatter(result.gtPos(collisionMask, 1), result.gtPos(collisionMask, 2), 34, ...
        [0.80 0.00 0.00], 'filled', 'DisplayName', 'collision');
end

axis equal;
grid on;
xlabel('map x [m]');
ylabel('map y [m]');
title(sprintf('%s Simulation Trajectory', result.bagName), 'Interpreter', 'none');
legend('Location', 'bestoutside', 'Interpreter', 'none');
savePlotFigure(fig, outputDir, 'Sim_Trajectory_GT_AMCL_EKF', showPlot);
end

function plotMaskedXY(xy, mask, lineStyle, color, lineWidth, handleVisibility, displayName)
xyPlot = xy;
mask = mask(:) & all(isfinite(xyPlot), 2);
xyPlot(~mask, :) = NaN;
plot(xyPlot(:, 1), xyPlot(:, 2), lineStyle, 'Color', color, 'LineWidth', lineWidth, ...
    'HandleVisibility', handleVisibility, 'DisplayName', displayName);
end

function plotSimPoseAndProgress(result, outputDir, showPlot)
fig = makePlotFigure('Simulation Pose and Progress', showPlot);
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
plot(result.tRel, result.gtPos(:, 1), 'k-', 'LineWidth', 1.4, 'DisplayName', 'GT x');
plot(result.tRel, result.ekfPos(:, 1), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.0, 'DisplayName', 'EKF x');
plot(result.tRel, result.amclPos(:, 1), ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2, 'DisplayName', 'AMCL x');
grid on;
ylabel('x [m]');
title(sprintf('%s Pose Time Series', result.bagName), 'Interpreter', 'none');
legend('Location', 'bestoutside', 'Interpreter', 'none');

nexttile;
hold on;
plot(result.tRel, result.gtPos(:, 2), 'k-', 'LineWidth', 1.4, 'DisplayName', 'GT y');
plot(result.tRel, result.ekfPos(:, 2), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.0, 'DisplayName', 'EKF y');
plot(result.tRel, result.amclPos(:, 2), ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2, 'DisplayName', 'AMCL y');
grid on;
ylabel('y [m]');

nexttile;
hold on;
plot(result.tRel, rad2deg(unwrap(result.gtYaw)), 'k-', 'LineWidth', 1.4, 'DisplayName', 'GT yaw');
plot(result.tRel, rad2deg(unwrap(result.ekfYaw)), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.0, 'DisplayName', 'EKF yaw');
plot(result.tRel, rad2deg(unwrap(result.amclYaw)), ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2, 'DisplayName', 'AMCL yaw');
grid on;
ylabel('yaw [deg]');

nexttile;
yyaxis left;
plot(result.tRel, result.progressRatio, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.1);
ylabel('progress ratio');
yyaxis right;
stairs(result.tRel, result.lapCount, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
ylabel('lap count');
grid on;
xlabel('time [s]');

savePlotFigure(fig, outputDir, 'Sim_Pose_And_Progress', showPlot);
end

function plotSimCovariance(result, outputDir, showPlot)
if ~any(isfinite(result.ekfCovX)) && ~any(isfinite(result.ekfCovY)) && ~any(isfinite(result.ekfCovYaw))
    return;
end

fig = makePlotFigure('Simulation EKF Covariance', showPlot);
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(result.tRel, result.ekfCovX, 'LineWidth', 1.0);
grid on;
ylabel('cov x');
title(sprintf('%s EKF covariance', result.bagName), 'Interpreter', 'none');

nexttile;
plot(result.tRel, result.ekfCovY, 'LineWidth', 1.0);
grid on;
ylabel('cov y');

nexttile;
plot(result.tRel, result.ekfCovYaw, 'LineWidth', 1.0);
grid on;
ylabel('cov yaw');
xlabel('time [s]');

savePlotFigure(fig, outputDir, 'Sim_EKF_Covariance', showPlot);
end

function plotSimAmclEkfDifference(result, outputDir, showPlot)
if ~any(isfinite(result.amclPos(:, 1)))
    return;
end

dX = result.ekfPos(:, 1) - result.amclPos(:, 1);
dY = result.ekfPos(:, 2) - result.amclPos(:, 2);
dXY = hypot(dX, dY);
dYaw = wrapAnglePiLocal(result.ekfYaw - result.amclYaw);

fig = makePlotFigure('Simulation AMCL to EKF Difference', showPlot);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(result.tRel, dX, 'LineWidth', 1.0);
grid on;
ylabel('x [m]');
title('EKF - AMCL x');

nexttile;
plot(result.tRel, dY, 'LineWidth', 1.0);
grid on;
ylabel('y [m]');
title('EKF - AMCL y');

nexttile;
plot(result.tRel, dXY, 'LineWidth', 1.0);
grid on;
xlabel('time [s]');
ylabel('XY [m]');
title('EKF - AMCL XY norm');

nexttile;
plot(result.tRel, rad2deg(dYaw), 'LineWidth', 1.0);
grid on;
xlabel('time [s]');
ylabel('yaw [deg]');
title('EKF - AMCL yaw');

savePlotFigure(fig, outputDir, 'Sim_AMCL_EKF_Difference', showPlot);
end

function plotMpcOutputs(localizerDir, outputDir, showPlot)
mpcDir = fullfile(localizerDir, 'mpc');
if ~isfolder(mpcDir)
    return;
end

solverCsv = fullfile(mpcDir, 'solver.csv');
if isfile(solverCsv)
    try
        plotMpcSolverSummary(solverCsv, outputDir, showPlot);
    catch ME
        warning('MPC solver plot failed for %s: %s', solverCsv, ME.message);
    end
end

racelineCsv = fullfile(mpcDir, 'local_raceline.csv');
if isfile(racelineCsv)
    try
        plotMpcLocalRaceline(racelineCsv, outputDir, showPlot);
    catch ME
        warning('MPC raceline plot failed for %s: %s', racelineCsv, ME.message);
    end
end
end

function plotMpcSolverSummary(solverCsv, outputDir, showPlot)
T = readtable(solverCsv, 'VariableNamingRule', 'preserve');
if height(T) == 0
    return;
end

t = relativeTimeSeconds(pickTimeColumn(T, {'pose_ros_time_ns', 'unix_time_ns', 'odom_ros_time_ns'}));
solveUs = optionalNumericColumn(T, 'solve_us', NaN(height(T), 1));
iterations = optionalNumericColumn(T, 'iterations', NaN(height(T), 1));
status = optionalNumericColumn(T, 'status', NaN(height(T), 1));
cmdSteer = optionalNumericColumn(T, 'cmd_steer', NaN(height(T), 1));
cmdAccel = optionalNumericColumn(T, 'cmd_accel', NaN(height(T), 1));
vx = optionalNumericColumn(T, 'vx', NaN(height(T), 1));
vRef0 = optionalNumericColumn(T, 'v_ref0', NaN(height(T), 1));
eY = optionalNumericColumn(T, 'e_y', NaN(height(T), 1));
ePsi = optionalNumericColumn(T, 'e_psi', NaN(height(T), 1));

fig = makePlotFigure('MPC Solver Summary', showPlot);
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t, solveUs, 'LineWidth', 1.0);
grid on;
ylabel('solve [us]');
title('MPC solve time');

nexttile;
yyaxis left;
plot(t, iterations, 'LineWidth', 1.0);
ylabel('iterations');
yyaxis right;
stairs(t, status, 'LineWidth', 1.0);
ylabel('status');
grid on;

nexttile;
hold on;
plot(t, vx, 'LineWidth', 1.0, 'DisplayName', 'vx');
plot(t, vRef0, '--', 'LineWidth', 1.0, 'DisplayName', 'v ref');
grid on;
ylabel('speed [m/s]');
legend('Location', 'best');

nexttile;
hold on;
plot(t, cmdSteer, 'LineWidth', 1.0, 'DisplayName', 'cmd steer');
plot(t, cmdAccel, 'LineWidth', 1.0, 'DisplayName', 'cmd accel');
plot(t, eY, '--', 'LineWidth', 0.9, 'DisplayName', 'e_y');
plot(t, ePsi, '--', 'LineWidth', 0.9, 'DisplayName', 'e_\psi');
grid on;
xlabel('time [s]');
ylabel('command/error');
legend('Location', 'bestoutside');

savePlotFigure(fig, outputDir, 'MPC_Solver_Summary', showPlot);
end

function plotMpcLocalRaceline(racelineCsv, outputDir, showPlot)
T = readtable(racelineCsv, 'VariableNamingRule', 'preserve');
if height(T) == 0
    return;
end
assertColumnsLocal(T, {'local_raceline_seq', 'x_m', 'y_m', 'v_ref_mps'}, 'local_raceline.csv');

seq = double(T.local_raceline_seq);
seqValues = unique(seq(isfinite(seq)), 'stable');
if isempty(seqValues)
    return;
end
lastSeq = seqValues(end);
mask = seq == lastSeq;

fig = makePlotFigure('MPC Local Raceline Last Snapshot', showPlot);
scatter(double(T.x_m(mask)), double(T.y_m(mask)), 20, double(T.v_ref_mps(mask)), 'filled');
axis equal;
grid on;
xlabel('map x [m]');
ylabel('map y [m]');
title(sprintf('Last local raceline seq %g', lastSeq));
cb = colorbar;
ylabel(cb, 'v ref [m/s]');

savePlotFigure(fig, outputDir, 'MPC_Local_Raceline_Last', showPlot);
end

function data = loadSimPipelineMonitorData(localizerDir)
pipelineDir = fullfile(localizerDir, 'pipeline');
systemDir = fullfile(localizerDir, 'system');
data = [];

if isfolder(pipelineDir)
    data = loadPipelineMonitorData(pipelineDir, pipelineDir);
end

if isfolder(systemDir)
    systemData = loadPipelineMonitorData(systemDir, systemDir);
    if isempty(data)
        data = systemData;
    else
        data.cpu = systemData.cpu;
        data.gpu = systemData.gpu;
        data.perCore = systemData.perCore;
        data.node = systemData.node;
    end
end

if isempty(data)
    data = emptyPipelineMonitorData(localizerDir);
end
end

function data = emptyPipelineMonitorData(csvDir)
data = struct();
data.csvDir = csvDir;
data.topicCsvDir = csvDir;
data.pipeline = struct('hasData', false);
data.cpu = struct('hasLong', false, 'hasShort', false);
data.gpu = struct('hasFile', false, 'hasLong', false, 'hasShort', false);
data.perCore = struct('hasData', false);
data.node = struct('hasData', false);
data.amcl = struct('hasTimingParticles', false);
end

function tf = hasAnyPipelineMonitorData(data)
tf = isstruct(data) && ...
    ((isfield(data, 'pipeline') && isfield(data.pipeline, 'hasData') && data.pipeline.hasData) || ...
     (isfield(data, 'cpu') && ((isfield(data.cpu, 'hasLong') && data.cpu.hasLong) || ...
                               (isfield(data.cpu, 'hasShort') && data.cpu.hasShort))) || ...
     (isfield(data, 'gpu') && ((isfield(data.gpu, 'hasFile') && data.gpu.hasFile) || ...
                               (isfield(data.gpu, 'hasLong') && data.gpu.hasLong) || ...
                               (isfield(data.gpu, 'hasShort') && data.gpu.hasShort))) || ...
     (isfield(data, 'perCore') && isfield(data.perCore, 'hasData') && data.perCore.hasData) || ...
     (isfield(data, 'node') && isfield(data.node, 'hasData') && data.node.hasData) || ...
     (isfield(data, 'amcl') && isfield(data.amcl, 'hasTimingParticles') && data.amcl.hasTimingParticles));
end

function plotPipelineMonitorSuite(data, outputDir, showPlot)
if ~isfolder(outputDir)
    mkdir(outputDir);
end

plotters = {@plotPipelineLatencyOverTime, @plotLatencyHistograms, @plotLatencyBoxplot, ...
    @plotCpuWindows, @plotPerCoreCpu, @plotGpuUsage, @plotPerNodeCpu, ...
    @plotAmclTimingParticleHeatmap};

for i = 1:numel(plotters)
    try
        feval(plotters{i}, data, outputDir, showPlot);
    catch ME
        warning('plotSimAmclRun:PipelinePlotterFailed', ...
            'Pipeline monitor plotter %d failed: %s', i, ME.message);
    end
end

try
    printBenchmarkSummary(data);
catch ME
    warning('plotSimAmclRun:PipelineSummaryFailed', ...
        'Pipeline monitor summary failed: %s', ME.message);
end
end

function plotSimRunComparison(runInfos, outputDir, showPlot)
if numel(runInfos) < 1
    return;
end

results = [runInfos.result];
plotSimComparisonTrajectory(results, outputDir, showPlot);
plotSimErrorCdf(results, outputDir, showPlot);
plotSimSummaryBars(results, outputDir, showPlot);
plotPipelineComparison(runInfos, outputDir, showPlot);
end

function plotSimComparisonTrajectory(results, outputDir, showPlot)
fig = makePlotFigure('Simulation Localizer Trajectory Comparison', showPlot);
hold on;
colors = lines(max(numel(results), 1));

plot(results(1).gtPos(:, 1), results(1).gtPos(:, 2), 'k-', 'LineWidth', 2.0, ...
    'DisplayName', 'ground truth');
for i = 1:numel(results)
    mask = results(i).metricMask(:);
    xy = results(i).ekfPos(:, 1:2);
    xy(~mask, :) = NaN;
    plot(xy(:, 1), xy(:, 2), '--', 'Color', colors(i, :), 'LineWidth', 1.3, ...
        'DisplayName', sprintf('%s EKF', results(i).bagName));
end

axis equal;
grid on;
xlabel('map x [m]');
ylabel('map y [m]');
title('GT vs EKF by Localizer');
legend('Location', 'bestoutside', 'Interpreter', 'none');
savePlotFigure(fig, outputDir, 'Sim_Localizer_Trajectory_Comparison', showPlot);
end

function plotSimErrorCdf(results, outputDir, showPlot)
fig = makePlotFigure('Simulation Error CDF', showPlot);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
colors = lines(max(numel(results), 1));

nexttile;
hold on;
for i = 1:numel(results)
    values = metricValues(results(i).xyError, results(i).metricMask);
    [x, y] = empiricalCdfLocal(values);
    plot(x, y, 'LineWidth', 1.4, 'Color', colors(i, :), 'DisplayName', results(i).bagName);
end
grid on;
xlabel('XY error [m]');
ylabel('CDF');
title('XY error CDF');
legend('Location', 'best', 'Interpreter', 'none');

nexttile;
hold on;
for i = 1:numel(results)
    values = rad2deg(abs(metricValues(results(i).yawError, results(i).metricMask)));
    [x, y] = empiricalCdfLocal(values);
    plot(x, y, 'LineWidth', 1.4, 'Color', colors(i, :), 'DisplayName', results(i).bagName);
end
grid on;
xlabel('|yaw error| [deg]');
ylabel('CDF');
title('Yaw error CDF');

savePlotFigure(fig, outputDir, 'Sim_Error_CDF', showPlot);
end

function plotSimSummaryBars(results, outputDir, showPlot)
labels = string({results.bagName});
meanXY = arrayfun(@(r) mean(metricValues(r.xyError, r.metricMask), 'omitnan'), results);
p95XY = arrayfun(@(r) percentileLocal(metricValues(r.xyError, r.metricMask), 95), results);
rmseXY = arrayfun(@(r) sqrt(mean(metricValues(r.xyError, r.metricMask).^2, 'omitnan')), results);
meanYaw = arrayfun(@(r) mean(rad2deg(abs(metricValues(r.yawError, r.metricMask))), 'omitnan'), results);
p95Yaw = arrayfun(@(r) percentileLocal(rad2deg(abs(metricValues(r.yawError, r.metricMask))), 95), results);

fig = makePlotFigure('Simulation Summary Bars', showPlot);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
bar(categorical(labels), [meanXY(:), p95XY(:), rmseXY(:)]);
grid on;
ylabel('XY error [m]');
title('Position error');
legend({'mean', 'p95', 'RMSE'}, 'Location', 'best');

nexttile;
bar(categorical(labels), [meanYaw(:), p95Yaw(:)]);
grid on;
ylabel('|yaw error| [deg]');
title('Yaw error');
legend({'mean', 'p95'}, 'Location', 'best');

savePlotFigure(fig, outputDir, 'Sim_Error_Summary_Bars', showPlot);
end

function plotPipelineComparison(runInfos, outputDir, showPlot)
valid = false(numel(runInfos), 1);
for i = 1:numel(runInfos)
    valid(i) = hasAnyPipelineMonitorData(runInfos(i).pipelineData) && ...
        isfield(runInfos(i).pipelineData, 'pipeline') && runInfos(i).pipelineData.pipeline.hasData;
end
if ~any(valid)
    return;
end

labels = string({runInfos(valid).name});
metrics = {'scanStamp2Scan', 'scan2amcl', 'amcl2ekf', 'scan2ekf', 'e2d', 'd2a', 's2a'};
metricLabels = {'scan stamp->rx', 'scan->amcl', 'amcl->ekf', 'scan->ekf', ...
    'ekf->drive', 'drive->ackermann', 'scan->ackermann'};
meanVals = NaN(nnz(valid), numel(metrics));
p95Vals = NaN(nnz(valid), numel(metrics));
validInfos = runInfos(valid);

for i = 1:numel(validInfos)
    p = validInfos(i).pipelineData.pipeline;
    for j = 1:numel(metrics)
        y = pipelineMetricValues(p, metrics{j});
        meanVals(i, j) = mean(y, 'omitnan');
        p95Vals(i, j) = percentileLocal(y, 95);
    end
end

fig = makePlotFigure('Pipeline Latency Comparison', showPlot);
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
bar(categorical(labels), meanVals);
grid on;
ylabel('mean latency [ms]');
title('Mean pipeline latency');
legend(metricLabels, 'Location', 'bestoutside', 'Interpreter', 'none');

nexttile;
bar(categorical(labels), p95Vals);
grid on;
ylabel('p95 latency [ms]');
title('P95 pipeline latency');
legend(metricLabels, 'Location', 'bestoutside', 'Interpreter', 'none');

savePlotFigure(fig, outputDir, 'Pipeline_Latency_Comparison', showPlot);
end

function summaryPath = writeSimGroundTruthSummary(results, outputDir, fileStem)
ensureDirectory(outputDir);

n = numel(results);
localizer = strings(n, 1);
duration_s = NaN(n, 1);
n_samples = NaN(n, 1);
n_metric_samples = NaN(n, 1);
n_excluded_samples = NaN(n, 1);
n_laps = NaN(n, 1);
lap_mean_s = NaN(n, 1);
mean_x_error_m = NaN(n, 1);
mean_y_error_m = NaN(n, 1);
mean_xy_error_m = NaN(n, 1);
median_xy_error_m = NaN(n, 1);
p95_xy_error_m = NaN(n, 1);
max_xy_error_m = NaN(n, 1);
rmse_xy_error_m = NaN(n, 1);
mean_abs_yaw_error_deg = NaN(n, 1);
p95_abs_yaw_error_deg = NaN(n, 1);
max_abs_yaw_error_deg = NaN(n, 1);
mean_amcl_xy_error_m = NaN(n, 1);
p95_amcl_xy_error_m = NaN(n, 1);
mean_abs_amcl_yaw_error_deg = NaN(n, 1);
p95_abs_amcl_yaw_error_deg = NaN(n, 1);
mean_ekf_cov_x = NaN(n, 1);
mean_ekf_cov_y = NaN(n, 1);
mean_ekf_cov_yaw = NaN(n, 1);
collision_samples = NaN(n, 1);

for i = 1:n
    r = results(i);
    xy = metricValues(r.xyError, r.metricMask);
    yawDeg = rad2deg(abs(metricValues(r.yawError, r.metricMask)));
    amclMask = getAmclMetricMask(r);
    amclXY = metricValues(r.amclXYError, amclMask);
    amclYawDeg = rad2deg(abs(metricValues(r.amclYawError, amclMask)));

    localizer(i) = string(r.bagName);
    duration_s(i) = r.durationS;
    n_samples(i) = r.nSamples;
    n_metric_samples(i) = r.nMetricSamples;
    n_excluded_samples(i) = r.nExcludedSamples;
    n_laps(i) = r.nLaps;
    lap_mean_s(i) = mean(r.lapDurationsS, 'omitnan');
    mean_x_error_m(i) = mean(metricValues(r.xError, r.metricMask), 'omitnan');
    mean_y_error_m(i) = mean(metricValues(r.yError, r.metricMask), 'omitnan');
    mean_xy_error_m(i) = mean(xy, 'omitnan');
    median_xy_error_m(i) = median(xy, 'omitnan');
    p95_xy_error_m(i) = percentileLocal(xy, 95);
    max_xy_error_m(i) = maxFiniteLocal(xy);
    rmse_xy_error_m(i) = sqrt(mean(xy.^2, 'omitnan'));
    mean_abs_yaw_error_deg(i) = mean(yawDeg, 'omitnan');
    p95_abs_yaw_error_deg(i) = percentileLocal(yawDeg, 95);
    max_abs_yaw_error_deg(i) = maxFiniteLocal(yawDeg);
    mean_amcl_xy_error_m(i) = mean(amclXY, 'omitnan');
    p95_amcl_xy_error_m(i) = percentileLocal(amclXY, 95);
    mean_abs_amcl_yaw_error_deg(i) = mean(amclYawDeg, 'omitnan');
    p95_abs_amcl_yaw_error_deg(i) = percentileLocal(amclYawDeg, 95);
    mean_ekf_cov_x(i) = mean(r.ekfCovX, 'omitnan');
    mean_ekf_cov_y(i) = mean(r.ekfCovY, 'omitnan');
    mean_ekf_cov_yaw(i) = mean(r.ekfCovYaw, 'omitnan');
    collision_samples(i) = nnz(r.collision ~= 0);
end

summaryTable = table(localizer, duration_s, n_samples, n_metric_samples, n_excluded_samples, ...
    n_laps, lap_mean_s, mean_x_error_m, mean_y_error_m, mean_xy_error_m, ...
    median_xy_error_m, p95_xy_error_m, max_xy_error_m, rmse_xy_error_m, ...
    mean_abs_yaw_error_deg, p95_abs_yaw_error_deg, max_abs_yaw_error_deg, ...
    mean_amcl_xy_error_m, p95_amcl_xy_error_m, ...
    mean_abs_amcl_yaw_error_deg, p95_abs_amcl_yaw_error_deg, ...
    mean_ekf_cov_x, mean_ekf_cov_y, mean_ekf_cov_yaw, collision_samples);

summaryPath = fullfile(outputDir, [sanitizeFileName(fileStem), '.csv']);
writetable(summaryTable, summaryPath);

fprintf('\n=== Simulation Ground Truth Summary ===\n');
disp(summaryTable);
end

function summaryPath = writePipelineSummary(data, outputDir, localizerName)
ensureDirectory(outputDir);
T = pipelineSummaryTable(data, localizerName);
summaryPath = fullfile(outputDir, 'Pipeline_Latency_Summary.csv');
writetable(T, summaryPath);
end

function summaryPath = writeRunPipelineSummary(runInfos, outputDir)
rows = [];
for i = 1:numel(runInfos)
    if hasAnyPipelineMonitorData(runInfos(i).pipelineData) && ...
            isfield(runInfos(i).pipelineData, 'pipeline') && runInfos(i).pipelineData.pipeline.hasData
        row = pipelineSummaryTable(runInfos(i).pipelineData, runInfos(i).name);
        if isempty(rows)
            rows = row;
        else
            rows = [rows; row]; %#ok<AGROW>
        end
    end
end

if isempty(rows)
    summaryPath = '';
    return;
end

summaryPath = fullfile(outputDir, 'Pipeline_Latency_Summary.csv');
writetable(rows, summaryPath);
end

function T = pipelineSummaryTable(data, localizerName)
metrics = {'scanStamp2Scan', 'scan2amcl', 'amcl2ekf', 'scan2ekf', 'e2d', 'd2a', 's2a'};
labels = {'scan_stamp_to_scan_ms', 'scan_to_amcl_ms', 'amcl_to_ekf_ms', ...
    'scan_to_ekf_ms', 'ekf_to_drive_ms', 'drive_to_ackermann_ms', 'scan_to_ackermann_ms'};

localizer = strings(numel(metrics), 1);
stage = strings(numel(metrics), 1);
mean_ms = NaN(numel(metrics), 1);
var_ms2 = NaN(numel(metrics), 1);
p95_ms = NaN(numel(metrics), 1);
max_ms = NaN(numel(metrics), 1);
n_samples = NaN(numel(metrics), 1);

for i = 1:numel(metrics)
    y = pipelineMetricValues(data.pipeline, metrics{i});
    localizer(i) = string(localizerName);
    stage(i) = string(labels{i});
    mean_ms(i) = mean(y, 'omitnan');
    var_ms2(i) = var(y, 0, 'omitnan');
    p95_ms(i) = percentileLocal(y, 95);
    max_ms(i) = maxFiniteLocal(y);
    n_samples(i) = nnz(isfinite(y));
end

T = table(localizer, stage, mean_ms, var_ms2, p95_ms, max_ms, n_samples);
end

function values = pipelineMetricValues(pipeline, metricName)
values = NaN(0, 1);
if ~isstruct(pipeline) || ~isfield(pipeline, 'hasData') || ~pipeline.hasData
    return;
end

switch metricName
    case 'scanStamp2Scan'
        if isfield(pipeline, 'hasScanStamp2Scan') && pipeline.hasScanStamp2Scan
            values = pipeline.scanStamp2Scan;
        end
    case 'scan2amcl'
        values = pipeline.scan2amcl;
    case 'amcl2ekf'
        values = pipeline.amcl2ekf;
    case 'scan2ekf'
        values = pipeline.scan2ekf;
    case 'e2d'
        if isfield(pipeline, 'hasE2D') && pipeline.hasE2D
            values = pipeline.e2d;
        end
    case 'd2a'
        if isfield(pipeline, 'hasD2A') && pipeline.hasD2A
            values = pipeline.d2a;
        end
    case 's2a'
        if isfield(pipeline, 'hasS2A') && pipeline.hasS2A
            values = pipeline.s2a;
        end
    otherwise
        error('Unknown pipeline metric: %s', metricName);
end

values = values(:);
values = values(isfinite(values) & values >= 0);
end

function values = metricValues(valuesIn, metricMask)
values = valuesIn(:);
mask = metricMask(:);
if numel(mask) ~= numel(values)
    mask = true(size(values));
end
values = values(mask & isfinite(values));
end

function mask = getAmclMetricMask(result)
if isfield(result, 'amclMetricMask') && numel(result.amclMetricMask) == numel(result.amclXYError)
    mask = result.amclMetricMask(:);
else
    mask = isfinite(result.amclXYError(:));
end
end

function [x, y] = empiricalCdfLocal(values)
values = values(:);
values = sort(values(isfinite(values)));
if isempty(values)
    x = NaN;
    y = NaN;
    return;
end
x = values;
y = (1:numel(values))' ./ numel(values);
end

function p = percentileLocal(values, q)
values = sort(values(isfinite(values)));
if isempty(values)
    p = NaN;
    return;
end
if isscalar(values)
    p = values(1);
    return;
end
q = max(0, min(100, q));
pos = 1 + (numel(values) - 1) * q / 100;
lo = floor(pos);
hi = ceil(pos);
if lo == hi
    p = values(lo);
else
    alpha = pos - lo;
    p = (1 - alpha) * values(lo) + alpha * values(hi);
end
end

function value = maxFiniteLocal(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = minFiniteLocal(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function t = relativeTimeSeconds(timeNs)
timeNs = double(timeNs(:));
if isempty(timeNs) || ~any(isfinite(timeNs))
    t = NaN(size(timeNs));
    return;
end
t0 = timeNs(find(isfinite(timeNs), 1, 'first'));
t = (timeNs - t0) * 1e-9;
end

function col = optionalNumericColumn(T, name, defaultValue)
if ismember(name, T.Properties.VariableNames)
    col = double(T.(name));
else
    col = defaultValue;
end
col = col(:);
end

function timeNs = pickTimeColumn(T, candidates)
for i = 1:numel(candidates)
    if ismember(candidates{i}, T.Properties.VariableNames)
        timeNs = T.(candidates{i});
        return;
    end
end
timeNs = (1:height(T))';
end

function assertColumnsLocal(T, required, tableLabel)
missing = required(~ismember(required, T.Properties.VariableNames));
if ~isempty(missing)
    error('%s missing required columns: %s', tableLabel, strjoin(missing, ', '));
end
end

function ensureDirectory(dirPath)
if isfolder(dirPath)
    return;
end
[ok, msg] = mkdir(dirPath);
if ~ok && ~isfolder(dirPath)
    error('Could not create output directory %s: %s', dirPath, msg);
end
end

function a = wrapAnglePiLocal(a)
a = mod(a + pi, 2 * pi) - pi;
end
