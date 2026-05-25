clc;
close all;
% Aggregate Monitor folders from NormalMap benchmark runs.
%
% Optional workspace inputs before pressing Run:
%   normalMapRoot - default repo/bags/NormalMap
%   plotsRootDir  - default Matlab/plots/NormalMapReports/MonitorComparison
%   showPlots     - true to keep figures open, false to save only
%   runConfigs    - struct array with name, group, monitorDir

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

repoRoot = fileparts(fileparts(fileparts(scriptDir)));
defaultNormalMapRoot = fullfile(repoRoot, 'bags', 'NormalMap');
if ~exist('normalMapRoot', 'var') || isempty(normalMapRoot)
    normalMapRoot = defaultNormalMapRoot;
end
if ~exist('plotsRootDir', 'var') || isempty(plotsRootDir)
    plotsRootDir = fullfile(scriptDir, 'plots', 'NormalMapReports', 'MonitorComparison');
end
if ~exist('showPlots', 'var') || isempty(showPlots)
    showPlots = false;
end
if ~exist('runConfigs', 'var') || isempty(runConfigs)
    runConfigs = defaultNormalMapMonitorConfigs(normalMapRoot);
end
racelineCsv = findDefaultRacelineCsv(fullfile(normalMapRoot, 'Raceline'));

ensureDirectoryLocal(plotsRootDir);

fprintf('NormalMap root : %s\n', normalMapRoot);
fprintf('Output folder  : %s\n', plotsRootDir);
fprintf('Monitor runs   : %d\n', numel(runConfigs));

runInfos = struct('name', {}, 'group', {}, 'monitorDir', {}, 'data', {});
failedRuns = {};

for i = 1:numel(runConfigs)
    cfg = runConfigs(i);
    fprintf('\n[%d/%d] %s\n', i, numel(runConfigs), cfg.name);
    fprintf('[%d/%d] Monitor: %s\n', i, numel(runConfigs), cfg.monitorDir);

    if ~isfolder(cfg.monitorDir)
        warning('Monitor folder missing: %s', cfg.monitorDir);
        failedRuns(end + 1, :) = {cfg.name, 'missing monitor folder'}; %#ok<SAGROW>
        continue;
    end

    try
        data = loadPipelineMonitorData(cfg.monitorDir, cfg.monitorDir);
        [lapWindowNs, windowLabel] = monitorLapWindowNs(cfg, racelineCsv);
        if all(isfinite(lapWindowNs))
            data = filterMonitorDataByTime(data, lapWindowNs(1), lapWindowNs(2));
            fprintf('[%d/%d] Window : %s\n', i, numel(runConfigs), windowLabel);
        end
        runInfos(end + 1).name = cfg.name; %#ok<SAGROW>
        runInfos(end).group = cfg.group;
        runInfos(end).monitorDir = cfg.monitorDir;
        runInfos(end).data = data;
    catch ME
        warning('Failed to load %s: %s', cfg.name, ME.message);
        failedRuns(end + 1, :) = {cfg.name, ME.message}; %#ok<SAGROW>
    end
end

if isempty(runInfos)
    error('No Monitor folders could be loaded.');
end

wideSummaryPath = writeWideSummary(runInfos, plotsRootDir);
systemSummaryPath = writeSystemSummary(runInfos, plotsRootDir);
latencySummaryPath = writeLatencySummary(runInfos, plotsRootDir);

plotSystemComparison(runInfos, plotsRootDir, showPlots);
plotLatencyComparison(runInfos, plotsRootDir, showPlots);

fprintf('\nWide summary    : %s\n', wideSummaryPath);
fprintf('System summary  : %s\n', systemSummaryPath);
fprintf('Latency summary : %s\n', latencySummaryPath);
fprintf('Plots saved to  : %s\n', plotsRootDir);

if ~isempty(failedRuns)
    fprintf('Failed monitor folder(s):\n');
    for i = 1:size(failedRuns, 1)
        fprintf('  %s: %s\n', failedRuns{i, 1}, failedRuns{i, 2});
    end
end

function configs = defaultNormalMapMonitorConfigs(normalMapRoot)
configs = struct('name', {}, 'group', {}, 'monitorDir', {}, 'bagDir', {}, 'applyLapWindow', {});
configs(end + 1) = monitorConfig('MCL CloudPublish OFF', 'MCL', ...
    fullfile(normalMapRoot, 'CloudPublishTest', 'PublishRateOff', 'Monitor'), ...
    fullfile(normalMapRoot, 'CloudPublishTest', 'PublishRateOff', 'LateralPlanner_20260519_092514'), true);
configs(end + 1) = monitorConfig('MCL CloudPublish ON', 'MCL', ...
    fullfile(normalMapRoot, 'CloudPublishTest', 'PublishRateON', 'Monitor'), ...
    fullfile(normalMapRoot, 'CloudPublishTest', 'PublishRateON'), true);
configs(end + 1) = monitorConfig('AMCL AdaptiveParticles', 'AMCL', ...
    fullfile(normalMapRoot, 'AdaptiveParticles', 'MPC_10Laps', 'Monitor'), ...
    fullfile(normalMapRoot, 'AdaptiveParticles', 'MPC_10Laps'), true);
configs(end + 1) = monitorConfig('Speed FPGA ROS2', 'Speed', ...
    fullfile(normalMapRoot, 'SpeedBags', 'FPGA_ROS2', 'Monitor'), ...
    fullfile(normalMapRoot, 'SpeedBags', 'FPGA_ROS2'), true);
configs(end + 1) = monitorConfig('Speed FPGA UDP', 'Speed', ...
    fullfile(normalMapRoot, 'SpeedBags', 'FPGA_UDP', 'Monitor'), ...
    fullfile(normalMapRoot, 'SpeedBags', 'FPGA_UDP'), true);
configs(end + 1) = monitorConfig('Nav2 Converge', 'Nav2', ...
    fullfile(normalMapRoot, 'Nav2Converge', 'Monitor'), ...
    fullfile(normalMapRoot, 'Nav2Converge'), false);
end

function cfg = monitorConfig(name, group, monitorDir, bagDir, applyLapWindow)
cfg = struct('name', name, 'group', group, 'monitorDir', monitorDir, ...
    'bagDir', bagDir, 'applyLapWindow', applyLapWindow);
end

function path = writeWideSummary(runInfos, outputDir)
n = numel(runInfos);
name = strings(n, 1);
group = strings(n, 1);
cpu_short_mean_percent = NaN(n, 1);
cpu_short_p95_percent = NaN(n, 1);
cpu_long_mean_percent = NaN(n, 1);
cpu_long_p95_percent = NaN(n, 1);
gpu_mean_percent = NaN(n, 1);
gpu_p95_percent = NaN(n, 1);
scan_to_amcl_mean_ms = NaN(n, 1);
scan_to_amcl_p95_ms = NaN(n, 1);
scan_to_ekf_mean_ms = NaN(n, 1);
scan_to_ekf_p95_ms = NaN(n, 1);
scan_to_ackermann_mean_ms = NaN(n, 1);
scan_to_ackermann_p95_ms = NaN(n, 1);

for i = 1:n
    data = runInfos(i).data;
    name(i) = string(runInfos(i).name);
    group(i) = string(runInfos(i).group);

    cpuShort = systemValues(data, 'cpu_short_window_percent');
    cpuLong = systemValues(data, 'cpu_long_window_percent');
    gpu = systemValues(data, 'gpu_monitor_percent');
    s2amcl = latencyValues(data, 'scan2amcl');
    s2ekf = latencyValues(data, 'scan2ekf');
    s2ack = latencyValues(data, 's2a');

    cpu_short_mean_percent(i) = mean(cpuShort, 'omitnan');
    cpu_short_p95_percent(i) = percentileLocal(cpuShort, 95);
    cpu_long_mean_percent(i) = mean(cpuLong, 'omitnan');
    cpu_long_p95_percent(i) = percentileLocal(cpuLong, 95);
    gpu_mean_percent(i) = mean(gpu, 'omitnan');
    gpu_p95_percent(i) = percentileLocal(gpu, 95);
    scan_to_amcl_mean_ms(i) = mean(s2amcl, 'omitnan');
    scan_to_amcl_p95_ms(i) = percentileLocal(s2amcl, 95);
    scan_to_ekf_mean_ms(i) = mean(s2ekf, 'omitnan');
    scan_to_ekf_p95_ms(i) = percentileLocal(s2ekf, 95);
    scan_to_ackermann_mean_ms(i) = mean(s2ack, 'omitnan');
    scan_to_ackermann_p95_ms(i) = percentileLocal(s2ack, 95);
end

T = table(name, group, cpu_short_mean_percent, cpu_short_p95_percent, ...
    cpu_long_mean_percent, cpu_long_p95_percent, gpu_mean_percent, gpu_p95_percent, ...
    scan_to_amcl_mean_ms, scan_to_amcl_p95_ms, scan_to_ekf_mean_ms, scan_to_ekf_p95_ms, ...
    scan_to_ackermann_mean_ms, scan_to_ackermann_p95_ms);

path = fullfile(outputDir, 'NormalMap_Monitor_Wide_Summary.csv');
writetable(T, path);
disp(' ');
disp('=== NormalMap Monitor Wide Summary ===');
disp(T);
end

function path = writeSystemSummary(runInfos, outputDir)
sources = {'cpu_short_window_percent', 'cpu_long_window_percent', ...
    'gpu_monitor_percent', 'gpu_short_window_percent', 'gpu_long_window_percent'};
rows = [];

for i = 1:numel(runInfos)
    for j = 1:numel(sources)
        values = systemValues(runInfos(i).data, sources{j});
        row = table(string(runInfos(i).name), string(runInfos(i).group), string(sources{j}), ...
            nnz(isfinite(values)), mean(values, 'omitnan'), median(values, 'omitnan'), ...
            percentileLocal(values, 95), maxFiniteLocal(values), ...
            'VariableNames', {'name', 'group', 'source', 'n_samples', ...
            'mean_percent', 'median_percent', 'p95_percent', 'max_percent'});
        if isempty(rows)
            rows = row;
        else
            rows = [rows; row]; %#ok<AGROW>
        end
    end
end

path = fullfile(outputDir, 'NormalMap_System_Usage_Summary.csv');
writetable(rows, path);
end

function path = writeLatencySummary(runInfos, outputDir)
metrics = {'scanStamp2Scan', 'scan2amcl', 'amcl2ekf', 'scan2ekf', 'e2d', 'd2a', 's2a'};
labels = {'scan_stamp_to_scan_ms', 'scan_to_amcl_ms', 'amcl_to_ekf_ms', ...
    'scan_to_ekf_ms', 'ekf_to_drive_ms', 'drive_to_ackermann_ms', 'scan_to_ackermann_ms'};
rows = [];

for i = 1:numel(runInfos)
    for j = 1:numel(metrics)
        values = latencyValues(runInfos(i).data, metrics{j});
        row = table(string(runInfos(i).name), string(runInfos(i).group), string(labels{j}), ...
            nnz(isfinite(values)), mean(values, 'omitnan'), var(values, 0, 'omitnan'), ...
            percentileLocal(values, 95), maxFiniteLocal(values), ...
            'VariableNames', {'name', 'group', 'stage', 'n_samples', ...
            'mean_ms', 'var_ms2', 'p95_ms', 'max_ms'});
        if isempty(rows)
            rows = row;
        else
            rows = [rows; row]; %#ok<AGROW>
        end
    end
end

path = fullfile(outputDir, 'NormalMap_Pipeline_Latency_Summary.csv');
writetable(rows, path);
end

function plotSystemComparison(runInfos, outputDir, showPlot)
labels = string({runInfos.name});

fig = makePlotFigure('NormalMap CPU/GPU Comparison', showPlot);
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plotBoxByRun(runInfos, labels, 'cpu_short_window_percent');
grid on;
ylabel('CPU [%]');
title('CPU short-window usage distribution');
xtickangle(20);

nexttile;
plotBoxByRun(runInfos, labels, 'gpu_monitor_percent');
grid on;
ylabel('GPU [%]');
title('GPU usage distribution');
xtickangle(20);

savePlotFigure(fig, outputDir, 'NormalMap_CPU_GPU_Comparison', showPlot);
end

function plotLatencyComparison(runInfos, outputDir, showPlot)
labels = string({runInfos.name});

fig = makePlotFigure('NormalMap Latency Comparison', showPlot);
plotBoxByRun(runInfos, labels, 'scan2amcl');
grid on;
ylabel('scan->AMCL latency [ms]');
title('Scan to AMCL latency distribution');
xtickangle(20);

savePlotFigure(fig, outputDir, 'NormalMap_Latency_Comparison', showPlot);
end

function plotBoxByRun(runInfos, labels, sourceName)
valuesAll = [];
groupsAll = [];
for i = 1:numel(runInfos)
    if startsWith(char(sourceName), 'cpu') || startsWith(char(sourceName), 'gpu')
        values = systemValues(runInfos(i).data, sourceName);
    else
        values = latencyValues(runInfos(i).data, sourceName);
    end
    values = values(isfinite(values));
    valuesAll = [valuesAll; values(:)]; %#ok<AGROW>
    groupsAll = [groupsAll; repmat(i, numel(values), 1)]; %#ok<AGROW>
end

if isempty(valuesAll)
    axis off;
    text(0.1, 0.5, 'No samples found', 'FontSize', 12);
    return;
end

boxchart(groupsAll, valuesAll, 'BoxFaceColor', [0.00, 0.45, 0.74], ...
    'MarkerStyle', '.', 'MarkerColor', [0.30, 0.30, 0.30]);
set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'TickLabelInterpreter', 'none');
end

function values = systemValues(data, sourceName)
values = NaN(0, 1);
if ~isstruct(data)
    return;
end

switch char(sourceName)
    case 'cpu_short_window_percent'
        if isfield(data, 'cpu') && isfield(data.cpu, 'hasShort') && data.cpu.hasShort
            values = data.cpu.short;
        end
    case 'cpu_long_window_percent'
        if isfield(data, 'cpu') && isfield(data.cpu, 'hasLong') && data.cpu.hasLong
            values = data.cpu.long;
        end
    case 'gpu_monitor_percent'
        if isfield(data, 'gpu') && isfield(data.gpu, 'hasFile') && data.gpu.hasFile
            values = data.gpu.standalone;
        elseif isfield(data, 'gpu') && isfield(data.gpu, 'hasShort') && data.gpu.hasShort
            values = data.gpu.short;
        elseif isfield(data, 'gpu') && isfield(data.gpu, 'hasLong') && data.gpu.hasLong
            values = data.gpu.long;
        end
    case 'gpu_short_window_percent'
        if isfield(data, 'gpu') && isfield(data.gpu, 'hasShort') && data.gpu.hasShort
            values = data.gpu.short;
        end
    case 'gpu_long_window_percent'
        if isfield(data, 'gpu') && isfield(data.gpu, 'hasLong') && data.gpu.hasLong
            values = data.gpu.long;
        end
end

values = values(:);
values = values(isfinite(values) & values >= 0 & values <= 100);
end

function values = latencyValues(data, metricName)
values = NaN(0, 1);
if ~isstruct(data) || ~isfield(data, 'pipeline') || ...
        ~isfield(data.pipeline, 'hasData') || ~data.pipeline.hasData
    return;
end

p = data.pipeline;
switch char(metricName)
    case 'scanStamp2Scan'
        if isfield(p, 'hasScanStamp2Scan') && p.hasScanStamp2Scan
            values = p.scanStamp2Scan;
        end
    case 'scan2amcl'
        values = p.scan2amcl;
    case 'amcl2ekf'
        values = p.amcl2ekf;
    case 'scan2ekf'
        values = p.scan2ekf;
    case 'e2d'
        if isfield(p, 'hasE2D') && p.hasE2D
            values = p.e2d;
        end
    case 'd2a'
        if isfield(p, 'hasD2A') && p.hasD2A
            values = p.d2a;
        end
    case 's2a'
        if isfield(p, 'hasS2A') && p.hasS2A
            values = p.s2a;
        end
end

values = values(:);
values = values(isfinite(values) & values >= 0);
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

function ensureDirectoryLocal(dirPath)
if isfolder(dirPath)
    return;
end
[ok, msg] = mkdir(dirPath);
if ~ok && ~isfolder(dirPath)
    error('Could not create output directory %s: %s', dirPath, msg);
end
end

function [windowNs, label] = monitorLapWindowNs(cfg, racelineCsv)
windowNs = [NaN, NaN];
label = 'full CSV';
if ~isfield(cfg, 'applyLapWindow') || ~cfg.applyLapWindow || ...
        ~isfield(cfg, 'bagDir') || isempty(cfg.bagDir) || ~isfolder(cfg.bagDir)
    return;
end
if isempty(racelineCsv) || ~isfile(racelineCsv)
    warning('Lap filter skipped for %s: raceline CSV missing', cfg.name);
    return;
end

try
    racelineXY = loadRacelineXYLocal(racelineCsv);
    [startS, endS, lapCount] = completedLapWindowFromBag(cfg.bagDir, racelineXY);
    if isfinite(startS) && isfinite(endS) && endS > startS
        windowNs = [startS, endS] * 1e9;
        label = sprintf('skip startup lap, keep %d completed lap(s), %.3f to %.3f s', ...
            max(0, lapCount - 1), startS, endS);
    end
catch ME
    warning('Lap filter skipped for %s: %s', cfg.name, ME.message);
end
end

function [startS, endS, lapCount] = completedLapWindowFromBag(bagDir, racelineXY)
startS = NaN;
endS = NaN;
lapCount = 0;
bag = ros2bagreader(bagDir);
allTopics = getTopicNamesLocal(bag.AvailableTopics);
ekfTopic = pickTopicLocal(allTopics, '/ekf_pose', 'ekf_pose');
if isempty(ekfTopic)
    error('No EKF pose topic found in %s', bagDir);
end

ekfMsgs = readMessages(select(bag, 'Topic', ekfTopic));
[tEkf, xyEkf] = extractXYSeriesLocal(ekfMsgs);
[~, crossingTimes] = detectFullLapDurationsLocal(tEkf, xyEkf(:, 1), xyEkf(:, 2), racelineXY, 5.0);
lapCount = max(0, numel(crossingTimes) - 1);
if numel(crossingTimes) < 3
    error('Not enough full lap crossings in %s', bagDir);
end
startS = crossingTimes(2);
endS = crossingTimes(end);
end

function data = filterMonitorDataByTime(data, startNs, endNs)
if isfield(data, 'pipeline') && isfield(data.pipeline, 'hasData') && data.pipeline.hasData && ...
        isfield(data.pipeline, 'timeNs')
    mask = data.pipeline.timeNs >= startNs & data.pipeline.timeNs <= endNs;
    data.pipeline = filterPipelineStruct(data.pipeline, mask);
end

if isfield(data, 'cpu')
    if isfield(data.cpu, 'hasShort') && data.cpu.hasShort && isfield(data.cpu, 'timeNsShort')
        mask = data.cpu.timeNsShort >= startNs & data.cpu.timeNsShort <= endNs;
        data.cpu.short = data.cpu.short(mask);
        data.cpu.timeNsShort = data.cpu.timeNsShort(mask);
        data.cpu.tShort = relativeTimeSecondsLocal(data.cpu.timeNsShort);
        if isfield(data.gpu, 'hasShort') && data.gpu.hasShort
            data.gpu.short = data.gpu.short(mask);
        end
    end
    if isfield(data.cpu, 'hasLong') && data.cpu.hasLong && isfield(data.cpu, 'timeNsLong')
        mask = data.cpu.timeNsLong >= startNs & data.cpu.timeNsLong <= endNs;
        data.cpu.long = data.cpu.long(mask);
        data.cpu.timeNsLong = data.cpu.timeNsLong(mask);
        data.cpu.tLong = relativeTimeSecondsLocal(data.cpu.timeNsLong);
        if isfield(data.gpu, 'hasLong') && data.gpu.hasLong
            data.gpu.long = data.gpu.long(mask);
        end
    end
end

if isfield(data, 'gpu') && isfield(data.gpu, 'hasFile') && data.gpu.hasFile && isfield(data.gpu, 'timeNs')
    mask = data.gpu.timeNs >= startNs & data.gpu.timeNs <= endNs;
    data.gpu.standalone = data.gpu.standalone(mask);
    data.gpu.timeNs = data.gpu.timeNs(mask);
    data.gpu.t = relativeTimeSecondsLocal(data.gpu.timeNs);
end

if isfield(data, 'perCore') && isfield(data.perCore, 'hasData') && data.perCore.hasData && ...
        isfield(data.perCore, 'timeNs')
    mask = data.perCore.timeNs >= startNs & data.perCore.timeNs <= endNs;
    data.perCore.cpu = data.perCore.cpu(mask, :);
    data.perCore.timeNs = data.perCore.timeNs(mask);
    data.perCore.t = relativeTimeSecondsLocal(data.perCore.timeNs);
end

if isfield(data, 'node') && isfield(data.node, 'hasData') && data.node.hasData && ...
        isfield(data.node, 'timeNs')
    mask = data.node.timeNs >= startNs & data.node.timeNs <= endNs;
    data.node.table = data.node.table(mask, :);
    data.node.timeNs = data.node.timeNs(mask);
end
end

function pipeline = filterPipelineStruct(pipeline, mask)
fields = {'timeNs', 't', 'scanStamp2Scan', 'scan2amcl', 'amcl2ekf', ...
    'scan2ekf', 'e2d', 'd2a', 's2a'};
for i = 1:numel(fields)
    f = fields{i};
    if isfield(pipeline, f) && numel(pipeline.(f)) == numel(mask)
        pipeline.(f) = pipeline.(f)(mask);
    end
end
if isfield(pipeline, 'table') && height(pipeline.table) == numel(mask)
    pipeline.table = pipeline.table(mask, :);
end
if isfield(pipeline, 'timeNs') && ~isempty(pipeline.timeNs)
    pipeline.t = relativeTimeSecondsLocal(pipeline.timeNs);
end
end

function t = relativeTimeSecondsLocal(timeNs)
timeNs = double(timeNs(:));
if isempty(timeNs)
    t = zeros(0, 1);
    return;
end
t = (timeNs - timeNs(1)) * 1e-9;
end

function csvPath = findDefaultRacelineCsv(racelineRoot)
csvPath = '';
preferred = fullfile(racelineRoot, 'my_track_raceline.csv');
if isfile(preferred)
    csvPath = preferred;
    return;
end
candidates = dir(fullfile(racelineRoot, '*.csv'));
if ~isempty(candidates)
    [~, idx] = max([candidates.datenum]);
    csvPath = fullfile(candidates(idx).folder, candidates(idx).name);
end
end

function names = getTopicNamesLocal(topicsTbl)
vars = topicsTbl.Properties.VariableNames;
if ismember("TopicName", vars)
    names = string(topicsTbl.TopicName);
elseif ismember("Name", vars)
    names = string(topicsTbl.Name);
elseif ismember("Topic", vars)
    names = string(topicsTbl.Topic);
else
    rowNames = topicsTbl.Properties.RowNames;
    if isempty(rowNames)
        error('Unknown AvailableTopics table format.');
    end
    names = string(rowNames);
end
end

function topic = pickTopicLocal(allTopics, preferred, fallbackContains)
topic = '';
if any(strcmp(allTopics, preferred))
    topic = char(preferred);
    return;
end
idx = contains(lower(allTopics), lower(fallbackContains));
if any(idx)
    topic = char(allTopics(find(idx, 1, 'first')));
end
end

function [t, xy] = extractXYSeriesLocal(msgs)
n = numel(msgs);
t = zeros(n, 1);
xy = nan(n, 2);
for k = 1:n
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end
    t(k) = extractHeaderTimeLocal(m, k);
    [x, y, ok] = extractPositionXYLocal(m);
    if ok
        xy(k, :) = [x, y];
    end
end
valid = isfinite(t) & isfinite(xy(:, 1)) & isfinite(xy(:, 2));
t = t(valid);
xy = xy(valid, :);
[t, idx] = sort(t);
xy = xy(idx, :);
[t, iUnique] = unique(t);
xy = xy(iUnique, :);
end

function t = extractHeaderTimeLocal(m, fallback)
t = fallback;
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

function [x, y, ok] = extractPositionXYLocal(m)
ok = false;
x = NaN;
y = NaN;
[poseField, hasPose] = getFieldIgnoreCaseLocal(m, 'pose');
poseStruct = [];
if hasPose && isstruct(poseField)
    [innerPose, hasInnerPose] = getFieldIgnoreCaseLocal(poseField, 'pose');
    if hasInnerPose && isstruct(innerPose)
        poseStruct = innerPose;
    else
        poseStruct = poseField;
    end
end
[pos, hasPos] = getFieldIgnoreCaseLocal(poseStruct, 'position');
if ~hasPos
    [pos, hasPos] = getFieldIgnoreCaseLocal(m, 'position');
end
if ~hasPos || ~isstruct(pos)
    return;
end
[xv, hasX] = getNumericFieldIgnoreCaseLocal(pos, 'x');
[yv, hasY] = getNumericFieldIgnoreCaseLocal(pos, 'y');
if hasX && hasY
    x = double(xv);
    y = double(yv);
    ok = true;
end
end

function [value, found] = getFieldIgnoreCaseLocal(s, name)
value = [];
found = false;
if isempty(s) || ~isstruct(s)
    return;
end
fns = fieldnames(s);
idx = find(strcmpi(fns, name), 1, 'first');
if ~isempty(idx)
    value = s.(fns{idx});
    found = true;
end
end

function [value, found] = getNumericFieldIgnoreCaseLocal(s, name)
[v, found] = getFieldIgnoreCaseLocal(s, name);
if found
    value = double(v);
else
    value = NaN;
end
end

function xy = loadRacelineXYLocal(csvPath)
T = readtable(csvPath, 'VariableNamingRule', 'preserve');
names = string(T.Properties.VariableNames);
low = lower(names);
idxX = find(contains(low, "x"), 1, 'first');
idxY = find(contains(low, "y"), 1, 'first');
if isempty(idxX) || isempty(idxY) || idxX == idxY
    numIdx = find(varfun(@isnumeric, T, 'OutputFormat', 'uniform'));
    idxX = numIdx(1);
    idxY = numIdx(2);
end
xy = [double(T{:, idxX}), double(T{:, idxY})];
xy = xy(all(isfinite(xy), 2), :);
end

function [lapDurS, crossingTimes] = detectFullLapDurationsLocal(t, x, y, racelineXY, minLapTimeS)
lapDurS = [];
crossingTimes = [];
if numel(t) < 3 || isempty(racelineXY) || size(racelineXY, 1) < 2
    return;
end
[trackS, trackLength, closedRacelineXY] = buildRacelineArclengthLocal(racelineXY);
progress = projectTrajectoryToRacelineProgressLocal([x(:), y(:)], closedRacelineXY, trackS, trackLength);
valid = isfinite(t(:)) & isfinite(progress(:));
t = t(valid);
progress = progress(valid);
if numel(t) < 3 || trackLength <= 0
    return;
end
[t, sortIdx] = sort(t);
progress = progress(sortIdx);
[t, uniqueIdx] = unique(t, 'stable');
progress = progress(uniqueIdx);
unwrappedProgress = unwrap(2 * pi * progress(:) / trackLength) * trackLength / (2 * pi);
if unwrappedProgress(end) < unwrappedProgress(1)
    unwrappedProgress = -unwrappedProgress;
end
crossingTimes = debounceCrossingTimesLocal( ...
    detectProgressCrossingsLocal(t, unwrappedProgress, trackLength), minLapTimeS);
if numel(crossingTimes) >= 2
    lapDurS = diff(crossingTimes);
end
end

function [trackS, trackLength, closedXY] = buildRacelineArclengthLocal(racelineXY)
closedXY = racelineXY(all(isfinite(racelineXY), 2), :);
if hypot(closedXY(end, 1) - closedXY(1, 1), closedXY(end, 2) - closedXY(1, 2)) > 1e-6
    closedXY(end + 1, :) = closedXY(1, :);
end
segLen = hypot(diff(closedXY(:, 1)), diff(closedXY(:, 2)));
keepPoint = [true; segLen > 1e-9];
closedXY = closedXY(keepPoint, :);
segLen = hypot(diff(closedXY(:, 1)), diff(closedXY(:, 2)));
trackS = [0; cumsum(segLen)];
trackLength = trackS(end);
end

function progress = projectTrajectoryToRacelineProgressLocal(trajXY, racelineXY, trackS, trackLength)
progress = nan(size(trajXY, 1), 1);
if isempty(trackS) || trackLength <= 0 || size(racelineXY, 1) < 2
    return;
end
segStart = racelineXY(1:end-1, :);
segVec = diff(racelineXY, 1, 1);
segLen2 = sum(segVec .* segVec, 2);
validSeg = segLen2 > 1e-12;
segStart = segStart(validSeg, :);
segVec = segVec(validSeg, :);
segLen2 = segLen2(validSeg);
segS = trackS(1:end-1);
segS = segS(validSeg);
for i = 1:size(trajXY, 1)
    pt = trajXY(i, :);
    if any(~isfinite(pt))
        continue;
    end
    rel = pt - segStart;
    u = (rel(:, 1) .* segVec(:, 1) + rel(:, 2) .* segVec(:, 2)) ./ segLen2;
    u = min(max(u, 0), 1);
    proj = segStart + u .* segVec;
    dist2 = sum((proj - pt) .* (proj - pt), 2);
    [~, idx] = min(dist2);
    progress(i) = mod(segS(idx) + u(idx) * sqrt(segLen2(idx)), trackLength);
end
end

function crossingTimes = detectProgressCrossingsLocal(t, unwrappedProgress, trackLength)
crossingTimes = [];
epsProgress = max(1e-6, trackLength * 1e-9);
for i = 1:(numel(t) - 1)
    p0 = unwrappedProgress(i);
    p1 = unwrappedProgress(i + 1);
    if ~isfinite(p0) || ~isfinite(p1) || p1 <= p0 + epsProgress
        continue;
    end
    target = ceil((p0 + epsProgress) / trackLength) * trackLength;
    while target <= p1 + epsProgress
        alpha = (target - p0) / (p1 - p0);
        if alpha >= 0 && alpha <= 1
            crossingTimes(end + 1, 1) = t(i) + alpha * (t(i + 1) - t(i)); %#ok<AGROW>
        end
        target = target + trackLength;
    end
end
end

function crossingTimes = debounceCrossingTimesLocal(rawCrossings, minLapTimeS)
crossingTimes = [];
rawCrossings = sort(rawCrossings(:));
rawCrossings = rawCrossings(isfinite(rawCrossings));
for i = 1:numel(rawCrossings)
    if isempty(crossingTimes) || rawCrossings(i) - crossingTimes(end) >= minLapTimeS
        crossingTimes(end + 1, 1) = rawCrossings(i); %#ok<AGROW>
    end
end
end
