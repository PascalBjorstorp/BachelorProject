function plot_pipeline_monitor_results(csvDir)
% Plot pipeline latency and high-rate system usage CSV outputs.
% Usage:
%   plot_pipeline_monitor_results
%   plot_pipeline_monitor_results('/path/to/csv')
%
% The script looks for the newest files matching:
%   pipeline_latency_*.csv
%   system_usage_highrate_*.csv

if nargin < 1 || isempty(csvDir)
    scriptDir = fileparts(mfilename('fullpath'));
    csvDir = fullfile(scriptDir, 'csv');
end

plotsDir = fullfile(fileparts(mfilename('fullpath')), 'plots');
if ~exist(plotsDir, 'dir')
    mkdir(plotsDir);
end

pipelineFile = latestFile(csvDir, 'pipeline_latency_*.csv');
systemFile = latestFile(csvDir, 'system_usage_highrate_*.csv');

fprintf('Using pipeline CSV: %s\n', pipelineFile);
fprintf('Using system CSV:   %s\n', systemFile);

Tp = readtable(pipelineFile, 'VariableNamingRule', 'preserve');
Ts = readtable(systemFile, 'VariableNamingRule', 'preserve');

requiredPipeline = {'wall_time_ns','scan_to_scan_walls_ms','walls_to_amcl_ms','amcl_to_ekf_ms','scan_to_ekf_ms'};
for i = 1:numel(requiredPipeline)
    if ~ismember(requiredPipeline{i}, Tp.Properties.VariableNames)
        error('Pipeline CSV is missing required column: %s', requiredPipeline{i});
    end
end

requiredSystem = {'monotonic_time_ns','gpu_percent'};
for i = 1:numel(requiredSystem)
    if ~ismember(requiredSystem{i}, Ts.Properties.VariableNames)
        error('System CSV is missing required column: %s', requiredSystem{i});
    end
end

% Time vectors (seconds from start)
tPipe = (double(Tp.wall_time_ns) - double(Tp.wall_time_ns(1))) * 1e-9;
tSys = (double(Ts.monotonic_time_ns) - double(Ts.monotonic_time_ns(1))) * 1e-9;

% Plot cleaning settings
warmupSeconds = 3.0;      % Ignore startup transient pairing/matching
maxLatencyMs = 200.0;     % Clip unrealistic spikes for visualization

% Optional columns
hasE2D = ismember('ekf_to_drive_ms', Tp.Properties.VariableNames);
hasS2D = ismember('scan_to_drive_ms', Tp.Properties.VariableNames);

if hasE2D
    e2d = double(Tp.ekf_to_drive_ms);
else
    e2d = nan(height(Tp), 1);
end

if hasS2D
    s2d = double(Tp.scan_to_drive_ms);
else
    s2d = nan(height(Tp), 1);
end

% Replace sentinel -1 with NaN for plotting
e2d(e2d < 0) = NaN;
s2d(s2d < 0) = NaN;

scan2walls = double(Tp.scan_to_scan_walls_ms);
walls2amcl = double(Tp.walls_to_amcl_ms);
amcl2ekf = double(Tp.amcl_to_ekf_ms);
scan2ekf = double(Tp.scan_to_ekf_ms);

% Clean startup and extreme outliers for drive-related metrics.
e2dPlot = e2d;
s2dPlot = s2d;
e2dPlot(tPipe < warmupSeconds) = NaN;
s2dPlot(tPipe < warmupSeconds) = NaN;
e2dPlot(e2dPlot > maxLatencyMs) = NaN;
s2dPlot(s2dPlot > maxLatencyMs) = NaN;

%% Figure 1: Pipeline stage latency over time
f1 = figure('Name', 'Pipeline Latency Over Time', 'Color', 'w');
tiledlayout(3,2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plotLatencyTrace(tPipe, scan2walls, 'scan->scan_walls', [0.0 0.45 0.74]);

nexttile;
plotLatencyTrace(tPipe, walls2amcl, 'scan_walls->amcl', [0.85 0.33 0.10]);

nexttile;
plotLatencyTrace(tPipe, amcl2ekf, 'amcl->ekf', [0.93 0.69 0.13]);

nexttile;
plotLatencyTrace(tPipe, scan2ekf, 'scan->ekf', [0.49 0.18 0.56]);

nexttile;
if hasE2D
    plotLatencyTrace(tPipe, e2dPlot, 'ekf->drive', [0.47 0.67 0.19]);
else
    axis off; text(0.1,0.5,'No ekf->drive column','FontSize',11);
end

nexttile;
if hasS2D
    plotLatencyTrace(tPipe, s2dPlot, 'scan->drive', [0.30 0.75 0.93]);
else
    axis off; text(0.1,0.5,'No scan->drive column','FontSize',11);
end

%% Figure 2: End-to-end histogram
if hasS2D
    s2dValid = s2dPlot(~isnan(s2dPlot));
else
    s2dValid = [];
end

if isempty(s2dValid)
    s2dValid = double(Tp.scan_to_ekf_ms);
    histTitle = 'Histogram: scan->ekf';
    xlab = 'scan->ekf [ms]';
else
    histTitle = 'Histogram: scan->drive';
    xlab = 'scan->drive [ms]';
end

f2 = figure('Name', 'Latency Histogram', 'Color', 'w');
histogram(s2dValid, 80);
grid on;
title(histTitle);
xlabel(xlab);
ylabel('Count');
if ~isempty(s2dValid)
    xlim([0, max(5, prctile(s2dValid, 99.5))]);
end

%% Figure 3: CPU and GPU over time
cpuCols = startsWith(Ts.Properties.VariableNames, 'cpu_core_') & endsWith(Ts.Properties.VariableNames, '_percent');
cpuNames = Ts.Properties.VariableNames(cpuCols);

f3 = figure('Name', 'CPU GPU Usage', 'Color', 'w');
tiledlayout(max(2, numel(cpuNames) + 2), 1);

if ~isempty(cpuNames)
    cpuMat = zeros(height(Ts), numel(cpuNames));
    for i = 1:numel(cpuNames)
        cpuMat(:, i) = double(Ts.(cpuNames{i}));
    end
    for i = 1:numel(cpuNames)
        nexttile;
        plot(tSys, cpuMat(:, i), 'LineWidth', 0.8);
        ylabel('CPU [%]');
        title(strrep(cpuNames{i}, '_', '\_'));
        ylim([0, 100]);
        grid on;
    end

    nexttile;
    plot(tSys, mean(cpuMat,2), 'k', 'LineWidth', 1.6);
    ylabel('CPU [%]');
    title('cpu\_mean');
    ylim([0, 100]);
    grid on;
else
    nexttile;
    plot(tSys, nan(size(tSys)));
    title('No cpu_core_* columns found');
    grid on;
end

nexttile;
plot(tSys, Ts.gpu_percent, 'm', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('GPU [%]');
title('GPU Usage');

%% Figure 4: Correlate system load with latency
f4 = figure('Name', 'Latency vs System Load', 'Color', 'w');
if ~isempty(cpuNames) && hasS2D && any(~isnan(s2dPlot))
    cpuMat = zeros(height(Ts), numel(cpuNames));
    for i = 1:numel(cpuNames)
        cpuMat(:, i) = double(Ts.(cpuNames{i}));
    end
    cpuMeanSys = mean(cpuMat, 2);

    cpuAtPipe = interp1(tSys, cpuMeanSys, tPipe, 'linear', 'extrap');
    gpuAtPipe = interp1(tSys, double(Ts.gpu_percent), tPipe, 'linear', 'extrap');

    valid = ~isnan(s2dPlot);
    yyaxis left;
    scatter(cpuAtPipe(valid), s2dPlot(valid), 12, 'filled');
    ylabel('scan->drive latency [ms]');
    xlabel('CPU mean [%]');
    grid on;

    yyaxis right;
    scatter(gpuAtPipe(valid), s2dPlot(valid), 12, 'filled');
    ylabel('scan->drive latency [ms]');
    title('scan->drive latency vs CPU/GPU');
else
    axis off;
    text(0.1, 0.5, 'Not enough data for correlation plot', 'FontSize', 12);
end

%% Figure 5: Boxplot with six timing metrics
f5 = figure('Name', 'Latency Boxplot', 'Color', 'w');

b1 = clipForBoxplot(scan2walls(isfinite(scan2walls) & scan2walls >= 0));
b2 = clipForBoxplot(walls2amcl(isfinite(walls2amcl) & walls2amcl >= 0));
b3 = clipForBoxplot(amcl2ekf(isfinite(amcl2ekf) & amcl2ekf >= 0));
b4 = clipForBoxplot(scan2ekf(isfinite(scan2ekf) & scan2ekf >= 0));
b5 = clipForBoxplot(e2dPlot(isfinite(e2dPlot) & e2dPlot >= 0));
b6 = clipForBoxplot(s2dPlot(isfinite(s2dPlot) & s2dPlot >= 0));

allVals = [b1; b2; b3; b4; b5; b6];
grp = [
    repmat({'scan->scan_walls'}, numel(b1), 1);
    repmat({'scan_walls->amcl'}, numel(b2), 1);
    repmat({'amcl->ekf'}, numel(b3), 1);
    repmat({'scan->ekf'}, numel(b4), 1);
    repmat({'ekf->drive'}, numel(b5), 1);
    repmat({'scan->drive'}, numel(b6), 1);
    ];

if isempty(allVals)
    axis off;
    text(0.1, 0.5, 'No data available for boxplot', 'FontSize', 12);
else
    boxplot(allVals, grp, 'Whisker', 1.5, 'Symbol', '');
    ylabel('Latency [ms]');
    title('Latency Distribution by Stage');
    grid on;
    xtickangle(20);

    yTop = max(5, prctile(allVals, 99.5) * 1.2);
    ylim([0, yTop]);
end

%% Stats summary
fprintf('\nLatency summary\n');
printStats('scan->scan_walls', scan2walls);
printStats('scan_walls->amcl', walls2amcl);
printStats('amcl->ekf', amcl2ekf);
printStats('scan->ekf', scan2ekf);
if hasE2D
    printStats('ekf->drive', e2dPlot);
end
if hasS2D
    printStats('scan->drive', s2dPlot);
end

fprintf('\nPlots are shown interactively and not saved to disk.\n');
end

function fp = latestFile(dirPath, pattern)
files = dir(fullfile(dirPath, pattern));
if isempty(files)
    error('No files found in %s matching %s', dirPath, pattern);
end
[~, idx] = max([files.datenum]);
fp = fullfile(dirPath, files(idx).name);
end

function printStats(name, data)
data = double(data);
data = data(~isnan(data));
if isempty(data)
    fprintf('%-18s : no data\n', name);
    return;
end
p95 = prctile(data, 95);
p99 = prctile(data, 99);
fprintf('%-18s : mean=%8.3f ms   std=%8.3f   p95=%8.3f   p99=%8.3f\n', ...
    name, mean(data), std(data), p95, p99);
end

function plotLatencyTrace(t, y, titleText, color)
y = double(y);
valid = isfinite(y) & y >= 0;
t = t(valid);
y = y(valid);

if isempty(y)
    axis off;
    text(0.1, 0.5, ['No data for ' titleText], 'FontSize', 11);
    return;
end

function yOut = clipForBoxplot(yIn)
yIn = double(yIn(:));
yIn = yIn(isfinite(yIn));
if isempty(yIn)
    yOut = yIn;
    return;
end
lo = prctile(yIn, 0.5);
hi = prctile(yIn, 99.5);
yOut = yIn(yIn >= lo & yIn <= hi);
end

% Plot sampled raw points for density perception without overdraw.
ds = max(1, floor(numel(y) / 4000));
idx = 1:ds:numel(y);
plot(t(idx), y(idx), '.', 'Color', [color 0.25], 'MarkerSize', 4); hold on;

% Overlay robust trend line.
win = max(11, floor(numel(y) * 0.01));
if mod(win,2) == 0
    win = win + 1;
end
yTrend = movmedian(y, win);
plot(t, yTrend, '-', 'Color', color, 'LineWidth', 1.6);
hold off;

yl = prctile(y, [1 99]);
yTop = max(yl(2) * 1.15, yl(1) + 1.0);
yBot = max(0, yl(1) - 0.2);
ylim([yBot, yTop]);
grid on;
xlabel('Time [s]');
ylabel('Latency [ms]');
title(titleText);
end
