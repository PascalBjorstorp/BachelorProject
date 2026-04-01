function plot_pipeline_monitor_results(csvDir)
clc;
close all;
% Plot pipeline latency and CPU window CSV outputs.
% Usage:
%   plot_pipeline_monitor_results
%   plot_pipeline_monitor_results('/path/to/csv')
%
% The script looks for:
%   newest pipeline_latency_*.csv (fallback Pipeline_*.csv)
%   SystemUsageShort.csv (optional, includes gpu_percent when available)
%   SystemUsageLong.csv (optional, includes gpu_percent when available)
%   SystemUsagePerCore.csv (optional)

if nargin < 1 || isempty(csvDir)
    scriptDir = fileparts(mfilename('fullpath'));
    folderSelected = 'MPC_CPU';
    csvDir = fullfile(scriptDir, 'csv', folderSelected);
end


pipelineFile = latestFileAny(csvDir, {'pipeline_*.csv'});
shortCpuFile = fullfile(csvDir, 'SystemUsageShort.csv');
longCpuFile = fullfile(csvDir, 'SystemUsageLong.csv');
perCoreCpuFile = fullfile(csvDir, 'SystemUsagePerCore.csv');
gpuUsageFile = fullfile(csvDir, 'SystemUsageGpu.csv');

fprintf('Using pipeline CSV: %s\n', pipelineFile);

Tp = readtable(pipelineFile, 'VariableNamingRule', 'preserve');

requiredPipeline = {
    'wall_time_ns', ...
    'scan_to_scan_walls_ms', ...
    'walls_to_amcl_ms', ...
    'amcl_to_ekf_ms', ...
    'scan_to_ekf_ms'};
assertHasColumns(Tp, requiredPipeline, 'Pipeline CSV');

hasCpuWindows = isfile(shortCpuFile) && isfile(longCpuFile);
hasPerCoreCpu = isfile(perCoreCpuFile);
hasGpuShort = isfile(gpuUsageFile);
Tshort = table();
Tlong = table();
Tcore = table();
Tgpu = table();
shortCpu = [];
longCpu = [];
shortGpu = [];
longGpu = [];
hasGpuLong = false;
coreCpu = [];
coreNames = {};

if hasCpuWindows
    fprintf('Using short CPU CSV: %s\n', shortCpuFile);
    fprintf('Using long CPU CSV:  %s\n', longCpuFile);

    Tshort = readtable(shortCpuFile, 'VariableNamingRule', 'preserve');
    Tlong = readtable(longCpuFile, 'VariableNamingRule', 'preserve');

    assertHasColumns(Tshort, {'monotonic_time_ns','cpu_short_window_percent'}, 'Short CPU CSV');
    assertHasColumns(Tlong, {'monotonic_time_ns','cpu_long_window_percent'}, 'Long CPU CSV');

    if height(Tshort) == 0 || height(Tlong) == 0
        fprintf('CPU window CSVs are empty. Skipping CPU windows figure.\n');
        hasCpuWindows = false;
    else
        shortCpu = double(Tshort.cpu_short_window_percent);
        longCpu = double(Tlong.cpu_long_window_percent);

        hasGpuLong = ismember('gpu_percent', Tlong.Properties.VariableNames);
        if hasGpuLong
            longGpu = double(Tlong.gpu_percent);
        end
    end
else
    fprintf('CPU window CSVs not found. Expected:\n');
    fprintf('  %s\n', shortCpuFile);
    fprintf('  %s\n', longCpuFile);
    fprintf('Proceeding with pipeline latency plots only.\n');
end

if hasGpuShort
    fprintf('Using GPU CSV: %s\n', gpuUsageFile);
    Tgpu = readtable(gpuUsageFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tgpu, {'monotonic_time_ns','gpu_percent'}, 'GPU CSV');

    if height(Tgpu) == 0
        fprintf('GPU CSV is empty. Skipping GPU short-rate plots.\n');
        hasGpuShort = false;
    else
        shortGpu = double(Tgpu.gpu_percent);
    end
else
    fprintf('GPU CSV not found. Expected:\n');
    fprintf('  %s\n', gpuUsageFile);
    if hasCpuWindows && ismember('gpu_percent', Tshort.Properties.VariableNames)
        fprintf('Falling back to gpu_percent in short CPU CSV.\n');
        hasGpuShort = true;
        shortGpu = double(Tshort.gpu_percent);
    end
end

if hasPerCoreCpu
    fprintf('Using per-core CPU CSV: %s\n', perCoreCpuFile);
    Tcore = readtable(perCoreCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tcore, {'monotonic_time_ns'}, 'Per-core CPU CSV');

    coreCols = startsWith(Tcore.Properties.VariableNames, 'cpu_core_') & ...
        endsWith(Tcore.Properties.VariableNames, '_percent');
    coreNames = Tcore.Properties.VariableNames(coreCols);

    if height(Tcore) == 0 || isempty(coreNames)
        fprintf('Per-core CPU CSV is empty or missing cpu_core_*_percent columns.\n');
        hasPerCoreCpu = false;
    else
        coreCpu = zeros(height(Tcore), numel(coreNames));
        for i = 1:numel(coreNames)
            coreCpu(:, i) = double(Tcore.(coreNames{i}));
        end
    end
else
    fprintf('Per-core CPU CSV not found. Expected:\n');
    fprintf('  %s\n', perCoreCpuFile);
end

% Time vectors (seconds from start) for CPU files
if hasCpuWindows
    tShort = (double(Tshort.monotonic_time_ns) - double(Tshort.monotonic_time_ns(1))) * 1e-9;
    tLong = (double(Tlong.monotonic_time_ns) - double(Tlong.monotonic_time_ns(1))) * 1e-9;
else
    tShort = [];
    tLong = [];
end

if hasPerCoreCpu
    tCore = (double(Tcore.monotonic_time_ns) - double(Tcore.monotonic_time_ns(1))) * 1e-9;
else
    tCore = [];
end

if hasGpuShort
    if height(Tgpu) > 0
        tGpu = (double(Tgpu.monotonic_time_ns) - double(Tgpu.monotonic_time_ns(1))) * 1e-9;
    else
        tGpu = tShort;
    end
else
    tGpu = [];
end

hasPipeline = height(Tp) > 0;
if ~hasPipeline
    fprintf('Pipeline CSV has no data rows. Skipping pipeline latency plots.\n');
end

if hasPipeline
    tPipe = (double(Tp.wall_time_ns) - double(Tp.wall_time_ns(1))) * 1e-9;

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
    figure('Name', 'Pipeline Latency Over Time', 'Color', 'w');
    tiledlayout(3,2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plotLatencyTrace(tPipe, scan2walls, 'scan->scan_{walls}', [0.0 0.45 0.74]);

    nexttile;
    plotLatencyTrace(tPipe, walls2amcl, 'scan_{walls}->amcl', [0.85 0.33 0.10]);

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

    %% Figure 2: Combined histograms (scan->ekf on top, scan->drive below)
    scan2ekfValid = scan2ekf(isfinite(scan2ekf) & scan2ekf >= 0);

    if hasS2D
        s2dValid = s2dPlot(~isnan(s2dPlot));
    else
        s2dValid = [];
    end

    figure('Name', 'Latency Histograms', 'Color', 'w');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    histogram(scan2ekfValid, 80);
    grid on;
    title('Histogram: scan->ekf');
    xlabel('scan->ekf [ms]');
    ylabel('Count');
    if ~isempty(scan2ekfValid)
        xlim([0, max(5, prctile(scan2ekfValid, 99.5))]);
    end

    nexttile;
    if ~isempty(s2dValid)
        histogram(s2dValid, 80);
        grid on;
        title('Histogram: scan->drive');
        xlabel('scan->drive [ms]');
        ylabel('Count');
        xlim([0, max(5, prctile(s2dValid, 99.5))]);
    else
        axis off;
        text(0.1, 0.5, 'No scan->drive data available', 'FontSize', 11);
    end

    %% Figure 3: Boxplot with six timing metrics
    figure('Name', 'Latency Boxplot', 'Color', 'w');

    b1 = clipForBoxplot(scan2walls(isfinite(scan2walls) & scan2walls >= 0));
    b2 = clipForBoxplot(walls2amcl(isfinite(walls2amcl) & walls2amcl >= 0));
    b3 = clipForBoxplot(amcl2ekf(isfinite(amcl2ekf) & amcl2ekf >= 0));
    b4 = clipForBoxplot(scan2ekf(isfinite(scan2ekf) & scan2ekf >= 0));
    b5 = clipForBoxplot(e2dPlot(isfinite(e2dPlot) & e2dPlot >= 0));
    b6 = clipForBoxplot(s2dPlot(isfinite(s2dPlot) & s2dPlot >= 0));

    allVals = [b1; b2; b3; b4; b5; b6];
    grp = [
        repmat({'scan->scan_{walls}'}, numel(b1), 1);
        repmat({'scan_{walls}->amcl'}, numel(b2), 1);
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
end

%% Figure 4: CPU windows over time (optional)
if hasCpuWindows
    figure('Name', 'CPU Windows', 'Color', 'w');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(tShort, shortCpu, 'Color', [0.20 0.55 0.90], 'LineWidth', 0.8);
    grid on;
    ylabel('CPU [%]');
    title('Short window CPU (high-rate)');
    ylim([0, 100]);

    nexttile;
    stairs(tLong, longCpu, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('CPU [%]');
    title('Long window CPU (1 Hz)');
    ylim([0, 100]);

    % Figure 5: 25ms phase view in 5ms bins (for AMCL spike timing)
    phaseCycleSec = 0.025;
    phaseBinSec = 0.005;
    phaseSec = mod(tShort, phaseCycleSec);
    phaseEdgesSec = 0:phaseBinSec:phaseCycleSec;
    [~, ~, phaseBinIdx] = histcounts(phaseSec, phaseEdgesSec);

    nBins = numel(phaseEdgesSec) - 1;
    phaseCentersSec = phaseEdgesSec(1:end-1) + (phaseBinSec / 2);
    phaseMean = nan(nBins, 1);
    phaseP95 = nan(nBins, 1);
    phaseCount = zeros(nBins, 1);

    for b = 1:nBins
        vals = shortCpu(phaseBinIdx == b);
        vals = vals(isfinite(vals));
        phaseCount(b) = numel(vals);
        if ~isempty(vals)
            phaseMean(b) = mean(vals);
            phaseP95(b) = prctile(vals, 95);
        end
    end

    figure('Name', 'Short CPU Phase (25ms)', 'Color', 'w');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    scatter(phaseSec * 1e3, shortCpu, 8, [0.20 0.55 0.90], 'filled', ...
        'MarkerFaceAlpha', 0.15, 'MarkerEdgeAlpha', 0.15);
    grid on;
    ylabel('CPU [%]');
    title('Raw short CPU folded into 25ms cycle');
    xlim([0, phaseCycleSec * 1e3]);
    ylim([0, 100]);

    nexttile;
    plot(phaseCentersSec * 1e3, phaseMean, '-o', 'LineWidth', 1.4, 'Color', [0.00 0.45 0.74]);
    hold on;
    plot(phaseCentersSec * 1e3, phaseP95, '--s', 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]);
    hold off;
    grid on;
    xlabel('Phase in 25ms cycle [ms]');
    ylabel('CPU [%]');
    title('5ms-bin statistics within 25ms cycle');
    xlim([0, phaseCycleSec * 1e3]);
    xticks(0:(phaseBinSec * 1e3):(phaseCycleSec * 1e3));
    ylim([0, 100]);
    legend('Mean', 'P95', 'Location', 'best');
else
    figure('Name', 'CPU Windows', 'Color', 'w');
    axis off;
    text(0.1, 0.5, 'CPU window CSVs were not found', 'FontSize', 12);
end

%% Figure 6: Per-core CPU usage (optional)
if hasPerCoreCpu
    nCores = size(coreCpu, 2);

    figure('Name', 'Per-Core CPU', 'Color', 'w');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    imagesc(tCore, 1:nCores, coreCpu');
    axis xy;
    colormap(parula);
    caxis([0, 100]);
    cb = colorbar;
    cb.Label.String = 'CPU [%]';
    ylabel('Core index');
    title('Per-core CPU over time');
    if nCores <= 16
        yticks(1:nCores);
    else
        yticks(1:2:nCores);
    end
    grid on;

    phaseCycleCoreSec = 0.025;
    phaseBinCoreSec = 0.005;
    phaseCoreSec = mod(tCore, phaseCycleCoreSec);
    phaseCoreEdgesSec = 0:phaseBinCoreSec:phaseCycleCoreSec;
    [~, ~, phaseCoreBinIdx] = histcounts(phaseCoreSec, phaseCoreEdgesSec);

    nPhaseBins = numel(phaseCoreEdgesSec) - 1;
    phaseCoreCentersSec = phaseCoreEdgesSec(1:end-1) + (phaseBinCoreSec / 2);
    phaseCoreMean = nan(nCores, nPhaseBins);

    for b = 1:nPhaseBins
        idx = (phaseCoreBinIdx == b);
        if any(idx)
            phaseCoreMean(:, b) = mean(coreCpu(idx, :), 1, 'omitnan')';
        end
    end

    nexttile;
    imagesc(phaseCoreCentersSec * 1e3, 1:nCores, phaseCoreMean);
    axis xy;
    colormap(parula);
    caxis([0, 100]);
    cb2 = colorbar;
    cb2.Label.String = 'CPU [%]';
    xlabel('Phase in 25ms cycle [ms]');
    ylabel('Core index');
    title('Per-core mean CPU by 5ms phase bin');
    xticks(0:(phaseBinCoreSec * 1e3):(phaseCycleCoreSec * 1e3));
    if nCores <= 16
        yticks(1:nCores);
    else
        yticks(1:2:nCores);
    end
    grid on;
end

%% Figure 7: GPU windows over time (optional)
if hasGpuShort || hasGpuLong
    figure('Name', 'GPU Windows', 'Color', 'w');
    if hasGpuLong
        tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');
    else
        tiledlayout(1,1, 'Padding', 'compact', 'TileSpacing', 'compact');
    end

    nexttile;
    plot(tGpu, shortGpu, 'Color', [0.49 0.18 0.56], 'LineWidth', 0.8);
    grid on;
    ylabel('GPU [%]');
    title('Short-rate GPU usage');
    ylim([0, 100]);
    if ~hasGpuLong
        xlabel('Time [s]');
    end

    if hasGpuLong
        nexttile;
        stairs(tLong, longGpu, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.4);
        grid on;
        xlabel('Time [s]');
        ylabel('GPU [%]');
        title('Long window GPU (1 Hz log cadence)');
        ylim([0, 100]);
    end
end

%% Figure 8: CPU vs GPU alignment and phase comparison (optional)
if hasCpuWindows && hasGpuShort
    gpuAligned = interp1(tGpu, shortGpu, tShort, 'linear', 'extrap');

    phaseCycleAlignSec = 0.025;
    phaseBinAlignSec = 0.005;
    phaseAlignSec = mod(tShort, phaseCycleAlignSec);
    phaseAlignEdgesSec = 0:phaseBinAlignSec:phaseCycleAlignSec;
    [~, ~, phaseAlignIdx] = histcounts(phaseAlignSec, phaseAlignEdgesSec);

    nAlignBins = numel(phaseAlignEdgesSec) - 1;
    phaseAlignCentersSec = phaseAlignEdgesSec(1:end-1) + (phaseBinAlignSec / 2);
    cpuAlignMean = nan(nAlignBins, 1);
    gpuAlignMean = nan(nAlignBins, 1);

    for b = 1:nAlignBins
        cpuVals = shortCpu(phaseAlignIdx == b);
        gpuVals = gpuAligned(phaseAlignIdx == b);

        cpuVals = cpuVals(isfinite(cpuVals));
        gpuVals = gpuVals(isfinite(gpuVals));

        if ~isempty(cpuVals)
            cpuAlignMean(b) = mean(cpuVals);
        end
        if ~isempty(gpuVals)
            gpuAlignMean(b) = mean(gpuVals);
        end
    end

    figure('Name', 'CPU-GPU Alignment (25ms)', 'Color', 'w');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(tShort, shortCpu, 'Color', [0.00 0.45 0.74], 'LineWidth', 0.8);
    hold on;
    plot(tShort, gpuAligned, 'Color', [0.49 0.18 0.56], 'LineWidth', 0.8);
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Utilization [%]');
    title('Short-rate aligned CPU and GPU');
    ylim([0, 100]);
    legend('CPU short', 'GPU', 'Location', 'best');

    nexttile;
    plot(phaseAlignCentersSec * 1e3, cpuAlignMean, '-o', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
    hold on;
    plot(phaseAlignCentersSec * 1e3, gpuAlignMean, '-s', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.4);
    hold off;
    grid on;
    xlabel('Phase in 25ms cycle [ms]');
    ylabel('Mean utilization [%]');
    title('CPU vs GPU mean by 5ms phase bin');
    xticks(0:(phaseBinAlignSec * 1e3):(phaseCycleAlignSec * 1e3));
    ylim([0, 100]);
    legend('CPU mean', 'GPU mean', 'Location', 'best');
end

%% Stats summary
if hasPipeline
    fprintf('\nLatency summary\n');
    printStats('scan->scan_{walls}', scan2walls);
    printStats('scan_{walls}->amcl', walls2amcl);
    printStats('amcl->ekf', amcl2ekf);
    printStats('scan->ekf', scan2ekf);
    if hasE2D
        printStats('ekf->drive', e2dPlot);
    end
    if hasS2D
        printStats('scan->drive', s2dPlot);
    end
end

if hasCpuWindows
    fprintf('\nCPU window summary\n');
    printStats('cpu_short_window', shortCpu);
    printStats('cpu_long_window', longCpu);

    fprintf('\nShort CPU phase summary (25ms cycle, 5ms bins)\n');
    for b = 1:numel(phaseMean)
        fprintf('phase [%5.1f,%5.1f) ms : mean=%8.3f  p95=%8.3f  n=%d\n', ...
            phaseEdgesSec(b) * 1e3, phaseEdgesSec(b + 1) * 1e3, ...
            phaseMean(b), phaseP95(b), phaseCount(b));
    end
end

if hasGpuShort || hasGpuLong
    fprintf('\nGPU summary\n');
    if hasGpuShort
        printStats('gpu_short_window', shortGpu);
    end
    if hasGpuLong
        printStats('gpu_long_window', longGpu);
    end
end

if hasPerCoreCpu
    coreMean = mean(coreCpu, 1, 'omitnan');
    coreP95 = prctile(coreCpu, 95, 1);
    [~, coreOrder] = sort(coreMean, 'descend');
    topK = min(8, numel(coreOrder));

    fprintf('\nPer-core summary (top %d by mean usage)\n', topK);
    for i = 1:topK
        c = coreOrder(i);
        fprintf('%-12s : mean=%8.3f %%   p95=%8.3f %%\n', ...
            coreNames{c}, coreMean(c), coreP95(c));
    end
end

fprintf('\nPlots are shown interactively and not saved to disk.\n');
end

function fp = latestFileAny(dirPath, patterns)
allMatches = [];
for i = 1:numel(patterns)
    matches = dir(fullfile(dirPath, patterns{i}));
    if ~isempty(matches)
        allMatches = [allMatches; matches(:)]; %#ok<AGROW>
    end
end

if isempty(allMatches)
    error('No pipeline file found in %s matching patterns: %s', dirPath, strjoin(patterns, ', '));
end

[~, idx] = max([allMatches.datenum]);
fp = fullfile(dirPath, allMatches(idx).name);
end

function assertHasColumns(T, required, tableLabel)
for i = 1:numel(required)
    if ~ismember(required{i}, T.Properties.VariableNames)
        error('%s is missing required column: %s', tableLabel, required{i});
    end
end
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

% Plot smoothed trend with a local variability band.
win = max(11, floor(numel(y) * 0.01));
if mod(win,2) == 0
    win = win + 1;
end

yTrend = movmedian(y, win);
ySpread = movstd(y, win, 0, 'omitnan');
yLo = max(0, yTrend - ySpread);
yHi = yTrend + ySpread;

bandX = [t; flipud(t)];
bandY = [yLo; flipud(yHi)];
fill(bandX, bandY, color, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
hold on;

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
