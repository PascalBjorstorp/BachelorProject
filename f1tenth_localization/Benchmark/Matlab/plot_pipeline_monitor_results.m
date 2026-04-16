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
%   SystemUsageLong.csv (optional, includes gpu_percent when available)
%   SystemUsagePerCore.csv (optional)
%   SystemUsageNodeProcesses.csv (optional, per-ROS-node CPU usage)

if nargin < 1 || isempty(csvDir)
    scriptDir = fileparts(mfilename('fullpath'));
    %folderSelected = 'MPC_FPGA';
    %csvDir = fullfile(scriptDir, 'csv', folderSelected);
    csvDir = fullfile(scriptDir, 'csv');
end


pipelineFile = latestFileAny(csvDir, {'pipeline_*.csv'});
longCpuFile = fullfile(csvDir, 'SystemUsageLong.csv');
perCoreCpuFile = fullfile(csvDir, 'SystemUsagePerCore.csv');
nodeProcessCpuFile = fullfile(csvDir, 'SystemUsageNodeProcesses.csv');

fprintf('Using pipeline CSV: %s\n', pipelineFile);

Tp = readtable(pipelineFile, 'VariableNamingRule', 'preserve');

requiredPipeline = {
    'wall_time_ns', ...
    'scan_to_scan_walls_ms', ...
    'walls_to_amcl_ms', ...
    'amcl_to_ekf_ms', ...
    'scan_to_ekf_ms'};
assertHasColumns(Tp, requiredPipeline, 'Pipeline CSV');

hasCpuWindows = isfile(longCpuFile);
hasPerCoreCpu = isfile(perCoreCpuFile);
hasNodeProcessCpu = isfile(nodeProcessCpuFile);
Tlong = table();
Tcore = table();
Tnode = table();
longCpu = [];
longGpu = [];
hasGpuLong = false;
coreCpu = [];
coreNames = {};

if hasCpuWindows
    fprintf('Using long CPU CSV:  %s\n', longCpuFile);

    Tlong = readtable(longCpuFile, 'VariableNamingRule', 'preserve');

    assertHasColumns(Tlong, {'monotonic_time_ns','cpu_long_window_percent'}, 'Long CPU CSV');

    if height(Tlong) == 0
        fprintf('Long CPU CSV is empty. Skipping CPU windows figure.\n');
        hasCpuWindows = false;
    else
        longCpu = double(Tlong.cpu_long_window_percent);

        hasGpuLong = ismember('gpu_percent', Tlong.Properties.VariableNames);
        if hasGpuLong
            longGpu = double(Tlong.gpu_percent);
        end
    end
else
    fprintf('CPU long-window CSV not found. Expected:\n');
    fprintf('  %s\n', longCpuFile);
    fprintf('Proceeding with pipeline latency plots only.\n');
end

if hasNodeProcessCpu
    fprintf('Using node-process CPU CSV: %s\n', nodeProcessCpuFile);
    Tnode = readtable(nodeProcessCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tnode, {'monotonic_time_ns','node_name','pid','cpu_percent'}, 'Node Process CPU CSV');

    if height(Tnode) == 0
        fprintf('Node-process CPU CSV is empty. Skipping per-node CPU figure.\n');
        hasNodeProcessCpu = false;
    end
else
    fprintf('Node-process CPU CSV not found. Expected:\n');
    fprintf('  %s\n', nodeProcessCpuFile);
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
    tLong = (double(Tlong.monotonic_time_ns) - double(Tlong.monotonic_time_ns(1))) * 1e-9;
else
    tLong = [];
end

if hasPerCoreCpu
    tCore = (double(Tcore.monotonic_time_ns) - double(Tcore.monotonic_time_ns(1))) * 1e-9;
else
    tCore = [];
end

if hasNodeProcessCpu
    tNode = (double(Tnode.monotonic_time_ns) - double(Tnode.monotonic_time_ns(1))) * 1e-9;
else
    tNode = [];
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

    % Optional columns (prefer new command naming, keep drive fallback)
    hasE2D = ismember('ekf_to_command_ms', Tp.Properties.VariableNames) || ...
             ismember('ekf_to_drive_ms', Tp.Properties.VariableNames);
    hasS2D = ismember('scan_to_command_ms', Tp.Properties.VariableNames) || ...
             ismember('scan_to_drive_ms', Tp.Properties.VariableNames);

    if hasE2D
        if ismember('ekf_to_command_ms', Tp.Properties.VariableNames)
            e2d = double(Tp.ekf_to_command_ms);
        else
            e2d = double(Tp.ekf_to_drive_ms);
        end
    else
        e2d = nan(height(Tp), 1);
    end

    if hasS2D
        if ismember('scan_to_command_ms', Tp.Properties.VariableNames)
            s2d = double(Tp.scan_to_command_ms);
        else
            s2d = double(Tp.scan_to_drive_ms);
        end
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

    % Clean startup and extreme outliers for command-related metrics.
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
        plotLatencyTrace(tPipe, e2dPlot, 'ekf->motor_cmd', [0.47 0.67 0.19]);
    else
        axis off; text(0.1,0.5,'No ekf->motor_cmd column','FontSize',11);
    end

    nexttile;
    if hasS2D
        plotLatencyTrace(tPipe, s2dPlot, 'scan->motor_cmd', [0.30 0.75 0.93]);
    else
        axis off; text(0.1,0.5,'No scan->motor_cmd column','FontSize',11);
    end

    %% Figure 2: Combined histograms (scan->ekf on top, scan->motor_cmd below)
    scan2ekfAllValid = scan2ekf(isfinite(scan2ekf) & scan2ekf >= 0);

    if hasS2D
        commandMatchedMask = isfinite(scan2ekf) & (scan2ekf >= 0) & ~isnan(s2dPlot);
        scan2ekfValid = scan2ekf(commandMatchedMask);
        s2dValid = s2dPlot(~isnan(s2dPlot));
        droppedUnmatched = max(0, numel(scan2ekfAllValid) - numel(scan2ekfValid));
    else
        scan2ekfValid = scan2ekfAllValid;
        s2dValid = [];
        droppedUnmatched = 0;
    end

    figure('Name', 'Latency Histograms', 'Color', 'w');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    histogram(scan2ekfValid, 80);
    grid on;
    if hasS2D
        title(sprintf('Histogram: scan->ekf (command-matched, dropped %d unmatched)', droppedUnmatched));
    else
        title('Histogram: scan->ekf');
    end
    xlabel('scan->ekf [ms]');
    ylabel('Count');
    if ~isempty(scan2ekfValid)
        xlim([0, max(5, prctile(scan2ekfValid, 99.5))]);
    end

    nexttile;
    if ~isempty(s2dValid)
        histogram(s2dValid, 80);
        grid on;
        title('Histogram: scan->motor_cmd');
        xlabel('scan->motor_cmd [ms]');
        ylabel('Count');
        xlim([0, max(5, prctile(s2dValid, 99.5))]);
    else
        axis off;
        text(0.1, 0.5, 'No scan->motor_cmd data available', 'FontSize', 11);
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
        repmat({'ekf->motor_cmd'}, numel(b5), 1);
        repmat({'scan->motor_cmd'}, numel(b6), 1);
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
    figure('Name', 'CPU Long Window', 'Color', 'w');
    stairs(tLong, longCpu, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('CPU [%]');
    title('Long window CPU (1 Hz)');
    ylim([0, 100]);
else
    figure('Name', 'CPU Long Window', 'Color', 'w');
    axis off;
    text(0.1, 0.5, 'CPU long-window CSVs were not found', 'FontSize', 12);
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
if hasGpuLong
    figure('Name', 'GPU Long Window', 'Color', 'w');
    stairs(tLong, longGpu, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('GPU [%]');
    title('Long window GPU (1 Hz log cadence)');
    ylim([0, 100]);
end

%% Figure 9: Per-node CPU usage (optional)
if hasNodeProcessCpu
    nodeNames = string(Tnode.node_name);
    nodeCpu = double(Tnode.cpu_percent);
    nodePid = double(Tnode.pid);

    validNode = isfinite(nodeCpu) & (nodeCpu >= 0) & strlength(nodeNames) > 0;
    nodeNames = nodeNames(validNode);
    nodeCpu = nodeCpu(validNode);
    nodePid = nodePid(validNode);
    tNodeValid = tNode(validNode);

    if ~isempty(nodeCpu)
        [uniqueNodes, ~, nodeIdx] = unique(nodeNames, 'stable');
        nodeMean = accumarray(nodeIdx, nodeCpu, [], @mean, NaN);
        [~, sortIdx] = sort(nodeMean, 'descend');

        topK = min(8, numel(uniqueNodes));
        topNodes = uniqueNodes(sortIdx(1:topK));

        figure('Name', 'Per-Node CPU', 'Color', 'w');
        tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

        nexttile;
        hold on;
        cmap = lines(topK);
        lineHandles = gobjects(0);
        lineLabels = cell(0, 1);
        for i = 1:topK
            idx = (nodeNames == topNodes(i));
            tVals = tNodeValid(idx);
            yVals = nodeCpu(idx);

            [tVals, order] = sort(tVals);
            yVals = yVals(order);

            if numel(yVals) >= 3
                dt = median(diff(tVals));
                if ~isfinite(dt) || dt <= 0
                    dt = 0.2;
                end
                smoothWin = max(3, round(1.0 / dt));  % ~1 second smoothing
                ySmooth = movmean(yVals, smoothWin, 'omitnan');
            else
                ySmooth = yVals;
            end

            h = plot(tVals, ySmooth, '-', 'Color', cmap(i,:), 'LineWidth', 1.4);
            lineHandles(end + 1) = h; %#ok<AGROW>
            lineLabels{end + 1} = char(topNodes(i)); %#ok<AGROW>
        end

        nodeTimeNs = double(Tnode.monotonic_time_ns(validNode));
        [uniqueNodeTimeNs, ~, timeIdx] = unique(nodeTimeNs, 'stable');
        trackedRosTotal = accumarray(timeIdx, nodeCpu, [], @sum, NaN);
        trackedRosTime = (uniqueNodeTimeNs - uniqueNodeTimeNs(1)) * 1e-9;

        hTotal = plot(trackedRosTime, trackedRosTotal, 'k--', 'LineWidth', 1.8);
        lineHandles(end + 1) = hTotal; %#ok<AGROW>
        lineLabels{end + 1} = 'tracked_ros_total'; %#ok<AGROW>

        hold off;
        grid on;
        ylabel('CPU [%]');
        title('ROS node CPU over time (top nodes by mean, smoothed)');
        ylim([0, 100]);
        legend(lineHandles, lineLabels, 'Location', 'eastoutside');

        nodeMeanTop = nan(topK, 1);
        nodeP95Top = nan(topK, 1);
        for i = 1:topK
            idx = (nodeNames == topNodes(i));
            vals = nodeCpu(idx);
            vals = vals(isfinite(vals));
            if isempty(vals)
                continue;
            end

            nodeMeanTop(i) = mean(vals);
            nodeP95Top(i) = prctile(vals, 95);
        end

        nexttile;
        if all(isnan(nodeMeanTop))
            axis off;
            text(0.1, 0.5, 'No per-node CPU samples available', 'FontSize', 11);
        else
            bar([nodeMeanTop, nodeP95Top], 'grouped');
            grid on;
            ylabel('CPU [%]');
            xlabel('Node');
            title('Per-node CPU summary (mean and P95)');
            legend('Mean', 'P95', 'Location', 'best');
            xticks(1:topK);
            xticklabels(cellstr(topNodes));
            xtickangle(20);
            ylim([0, 100]);
        end

        fprintf('\nPer-node CPU summary (top %d by mean)\n', topK);
        for i = 1:topK
            idx = (nodeNames == topNodes(i));
            vals = nodeCpu(idx);
            vals = vals(isfinite(vals));
            if isempty(vals)
                continue;
            end
            pids = unique(nodePid(idx));
            fprintf('%-28s : mean=%8.3f %%   p95=%8.3f %%   p99=%8.3f %%   pids=%d\n', ...
                char(topNodes(i)), mean(vals), prctile(vals, 95), prctile(vals, 99), numel(pids));
        end

        trackedRosValid = trackedRosTotal(isfinite(trackedRosTotal));
        if ~isempty(trackedRosValid)
            fprintf('%-28s : mean=%8.3f %%   p95=%8.3f %%   p99=%8.3f %%\n', ...
                'tracked_ros_total', mean(trackedRosValid), ...
                prctile(trackedRosValid, 95), prctile(trackedRosValid, 99));
        end
    end
end

%% Stats summary
if hasPipeline
    fprintf('\nLatency summary\n');
    printStats('scan->scan_{walls}', scan2walls, 'ms');
    printStats('scan_{walls}->amcl', walls2amcl, 'ms');
    printStats('amcl->ekf', amcl2ekf, 'ms');
    printStats('scan->ekf', scan2ekf, 'ms');
    if hasE2D
        printStats('ekf->motor_cmd', e2dPlot, 'ms');
    end
    if hasS2D
        printStats('scan->motor_cmd', s2dPlot, 'ms');
    end
end

if hasCpuWindows
    fprintf('\nCPU window summary\n');
    printStats('cpu_long_window', longCpu, '%');
end

if hasGpuLong
    fprintf('\nGPU summary\n');
    printStats('gpu_long_window', longGpu, '%');
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

function printStats(name, data, unit)
if nargin < 3 || isempty(unit)
    unit = 'ms';
end

data = double(data);
data = data(~isnan(data));
if isempty(data)
    fprintf('%-18s : no data\n', name);
    return;
end
p95 = prctile(data, 95);
p99 = prctile(data, 99);
fprintf('%-18s : mean=%8.3f %s   std=%8.3f   p95=%8.3f   p99=%8.3f\n', ...
    name, mean(data), unit, std(data), p95, p99);
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
