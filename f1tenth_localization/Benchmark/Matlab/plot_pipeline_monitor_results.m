clc;
close all;
% Plot pipeline latency and system usage CSV outputs.
% This is a normal script (not a function).
%
% Edit these paths before pressing Run:
csvDir = '/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/Matlab/csv/TestRun';
plotsRootDir = '/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/Matlab/plots';
%
% The script expects these files in csvDir:
%   SystemUsageGpu.csv
%   SystemUsageLong.csv
%   SystemUsageShort.csv
%   SystemUsagePerCore.csv
%   SystemUsageNodeProcesses.csv
%
% Optional in csvDir:
%   newest pipeline_*.csv, Pipeline_*.csv, or pipeline_latency_*.csv
%
% Figures are saved automatically to:
%   <plotsRootDir>/<input_folder_name>/

scriptDir = fileparts(mfilename('fullpath'));
if isempty(csvDir)
    csvDir = fullfile(scriptDir, 'csv');
end
if isempty(plotsRootDir)
    plotsRootDir = fullfile(scriptDir, 'plots');
end

csvDir = stripTrailingSeparator(csvDir);
[runName, outputDir] = prepareOutputDirectory(csvDir, plotsRootDir);

pipelinePatterns = {'pipeline_*.csv', 'Pipeline_*.csv', 'pipeline_latency_*.csv', 'PipeLine_*.csv'};
pipelineFile = latestFileAnyOptional(csvDir, pipelinePatterns);
if isempty(pipelineFile)
    parentDir = fileparts(csvDir);
    if ~isempty(parentDir)
        pipelineFile = latestFileAnyOptional(parentDir, pipelinePatterns);
        if ~isempty(pipelineFile)
            fprintf('No pipeline CSV in input folder, using parent folder pipeline CSV.\n');
        end
    end
end
gpuFile = fullfile(csvDir, 'SystemUsageGpu.csv');
longCpuFile = fullfile(csvDir, 'SystemUsageLong.csv');
shortCpuFile = fullfile(csvDir, 'SystemUsageShort.csv');
perCoreCpuFile = fullfile(csvDir, 'SystemUsagePerCore.csv');
nodeProcessCpuFile = fullfile(csvDir, 'SystemUsageNodeProcesses.csv');

fprintf('Input CSV folder : %s\n', csvDir);
fprintf('Output plot folder: %s\n', outputDir);

% Data containers and availability flags
Tp = table();
Tlong = table();
Tshort = table();
Tgpu = table();
Tcore = table();
Tnode = table();

hasPipeline = false;
hasCpuLong = false;
hasCpuShort = false;
hasGpuFile = false;
hasGpuLong = false;
hasGpuShort = false;
hasPerCoreCpu = false;
hasNodeProcessCpu = false;

longCpu = [];
shortCpu = [];
gpuStandalone = [];
longGpu = [];
shortGpu = [];
coreCpu = [];
coreNames = {};

tLong = [];
tShort = [];
tGpu = [];
tCore = [];

if ~isempty(pipelineFile)
    fprintf('Using pipeline CSV: %s\n', pipelineFile);
    Tp = readtable(pipelineFile, 'VariableNamingRule', 'preserve');

    requiredPipeline = {
        'wall_time_ns', ...
        'scan_to_amcl_ms', ...
        'amcl_to_ekf_ms', ...
        'scan_to_ekf_ms'};
    assertHasColumns(Tp, requiredPipeline, 'Pipeline CSV');

    if height(Tp) == 0
        fprintf('Pipeline CSV has no rows. Skipping pipeline latency figures.\n');
    else
        hasPipeline = true;
    end
else
    fprintf('No pipeline file found in input folder. Proceeding with system usage figures only.\n');
end

if isfile(longCpuFile)
    fprintf('Using long CPU CSV:  %s\n', longCpuFile);
    Tlong = readtable(longCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tlong, {'monotonic_time_ns', 'cpu_long_window_percent'}, 'Long CPU CSV');

    if height(Tlong) > 0
        hasCpuLong = true;
        longCpu = double(Tlong.cpu_long_window_percent);
        tLong = (double(Tlong.monotonic_time_ns) - double(Tlong.monotonic_time_ns(1))) * 1e-9;

        hasGpuLong = ismember('gpu_percent', Tlong.Properties.VariableNames);
        if hasGpuLong
            longGpu = double(Tlong.gpu_percent);
        end
    else
        fprintf('Long CPU CSV is empty.\n');
    end
else
    fprintf('Long CPU CSV not found. Expected: %s\n', longCpuFile);
end

if isfile(shortCpuFile)
    fprintf('Using short CPU CSV: %s\n', shortCpuFile);
    Tshort = readtable(shortCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tshort, {'monotonic_time_ns', 'cpu_short_window_percent'}, 'Short CPU CSV');

    if height(Tshort) > 0
        hasCpuShort = true;
        shortCpu = double(Tshort.cpu_short_window_percent);
        tShort = (double(Tshort.monotonic_time_ns) - double(Tshort.monotonic_time_ns(1))) * 1e-9;

        hasGpuShort = ismember('gpu_percent', Tshort.Properties.VariableNames);
        if hasGpuShort
            shortGpu = double(Tshort.gpu_percent);
        end
    else
        fprintf('Short CPU CSV is empty.\n');
    end
else
    fprintf('Short CPU CSV not found. Expected: %s\n', shortCpuFile);
end

if isfile(gpuFile)
    fprintf('Using GPU CSV:       %s\n', gpuFile);
    Tgpu = readtable(gpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tgpu, {'monotonic_time_ns', 'gpu_percent'}, 'GPU CSV');

    if height(Tgpu) > 0
        hasGpuFile = true;
        gpuStandalone = double(Tgpu.gpu_percent);
        tGpu = (double(Tgpu.monotonic_time_ns) - double(Tgpu.monotonic_time_ns(1))) * 1e-9;
    else
        fprintf('GPU CSV is empty.\n');
    end
else
    fprintf('GPU CSV not found. Expected: %s\n', gpuFile);
end

if isfile(nodeProcessCpuFile)
    fprintf('Using node-process CPU CSV: %s\n', nodeProcessCpuFile);
    Tnode = readtable(nodeProcessCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tnode, {'monotonic_time_ns', 'node_name', 'pid', 'cpu_percent'}, 'Node Process CPU CSV');

    if height(Tnode) > 0
        hasNodeProcessCpu = true;
    else
        fprintf('Node-process CPU CSV is empty.\n');
    end
else
    fprintf('Node-process CPU CSV not found. Expected: %s\n', nodeProcessCpuFile);
end

if isfile(perCoreCpuFile)
    fprintf('Using per-core CPU CSV: %s\n', perCoreCpuFile);
    Tcore = readtable(perCoreCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tcore, {'monotonic_time_ns'}, 'Per-core CPU CSV');

    coreCols = startsWith(Tcore.Properties.VariableNames, 'cpu_core_') & ...
        endsWith(Tcore.Properties.VariableNames, '_percent');
    coreNames = Tcore.Properties.VariableNames(coreCols);

    if height(Tcore) > 0 && ~isempty(coreNames)
        hasPerCoreCpu = true;
        coreCpu = zeros(height(Tcore), numel(coreNames));
        for i = 1:numel(coreNames)
            coreCpu(:, i) = double(Tcore.(coreNames{i}));
        end
        tCore = (double(Tcore.monotonic_time_ns) - double(Tcore.monotonic_time_ns(1))) * 1e-9;
    else
        fprintf('Per-core CPU CSV is empty or missing cpu_core_*_percent columns.\n');
    end
else
    fprintf('Per-core CPU CSV not found. Expected: %s\n', perCoreCpuFile);
end

% Keep metrics in scope for summary printing
scan2amcl = [];
scanStamp2scan = [];
amcl2ekf = [];
scan2ekf = [];
e2dPlot = [];
d2aPlot = [];
s2aPlot = [];
hasScanStamp2Scan = false;
hasE2D = false;
hasD2A = false;
hasS2A = false;

if hasPipeline
    tPipe = (double(Tp.wall_time_ns) - double(Tp.wall_time_ns(1))) * 1e-9;

    warmupSeconds = 3.0;
    maxLatencyMs = 200.0;

    hasScanStamp2Scan = ismember('scan_stamp_to_scan_ms', Tp.Properties.VariableNames);

    hasE2D = ismember('ekf_to_command_ms', Tp.Properties.VariableNames) || ...
             ismember('ekf_to_drive_ms', Tp.Properties.VariableNames);
    hasD2A = ismember('drive_to_ackermann_ms', Tp.Properties.VariableNames) || ...
             ismember('drive_to_ackermann_cmd_ms', Tp.Properties.VariableNames);
    hasS2A = ismember('scan_to_ackermann_ms', Tp.Properties.VariableNames) || ...
             ismember('scan_to_command_ms', Tp.Properties.VariableNames) || ...
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

    if hasD2A
        if ismember('drive_to_ackermann_ms', Tp.Properties.VariableNames)
            d2a = double(Tp.drive_to_ackermann_ms);
        else
            d2a = double(Tp.drive_to_ackermann_cmd_ms);
        end
    else
        d2a = nan(height(Tp), 1);
    end

    if hasS2A
        if ismember('scan_to_ackermann_ms', Tp.Properties.VariableNames)
            s2a = double(Tp.scan_to_ackermann_ms);
        elseif ismember('scan_to_command_ms', Tp.Properties.VariableNames)
            s2a = double(Tp.scan_to_command_ms);
        else
            s2a = double(Tp.scan_to_drive_ms);
        end
    else
        s2a = nan(height(Tp), 1);
    end

    e2d(e2d < 0) = NaN;
    d2a(d2a < 0) = NaN;
    s2a(s2a < 0) = NaN;

    scan2amcl = double(Tp.scan_to_amcl_ms);
    if hasScanStamp2Scan
        scanStamp2scan = double(Tp.scan_stamp_to_scan_ms);
    else
        scanStamp2scan = nan(height(Tp), 1);
    end
    amcl2ekf = double(Tp.amcl_to_ekf_ms);
    scan2ekf = double(Tp.scan_to_ekf_ms);

    e2dPlot = e2d;
    d2aPlot = d2a;
    s2aPlot = s2a;
    scanStamp2scan(tPipe < warmupSeconds) = NaN;
    e2dPlot(tPipe < warmupSeconds) = NaN;
    d2aPlot(tPipe < warmupSeconds) = NaN;
    s2aPlot(tPipe < warmupSeconds) = NaN;
    scanStamp2scan(scanStamp2scan > maxLatencyMs) = NaN;
    e2dPlot(e2dPlot > maxLatencyMs) = NaN;
    d2aPlot(d2aPlot > maxLatencyMs) = NaN;
    s2aPlot(s2aPlot > maxLatencyMs) = NaN;

    %% Figure 1: Pipeline stage latency over time
    figure('Name', 'Pipeline Latency Over Time', 'Color', 'w');
    tiledlayout(4, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    if hasScanStamp2Scan
        plotLatencyTrace(tPipe, scanStamp2scan, 'scan_stamp->scan_rx', [0.35 0.35 0.35]);
    else
        axis off;
        text(0.1, 0.5, 'No scan_stamp->scan_rx column', 'FontSize', 11);
    end

    nexttile;
    plotLatencyTrace(tPipe, scan2amcl, 'scan->amcl', [0.0 0.45 0.74]);

    nexttile;
    plotLatencyTrace(tPipe, amcl2ekf, 'amcl->ekf', [0.85 0.33 0.10]);

    nexttile;
    plotLatencyTrace(tPipe, scan2ekf, 'scan->ekf', [0.93 0.69 0.13]);

    nexttile;
    if hasE2D
        plotLatencyTrace(tPipe, e2dPlot, 'ekf->drive', [0.49 0.18 0.56]);
    else
        axis off;
        text(0.1, 0.5, 'No ekf->drive column', 'FontSize', 11);
    end

    nexttile;
    if hasD2A
        plotLatencyTrace(tPipe, d2aPlot, 'drive->ackermann', [0.08 0.50 0.18]);
    else
        axis off;
        text(0.1, 0.5, 'No drive->ackermann column', 'FontSize', 11);
    end

    nexttile;
    if hasS2A
        plotLatencyTrace(tPipe, s2aPlot, 'scan->ackermann', [0.00 0.35 0.75]);
    else
        axis off;
        text(0.1, 0.5, 'No scan->ackermann column', 'FontSize', 11);
    end

    nexttile;
    axis off;

    %% Figure 2: Combined histograms
    scan2ekfAllValid = scan2ekf(isfinite(scan2ekf) & scan2ekf >= 0);
    scan2ekfValid = scan2ekfAllValid;
    if hasS2A
        s2dValid = s2aPlot(~isnan(s2aPlot));
    else
        s2dValid = [];
    end

    figure('Name', 'Latency Histograms', 'Color', 'w');
    tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    histogram(scan2ekfValid, 80);
    grid on;
    title('Histogram: scan->ekf', 'FontSize', 13);
    xlabel('scan->ekf [ms]', 'FontSize', 12);
    ylabel('Count', 'FontSize', 12);
    set(gca, 'FontSize', 11, 'LineWidth', 0.9);
    if ~isempty(scan2ekfValid)
        xlim([0, max(5, prctile(scan2ekfValid, 99.5))]);
    end

    nexttile;
    if ~isempty(s2dValid)
        histogram(s2dValid, 80);
        grid on;
        title('Histogram: scan->ackermann', 'FontSize', 13);
        xlabel('scan->ackermann [ms]', 'FontSize', 12);
        ylabel('Count', 'FontSize', 12);
        set(gca, 'FontSize', 11, 'LineWidth', 0.9);
        xlim([0, max(5, prctile(s2dValid, 99.5))]);
    else
        axis off;
        text(0.1, 0.5, 'No scan->ackermann data available', 'FontSize', 11);
    end

    %% Figure 3: Boxplot with seven timing metrics
    figure('Name', 'Latency Boxplot', 'Color', 'w');

    b1 = clipForBoxplot(scanStamp2scan(isfinite(scanStamp2scan) & scanStamp2scan >= 0));
    b2 = clipForBoxplot(scan2amcl(isfinite(scan2amcl) & scan2amcl >= 0));
    b3 = clipForBoxplot(amcl2ekf(isfinite(amcl2ekf) & amcl2ekf >= 0));
    b4 = clipForBoxplot(scan2ekf(isfinite(scan2ekf) & scan2ekf >= 0));
    b5 = clipForBoxplot(e2dPlot(isfinite(e2dPlot) & e2dPlot >= 0));
    b6 = clipForBoxplot(d2aPlot(isfinite(d2aPlot) & d2aPlot >= 0));
    b7 = clipForBoxplot(s2aPlot(isfinite(s2aPlot) & s2aPlot >= 0));

    allVals = [b1; b2; b3; b4; b5; b6; b7];
    grp = [
        repmat({'scan_stamp->scan_rx'}, numel(b1), 1);
        repmat({'scan->amcl'}, numel(b2), 1);
        repmat({'amcl->ekf'}, numel(b3), 1);
        repmat({'scan->ekf'}, numel(b4), 1);
        repmat({'ekf->drive'}, numel(b5), 1);
        repmat({'drive->ackermann'}, numel(b6), 1);
        repmat({'scan->ackermann'}, numel(b7), 1);
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

%% Figure 4: CPU windows over time
figure('Name', 'CPU Windows', 'Color', 'w');
if hasCpuLong || hasCpuShort
    hold on;
    handles = gobjects(0);
    labels = cell(0, 1);

    if hasCpuShort
        hShort = plot(tShort, shortCpu, '-', 'Color', [0.75 0.78 0.82], 'LineWidth', 0.9);
        handles(end + 1) = hShort; %#ok<AGROW>
        labels{end + 1} = 'CPU short window'; %#ok<AGROW>
    end

    if hasCpuLong
        hLong = stairs(tLong, longCpu, '-', 'Color', [0.05 0.05 0.05], 'LineWidth', 2.4);
        handles(end + 1) = hLong; %#ok<AGROW>
        labels{end + 1} = 'CPU long window'; %#ok<AGROW>
    end

    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('CPU [%]');
    title('CPU windows over time');
    ylim([0, 100]);
    legend(handles, labels, 'Location', 'northeast', 'FontSize', 12);
else
    axis off;
    text(0.1, 0.5, 'CPU window CSVs were not found', 'FontSize', 12);
end

%% Figure 5: Per-core CPU usage
if hasPerCoreCpu
    nCores = size(coreCpu, 2);

    figure('Name', 'Per-Core CPU', 'Color', 'w');
    imagesc(tCore, 1:nCores, coreCpu');
    axis xy;
    colormap(parula);
    caxis([0, 100]);
    cb = colorbar;
    cb.Label.String = 'CPU [%]';
    xlabel('Time [s]');
    ylabel('Core index');
    title('Per-core CPU over time');
    if nCores <= 16
        yticks(1:nCores);
    else
        yticks(1:2:nCores);
    end
    set(gca, 'FontSize', 11, 'LineWidth', 0.9);
end

%% Figure 6: GPU usage over time
figure('Name', 'GPU Usage', 'Color', 'w');
if hasGpuFile || hasGpuShort
    hold on;
    handles = gobjects(0);
    labels = cell(0, 1);
    gpuUpdateHz = NaN;

    if hasGpuFile
        % Plot raw monitor as a light line, then overlay a smoothed trend.
        gpuRaw = double(gpuStandalone);
        gpuRaw(~isfinite(gpuRaw) | gpuRaw < 0 | gpuRaw > 100) = NaN;
        gpuRawPlot = maskIsolatedGpuDropouts(gpuRaw, 4.0, 20.0, 35.0, 6);

        if numel(tGpu) >= 2
            dtRaw = median(diff(tGpu));
            if isfinite(dtRaw) && dtRaw > 0
                gpuUpdateHz = 1.0 / dtRaw;
            end
        end

        hGpuRaw = plot(tGpu, gpuRawPlot, '-', 'Color', [0.80 0.86 0.80], 'LineWidth', 0.9);
        handles(end + 1) = hGpuRaw; %#ok<AGROW>
        labels{end + 1} = 'GPU monitor (raw, filtered)'; %#ok<AGROW>

        if numel(tGpu) >= 3
            dtGpu = median(diff(tGpu));
            if ~isfinite(dtGpu) || dtGpu <= 0
                dtGpu = 0.2;
            end
            smoothWinGpu = max(5, round(1.0 / dtGpu));
            if mod(smoothWinGpu, 2) == 0
                smoothWinGpu = smoothWinGpu + 1;
            end
            gpuTrend = movmedian(gpuRawPlot, smoothWinGpu, 'omitnan');
        else
            gpuTrend = gpuRawPlot;
        end

        hGpuTrend = plot(tGpu, gpuTrend, '-', 'Color', [0.15 0.55 0.20], 'LineWidth', 2.0);
        handles(end + 1) = hGpuTrend; %#ok<AGROW>
        labels{end + 1} = 'GPU monitor (1s trend)'; %#ok<AGROW>
    end

    if hasGpuShort && ~hasGpuFile && ~hasGpuLong
        hGpuShort = plot(tShort, shortGpu, ':', 'Color', [0.64 0.08 0.18], 'LineWidth', 1.5);
        handles(end + 1) = hGpuShort; %#ok<AGROW>
        labels{end + 1} = 'GPU from short window'; %#ok<AGROW>
    end

    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('GPU [%]');
    if isfinite(gpuUpdateHz)
        title(sprintf('GPU usage over time (monitor ~%.1f Hz)', gpuUpdateHz));
    else
        title('GPU usage over time');
    end
    ylim([0, 100]);
    if ~isempty(handles)
        legend(handles, labels, 'Location', 'northeast', 'FontSize', 12);
    end
else
    axis off;
    text(0.1, 0.5, 'GPU CSVs were not found', 'FontSize', 12);
end

%% Figure 7: Per-node CPU usage
if hasNodeProcessCpu
    nodeNames = string(Tnode.node_name);
    nodeCpu = double(Tnode.cpu_percent);
    nodePid = double(Tnode.pid);

    validNode = isfinite(nodeCpu) & (nodeCpu >= 0) & strlength(nodeNames) > 0;
    nodeNames = nodeNames(validNode);
    nodeCpu = nodeCpu(validNode);
    nodePid = nodePid(validNode);

    if ~isempty(nodeCpu)
        [uniqueNodes, ~, nodeIdx] = unique(nodeNames, 'stable');
        nodeMean = accumarray(nodeIdx, nodeCpu, [], @mean, NaN);
        [~, sortIdx] = sort(nodeMean, 'descend');

        topK = min(8, numel(uniqueNodes));
        topNodes = uniqueNodes(sortIdx(1:topK));

        nodeTimeNs = double(Tnode.monotonic_time_ns(validNode));
        [uniqueNodeTimeNs, ~, timeIdx] = unique(nodeTimeNs, 'stable');
        trackedRosTotal = accumarray(timeIdx, nodeCpu, [], @sum, NaN);

        figure('Name', 'Per-Node CPU', 'Color', 'w');

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

        if all(isnan(nodeMeanTop))
            axis off;
            text(0.1, 0.5, 'No per-node CPU samples available', 'FontSize', 11);
        else
            bar([nodeMeanTop, nodeP95Top], 'grouped');
            grid on;
            ylabel('CPU [%]');
            xlabel('Node');
            title('Per-node CPU summary (mean and P95)');
            legend('Mean', 'P95', 'Location', 'northeast', 'FontSize', 12);
            xticks(1:topK);
            xticklabels(cellstr(topNodes));
            xtickangle(20);
            ylim([0, 100]);
            set(gca, 'FontSize', 11, 'LineWidth', 0.9);
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
                'tracked_ros_total', mean(trackedRosValid), prctile(trackedRosValid, 95), prctile(trackedRosValid, 99));
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

if hasCpuLong || hasCpuShort
    fprintf('\nCPU window summary\n');
    if hasCpuLong
        printStats('cpu_long_window', longCpu, '%');
    end
    if hasCpuShort
        printStats('cpu_short_window', shortCpu, '%');
    end
end

if hasGpuFile || hasGpuLong || hasGpuShort
    fprintf('\nGPU summary\n');
    if hasGpuFile
        printStats('gpu_monitor', gpuStandalone, '%');
    end
    if hasGpuLong
        printStats('gpu_long_window', longGpu, '%');
    end
    if hasGpuShort
        printStats('gpu_short_window', shortGpu, '%');
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
        fprintf('%-12s : mean=%8.3f %%   p95=%8.3f %%\n', coreNames{c}, coreMean(c), coreP95(c));
    end
end

saveFiguresReportReady(outputDir, runName);
fprintf('\nPlots are shown interactively and saved to disk.\n');

function fp = latestFileAnyOptional(dirPath, patterns)
allMatches = [];
for i = 1:numel(patterns)
    matches = dir(fullfile(dirPath, patterns{i}));
    if ~isempty(matches)
        allMatches = [allMatches; matches(:)]; %#ok<AGROW>
    end
end

if isempty(allMatches)
    fp = '';
    return;
end

[~, idx] = max([allMatches.datenum]);
fp = fullfile(dirPath, allMatches(idx).name);
end

function [runName, outputDir] = prepareOutputDirectory(csvDir, plotsRootDir)
if ~exist(plotsRootDir, 'dir')
    mkdir(plotsRootDir);
end

[~, runName] = fileparts(csvDir);
if isempty(runName)
    runName = 'csv';
end

outputDir = fullfile(plotsRootDir, runName);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
end

function saveFiguresReportReady(outputDir, runName)
figs = findall(0, 'Type', 'figure');
if isempty(figs)
    fprintf('\nNo figures generated to save.\n');
    return;
end

% Remove previous exports so output folder only contains the latest PNG set.
delete(fullfile(outputDir, '*.png'));
delete(fullfile(outputDir, '*.pdf'));

[~, order] = sort([figs.Number]);
figs = figs(order);

for i = 1:numel(figs)
    fig = figs(i);
    if ~isgraphics(fig, 'figure')
        continue;
    end

    set(fig, 'Color', 'w');
    set(fig, 'InvertHardcopy', 'off');
    set(fig, 'Units', 'pixels');

    pos = get(fig, 'Position');
    if pos(3) < 1600 || pos(4) < 900
        set(fig, 'Position', [pos(1), pos(2), 1800, 1000]);
    end

    figName = string(get(fig, 'Name'));
    figName = strtrim(figName);
    if strlength(figName) == 0
        figName = sprintf('figure_%02d', i);
    end

    figNameSafe = sanitizeFileName(char(figName));
    baseName = sprintf('%02d_%s', i, figNameSafe);

    pngPath = fullfile(outputDir, [baseName, '.png']);

    drawnow;
    try
        exportgraphics(fig, pngPath, 'Resolution', 450);
    catch
        if isgraphics(fig, 'figure')
            try
                print(fig, pngPath, '-dpng', '-r450');
            catch
                warning('Failed to save PNG for figure %d: %s', i, figNameSafe);
            end
        end
    end
end

fprintf('\nSaved %d figure(s) to %s\n', numel(figs), outputDir);
fprintf('Run folder label: %s\n', runName);
end

function nameOut = sanitizeFileName(nameIn)
nameOut = regexprep(nameIn, '[^a-zA-Z0-9_-]+', '_');
nameOut = regexprep(nameOut, '_+', '_');
nameOut = regexprep(nameOut, '^_+', '');
nameOut = regexprep(nameOut, '_+$', '');
if isempty(nameOut)
    nameOut = 'figure';
end
end

function outPath = stripTrailingSeparator(inPath)
outPath = char(inPath);
while numel(outPath) > 1 && (outPath(end) == '/' || outPath(end) == '\')
    outPath(end) = [];
end
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

win = max(11, floor(numel(y) * 0.01));
if mod(win, 2) == 0
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
xlabel('Time [s]', 'FontSize', 10);
ylabel('Latency [ms]', 'FontSize', 10);
title(titleText, 'FontSize', 11);
set(gca, 'FontSize', 9, 'LineWidth', 0.8);
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

function yOut = maskIsolatedGpuDropouts(yIn, lowThreshold, neighborMin, jumpThreshold, maxDropRun)
% Conservative cleanup: suppress isolated glitches and short near-zero runs.
yOut = double(yIn(:));
n = numel(yOut);
if n < 3
    return;
end

if nargin < 5 || isempty(maxDropRun)
    maxDropRun = 3;
end

for i = 2:(n - 1)
    yPrev = yOut(i - 1);
    yCur = yOut(i);
    yNext = yOut(i + 1);

    if ~isfinite(yPrev) || ~isfinite(yCur) || ~isfinite(yNext)
        continue;
    end

    % Isolated low dropout between two clearly valid neighbors.
    if yCur <= lowThreshold && yPrev >= neighborMin && yNext >= neighborMin
        yOut(i) = NaN;
        continue;
    end

    % Isolated spike/dip: large jump in and out, neighbors are close to each other.
    jumpIn = abs(yCur - yPrev);
    jumpOut = abs(yCur - yNext);
    neighborDelta = abs(yNext - yPrev);
    if jumpIn >= jumpThreshold && jumpOut >= jumpThreshold && neighborDelta <= 12.0
        yOut(i) = NaN;
    end
end

% Short-run near-zero dropout cleanup (keeps sustained low phases untouched).
isLow = isfinite(yOut) & (yOut <= lowThreshold);
edges = diff([false; isLow; false]);
runStarts = find(edges == 1);
runEnds = find(edges == -1) - 1;

for k = 1:numel(runStarts)
    s = runStarts(k);
    e = runEnds(k);
    runLen = e - s + 1;

    if runLen > maxDropRun
        continue;
    end
    if s <= 1 || e >= n
        continue;
    end

    yPrev = yOut(s - 1);
    yNext = yOut(e + 1);
    if isfinite(yPrev) && isfinite(yNext) && yPrev >= neighborMin && yNext >= neighborMin
        yOut(s:e) = NaN;
    end
end
end
