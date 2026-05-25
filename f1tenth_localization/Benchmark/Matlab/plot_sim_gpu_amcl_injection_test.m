function runSummary = plot_sim_gpu_amcl_injection_test(runRoot, figureOutputDir, showPlot)
%PLOT_SIM_GPU_AMCL_INJECTION_TEST Make report figures for particle injection.

if nargin < 1 || isempty(runRoot)
    thisFile = mfilename('fullpath');
    matlabDir = fileparts(thisFile);
    runRoot = latestInjectionRun(fullfile(matlabDir, 'sim_benchmark'));
end
if nargin < 2 || isempty(figureOutputDir)
    repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    figureOutputDir = fullfile(repoRoot, 'ReportMaterial', 'TestFigures');
end
if nargin < 3 || isempty(showPlot)
    showPlot = true;
end

runRoot = char(runRoot);
figureOutputDir = char(figureOutputDir);
if ~exist(figureOutputDir, 'dir')
    mkdir(figureOutputDir);
end

config.xyThresholdM = 0.50;
config.yawThresholdRad = 0.50;
config.stableWindowS = 1.0;
config.windowToleranceS = 0.10;
config.skipFirstS = 5.0;
config.minSamplesAfterConvergence = 20;

manifest = loadManifest(runRoot);
rows = repmat(emptySummaryRow(), height(manifest), 1);
for i = 1:height(manifest)
    rows(i) = summarizeRun(manifest(i, :), config);
end
runSummary = struct2table(rows);
runSummary.convergence_time_plot_s = runSummary.convergence_time_s;
endedBeforeConvergence = ~runSummary.converged | ~isfinite(runSummary.convergence_time_s);
runSummary.convergence_time_plot_s(endedBeforeConvergence) = ...
    runSummary.duration_s(endedBeforeConvergence);
runSummary.convergence_plot_is_run_end = endedBeforeConvergence;

summaryPath = fullfile(runRoot, 'AMCL_ParticleInjection_RunSummary.csv');
writetable(runSummary, summaryPath);

labels = ["0%", "5%", "10%", "25%"];
percents = [0, 5, 10, 25];
plotMetricBoxplot( ...
    runSummary, percents, labels, 'convergence_time_plot_s', ...
    'convergence time [s]', figureOutputDir, ...
    'AMCL_ParticleInjection_ConvergenceTime_Boxplot', showPlot, ...
    runSummary.convergence_plot_is_run_end);
convergedSummary = runSummary(runSummary.converged, :);
plotMetricBoxplot( ...
    convergedSummary, percents, labels, 'position_error_median_m', ...
    'median position error [m]', figureOutputDir, ...
    'AMCL_ParticleInjection_PositionError_Boxplot', showPlot);
plotMetricBoxplot( ...
    convergedSummary, percents, labels, 'yaw_error_median_deg', ...
    'median yaw error [deg]', figureOutputDir, ...
    'AMCL_ParticleInjection_YawError_Boxplot', showPlot);

fprintf('Run summary: %s\n', summaryPath);
fprintf('Figures: %s\n', figureOutputDir);
end

function manifest = loadManifest(runRoot)
manifestPath = fullfile(runRoot, 'particle_injection_manifest.csv');
if exist(manifestPath, 'file')
    manifest = readCsvTable(manifestPath);
    return;
end

percents = [0, 5, 10, 25];
rows = {};
for i = 1:numel(percents)
    condition = sprintf('inj_%02dpct', percents(i));
    runDirs = dir(fullfile(runRoot, condition, 'run_*'));
    for j = 1:numel(runDirs)
        if ~runDirs(j).isdir
            continue;
        end
        runDir = fullfile(runDirs(j).folder, runDirs(j).name);
        csvPath = fullfile(runDir, 'AMCL_benchmark', 'AMCL_benchmark.csv');
        statusPath = fullfile(runDir, 'AMCL_benchmark', 'AMCL_benchmark_status.json');
        rows(end + 1, :) = {condition, sprintf('%d%%', percents(i)), ...
            percents(i) / 100, percents(i), j, string(runDir), ...
            string(csvPath), string(statusPath), "", NaN, "", NaN}; %#ok<AGROW>
    end
end

manifest = cell2table(rows, 'VariableNames', { ...
    'condition', 'label', 'injection_ratio', 'injection_percent', 'run', ...
    'run_dir', 'csv_path', 'status_path', 'log_path', 'returncode', ...
    'status_reason', 'status_laps'});
end

function row = summarizeRun(manifestRow, config)
row = emptySummaryRow();
row.condition = string(manifestRow.condition);
row.label = string(manifestRow.label);
row.injection_ratio = numericValue(manifestRow.injection_ratio);
row.injection_percent = numericValue(manifestRow.injection_percent);
row.run = numericValue(manifestRow.run);
row.run_dir = string(manifestRow.run_dir);
row.csv_path = string(manifestRow.csv_path);
row.returncode = numericValue(manifestRow.returncode);
row.status_reason = string(manifestRow.status_reason);
row.status_laps = numericValue(manifestRow.status_laps);

csvPath = char(row.csv_path);
if ~exist(csvPath, 'file')
    row.reason = "missing_csv";
    return;
end

T = readCsvTable(csvPath);
required = ["wall_time_ns", "gt_x", "gt_y", "gt_yaw", "amcl_x", "amcl_y", "amcl_yaw"];
for name = required
    if ~any(strcmp(T.Properties.VariableNames, name))
        row.reason = "missing_column_" + name;
        return;
    end
end

wallNs = numericColumn(T, 'wall_time_ns');
firstWall = wallNs(find(isfinite(wallNs), 1, 'first'));
if isempty(firstWall)
    row.reason = "missing_time";
    return;
end
t = (wallNs - firstWall) .* 1e-9;

gtX = numericColumn(T, 'gt_x');
gtY = numericColumn(T, 'gt_y');
gtYaw = numericColumn(T, 'gt_yaw');
amclX = numericColumn(T, 'amcl_x');
amclY = numericColumn(T, 'amcl_y');
amclYaw = numericColumn(T, 'amcl_yaw');

valid = isfinite(t) & isfinite(gtX) & isfinite(gtY) & isfinite(gtYaw) & ...
    isfinite(amclX) & isfinite(amclY) & isfinite(amclYaw);
posErr = hypot(amclX - gtX, amclY - gtY);
yawErrRad = abs(wrapToPiLocal(amclYaw - gtYaw));

row.samples = numel(t);
row.valid_samples = nnz(valid);
row.valid_fraction = row.valid_samples / max(1, row.samples);
row.duration_s = maxFinite(t(valid));

convIdx = firstConvergedIndex(t, valid, posErr, yawErrRad, config);
if ~isempty(convIdx)
    row.converged = true;
    row.convergence_time_s = t(convIdx);
    analysisStart = row.convergence_time_s;
else
    row.converged = false;
    row.convergence_time_s = NaN;
    analysisStart = config.skipFirstS;
end

analysisMask = valid & t >= analysisStart;
if nnz(analysisMask) < config.minSamplesAfterConvergence
    analysisMask = valid & t >= config.skipFirstS;
end
if nnz(analysisMask) < config.minSamplesAfterConvergence
    analysisMask = valid;
end

if any(analysisMask)
    row.position_error_median_m = medianFinite(posErr(analysisMask));
    row.position_error_p95_m = percentileLocal(posErr(analysisMask), 95);
    row.yaw_error_median_rad = medianFinite(yawErrRad(analysisMask));
    row.yaw_error_p95_rad = percentileLocal(yawErrRad(analysisMask), 95);
    row.yaw_error_median_deg = rad2deg(row.yaw_error_median_rad);
    row.yaw_error_p95_deg = rad2deg(row.yaw_error_p95_rad);
    row.reason = "ok";
else
    row.reason = "no_valid_samples";
end
end

function idx = firstConvergedIndex(t, valid, posErr, yawErrRad, config)
idx = [];
good = valid & posErr <= config.xyThresholdM & yawErrRad <= config.yawThresholdRad;
candidateIdx = find(good(:)');
for k = candidateIdx
    windowEnd = t(k) + config.stableWindowS;
    window = t >= t(k) & t <= windowEnd;
    if ~any(window)
        continue;
    end
    lastT = max(t(window), [], 'omitnan');
    if lastT < windowEnd - config.windowToleranceS
        continue;
    end
    if all(good(window))
        idx = k;
        return;
    end
end
end

function plotMetricBoxplot(summary, percents, labels, metricName, yLabelText, outputDir, fileStem, showPlot, specialMask)
if nargin < 9 || isempty(specialMask)
    specialMask = false(height(summary), 1);
end

values = summary.(metricName);
groups = summary.injection_percent;
finite = isfinite(values) & isfinite(groups);
values = values(finite);
groups = groups(finite);
specialMask = specialMask(finite);
if isempty(values)
    warning('No finite values for %s; skipping figure.', metricName);
    return;
end

present = false(size(percents));
for i = 1:numel(percents)
    present(i) = any(groups == percents(i));
end
presentPercents = percents(present);
presentLabels = labels(present);

groupIndex = nan(size(groups));
for i = 1:numel(presentPercents)
    groupIndex(groups == presentPercents(i)) = i;
end
keep = isfinite(groupIndex);
values = values(keep);
groupIndex = groupIndex(keep);
specialMask = specialMask(keep);
if isempty(values)
    warning('No values remain after grouping for %s; skipping figure.', metricName);
    return;
end

visible = 'on';
if ~showPlot
    visible = 'off';
end
fig = figure('Visible', visible, 'Color', 'w', 'Units', 'centimeters', ...
    'Position', [2, 2, 14.5, 8.2]);
ax = axes(fig);
boxplot(values, groupIndex, 'Labels', cellstr(presentLabels), 'Symbol', '', ...
    'Widths', 0.55, 'Colors', [0.10, 0.10, 0.10]);
hold(ax, 'on');
rng(7);
jitter = (rand(size(groupIndex)) - 0.5) * 0.14;
normalMask = ~specialMask;
hRuns = scatter(ax, groupIndex(normalMask) + jitter(normalMask), ...
    values(normalMask), 22, [0.05, 0.25, 0.45], ...
    'filled', 'MarkerFaceAlpha', 0.65, 'MarkerEdgeAlpha', 0.65, ...
    'DisplayName', 'runs');
legendHandles = hRuns;
legendLabels = {'runs'};
if any(specialMask)
    hSpecial = scatter(ax, groupIndex(specialMask) + jitter(specialMask), ...
        values(specialMask), 36, [0.70, 0.10, 0.10], 'x', ...
        'LineWidth', 1.2, 'DisplayName', 'run ended before convergence');
    legendHandles(end + 1) = hSpecial; %#ok<AGROW>
    legendLabels{end + 1} = 'run ended before convergence'; %#ok<AGROW>
end
grid(ax, 'on');
ax.GridAlpha = 0.22;
ax.Box = 'off';
ax.FontName = 'Arial';
ax.FontSize = 11;
ax.LineWidth = 0.9;
xlabel(ax, 'particle injection');
ylabel(ax, yLabelText);
xlim(ax, [0.5, numel(presentLabels) + 0.5]);
legend(ax, legendHandles, legendLabels, 'Location', 'best', 'Box', 'off');

boxes = findobj(ax, 'Tag', 'Box');
medians = findobj(ax, 'Tag', 'Median');
whiskers = findobj(ax, 'Tag', 'Whisker');
set([boxes; medians; whiskers], 'LineWidth', 1.1);

saveFigure(fig, outputDir, fileStem, showPlot);
end

function saveFigure(fig, outputDir, fileStem, showPlot)
pngPath = fullfile(outputDir, [fileStem, '.png']);
drawnow;
try
    exportgraphics(fig, pngPath, 'Resolution', 450);
catch
    print(fig, pngPath, '-dpng', '-r450');
end
if ~showPlot && isgraphics(fig, 'figure')
    close(fig);
end
end

function value = numericValue(raw)
if isnumeric(raw)
    value = double(raw);
elseif iscell(raw)
    value = str2double(string(raw{1}));
else
    value = str2double(string(raw));
end
if isempty(value)
    value = NaN;
end
end

function values = numericColumn(T, name)
raw = T.(name);
if isnumeric(raw)
    values = double(raw);
elseif iscell(raw)
    values = str2double(string(raw));
else
    values = str2double(string(raw));
end
end

function T = readCsvTable(path)
try
    T = readtable(path, 'FileType', 'text', 'Delimiter', ',', ...
        'ReadVariableNames', true, 'VariableNamingRule', 'preserve', ...
        'TextType', 'string');
catch
    opts = detectImportOptions(path, 'FileType', 'text', 'Delimiter', ',');
    try
        opts.VariableNamingRule = 'preserve';
    catch
    end
    T = readtable(path, opts);
end
end

function y = wrapToPiLocal(x)
y = atan2(sin(x), cos(x));
end

function p = percentileLocal(values, pct)
values = sort(values(isfinite(values)));
if isempty(values)
    p = NaN;
    return;
end
if numel(values) == 1
    p = values;
    return;
end
rank = 1 + (pct / 100) * (numel(values) - 1);
lo = floor(rank);
hi = ceil(rank);
if lo == hi
    p = values(lo);
else
    p = values(lo) + (rank - lo) * (values(hi) - values(lo));
end
end

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = median(values);
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

function row = emptySummaryRow()
row = struct( ...
    'condition', "", ...
    'label', "", ...
    'injection_ratio', NaN, ...
    'injection_percent', NaN, ...
    'run', NaN, ...
    'run_dir', "", ...
    'csv_path', "", ...
    'returncode', NaN, ...
    'status_reason', "", ...
    'status_laps', NaN, ...
    'reason', "", ...
    'samples', NaN, ...
    'valid_samples', NaN, ...
    'valid_fraction', NaN, ...
    'duration_s', NaN, ...
    'converged', false, ...
    'convergence_time_s', NaN, ...
    'position_error_median_m', NaN, ...
    'position_error_p95_m', NaN, ...
    'yaw_error_median_rad', NaN, ...
    'yaw_error_p95_rad', NaN, ...
    'yaw_error_median_deg', NaN, ...
    'yaw_error_p95_deg', NaN);
end

function runRoot = latestInjectionRun(parentDir)
matches = dir(fullfile(parentDir, 'gpu_amcl_injection_*'));
matches = matches([matches.isdir]);
if isempty(matches)
    error('No gpu_amcl_injection_* runs found below %s.', parentDir);
end
[~, idx] = max([matches.datenum]);
runRoot = fullfile(matches(idx).folder, matches(idx).name);
end
