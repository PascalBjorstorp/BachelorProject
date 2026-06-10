clc;
close all;
% Compare the real OptiTrack AMCL/EKF error against simulation error in a
% clean map region. This is a spatial ROI consistency check, not a replay of
% the same raceline.
%
% Important: only use this after the simulation has been run in the same map
% frame as the OptiTrack bag. Use extractBagMapAndRaceline.m to extract that
% map from the bag first.

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
repoRoot = fileparts(fileparts(fileparts(scriptDir)));

plotFunctionsDir = fullfile(scriptDir, 'matlab plotting functions');
if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    error('Could not find plotting functions directory: %s', plotFunctionsDir);
end

if ~exist('showPlots', 'var') || isempty(showPlots)
    showPlots = false;
end
if ~exist('realBagRootDir', 'var') || isempty(realBagRootDir)
    realBagRootDir = fullfile(scriptDir, '..', 'bags', 'OptitrackBags', 'AMCL');
end
if ~exist('simRunRootDir', 'var') || isempty(simRunRootDir)
    simRunRootDir = "";
end
if ~exist('outputDir', 'var') || isempty(outputDir)
    outputDir = fullfile(scriptDir, 'plots', 'RealVsSimAmclRoi');
end
if ~exist('reportImageDir', 'var') || isempty(reportImageDir)
    reportImageDir = fullfile(repoRoot, 'Report', 'Sections', 'Localization', 'AMCL', 'Images', 'Test');
end
if ~exist('issueCsvPath', 'var') || isempty(issueCsvPath)
    issueCsvPath = fullfile(scriptDir, 'plots', 'OptiTrackFailureMap', ...
        'OptiTrack_Failure_Map_Samples.csv');
end
if ~exist('simLocalizers', 'var') || isempty(simLocalizers)
    simLocalizers = "gpu";
end
if ~exist('realParticleCountsToUse', 'var')
    realParticleCountsToUse = [];
end
if ~exist('minSimSpeedMps', 'var') || isempty(minSimSpeedMps)
    minSimSpeedMps = 0.2;
end
if ~exist('assumeSharedMapFrame', 'var') || isempty(assumeSharedMapFrame)
    assumeSharedMapFrame = false;
end

roi = struct();
roi.minX = 0.0;
roi.maxX = 8.0;
roi.minY = -1.4;
roi.maxY = inf;
issueTimeToleranceS = 1 / 240;

ensureDirectory(outputDir);
ensureDirectory(reportImageDir);

fprintf('Real bag root : %s\n', realBagRootDir);
fprintf('Sim run root  : %s\n', char(string(simRunRootDir)));
fprintf('ROI           : %.1f <= x <= %.1f m, y >= %.1f m\n', ...
    roi.minX, roi.maxX, roi.minY);
fprintf('Sim localizer : %s\n', strjoin(string(simLocalizers), ', '));

if strlength(string(simRunRootDir)) == 0 || ~isfolder(simRunRootDir)
    error(['Set simRunRootDir to a simulation run generated with the extracted OptiTrack bag map. ', ...
        'Use extractBagMapAndRaceline.m first, then run the simulator with the extracted bag_map.yaml.']);
end
if ~assumeSharedMapFrame
    error(['Refusing to compare raw x/y with assumeSharedMapFrame=false. ', ...
        'Run the simulation with the extracted bag map/raceline, then set assumeSharedMapFrame=true.']);
end

realConfig = defaultRealOptiTrackConfig();
realResults = loadOptiTrackBenchmarkBags(realBagRootDir, realConfig);
realSamples = collectRealRoiSamples(realResults, roi, issueCsvPath, ...
    realParticleCountsToUse, issueTimeToleranceS);
simSamples = collectSimRoiSamples(simRunRootDir, simLocalizers, roi, minSimSpeedMps);

sampleTable = [realSamples; simSamples];
if height(sampleTable) == 0
    error('No ROI samples survived filtering.');
end

summaryTable = summarizeRoiSamples(sampleTable);

samplePath = fullfile(outputDir, 'Real_vs_Sim_AMCL_ROI_Samples.csv');
summaryPath = fullfile(outputDir, 'Real_vs_Sim_AMCL_ROI_Summary.csv');
writetable(sampleTable, samplePath);
writetable(summaryTable, summaryPath);

figurePath = plotRoiComparison(sampleTable, outputDir, reportImageDir, showPlots);

fprintf('\nSample CSV  : %s\n', samplePath);
fprintf('Summary CSV : %s\n', summaryPath);
fprintf('Figure      : %s\n', figurePath);
disp(summaryTable);

function config = defaultRealOptiTrackConfig()
config = struct();
config.ekfTopic = '/ekf_pose';
config.optitrackTopic = '/vrpn_mocap/Car2/pose';
config.staticTfTopic = '/tf_static';
config.mapTopic = '/map';
config.mapFrame = 'map';
config.optitrackFrame = 'world';
config.skipStartupAndIncompleteLaps = true;

config.lapCloseRadiusM = 0.45;
config.lapRearmRadiusM = 0.90;
config.minLapDurationS = 2.0;
config.minLapDistanceM = 2.0;

config.optitrackDropoutZeroRadiusM = 1e-6;
config.optitrackFreezeDistanceM = 1e-6;
config.optitrackMaxSpeedMps = 12.0;

config.startCalibrationEnabled = true;
config.startCalibrationDurationS = 3.0;
config.startCalibrationMinSamples = 50;
config.startCalibrationMaxStdM = 0.03;
config.startCalibrationMaxTravelM = 0.05;

config.optitrackFreezeZoneExcludeEnabled = true;
config.optitrackFreezeZoneWindowS = 0.05;
config.optitrackFreezeZoneMaxGtTravelM = 0.02;
config.optitrackFreezeZoneMinEkfTravelM = 0.10;
config.optitrackFreezeZonePaddingS = 0.05;

config.excludeYawOutlierLaps = true;
config.yawOutlierLapThresholdRad = pi / 2;
config.yawOutlierLapParticleCounts = 100;

config.yawIsolatedOutlierFilterEnabled = true;
config.yawIsolatedOutlierThresholdRad = pi / 2;
config.yawIsolatedOutlierMaxRunLength = 2;

% Keep the full requested ROI. OptiTrack suspect samples are filtered by the
% failure-map CSV instead of using the older x >= 2 m metric crop.
config.metricExcludeXLessThanM = -inf;
end

function samples = collectRealRoiSamples(results, roi, issueCsvPath, particleCountsToUse, toleranceS)
samples = emptySampleTable();
issueTable = loadOptiTrackIssues(issueCsvPath);
totalBase = 0;
totalIssueRejected = 0;

for i = 1:numel(results)
    r = results(i);
    if ~isempty(particleCountsToUse) && ...
            ~any(double(r.particleCount) == double(particleCountsToUse(:)))
        continue;
    end

    gtXY = r.gtPos(:, 1:2);
    baseMask = logical(r.metricMask(:)) & roiMask(gtXY, roi) & ...
        isfinite(r.xyError(:)) & isfinite(r.yawError(:));

    issueMask = false(numel(baseMask), 1);
    if ~isempty(issueTable) && ismember('bag', issueTable.Properties.VariableNames)
        bagRows = strcmp(string(issueTable.group), "AMCL") & ...
            strcmp(string(issueTable.bag), string(r.bagName));
        issueTimes = double(issueTable.t_rel_s(bagRows));
        issueMask = markTimesNear(r.tRel(:), issueTimes, toleranceS);
    end

    keepMask = baseMask & ~issueMask;
    totalBase = totalBase + nnz(baseMask);
    totalIssueRejected = totalIssueRejected + nnz(baseMask & issueMask);

    idx = find(keepMask);
    if isempty(idx)
        continue;
    end

    samples = [samples; table( ...
        repmat("Real OptiTrack ROI", numel(idx), 1), ...
        repmat(string(r.bagName), numel(idx), 1), ...
        repmat(double(r.particleCount), numel(idx), 1), ...
        gtXY(idx, 1), gtXY(idx, 2), ...
        double(r.xyError(idx)) * 100.0, ...
        rad2deg(double(r.yawError(idx))), ...
        repmat(true, numel(idx), 1), ...
        repmat("OptiTrack", numel(idx), 1), ...
        repmat(totalIssueRejected, numel(idx), 1), ...
        'VariableNames', samples.Properties.VariableNames)]; %#ok<AGROW>
end

fprintf('Real ROI samples before issue filter: %d\n', totalBase);
fprintf('Real ROI samples rejected by OptiTrack plausibility flags: %d\n', totalIssueRejected);
end

function samples = collectSimRoiSamples(simRunRootDir, simLocalizers, roi, minSpeedMps)
samples = emptySampleTable();
simLocalizers = string(simLocalizers);

for i = 1:numel(simLocalizers)
    localizerName = simLocalizers(i);
    csvFiles = findSimGroundTruthCsvs(simRunRootDir, localizerName);
    if isempty(csvFiles)
        warning('No simulation ground-truth CSVs found for %s under %s', ...
            localizerName, simRunRootDir);
        continue;
    end

    for k = 1:numel(csvFiles)
        csvPath = csvFiles{k};
        T = readtable(csvPath, 'VariableNamingRule', 'preserve');
        required = {'gt_x', 'gt_y', 'err_xy', 'err_yaw'};
        assertColumns(T, required, csvPath);

        gtXY = [double(T.gt_x), double(T.gt_y)];
        positionErrorCm = double(T.err_xy) * 100.0;
        yawErrorDeg = rad2deg(wrapAnglePiLocal(double(T.err_yaw)));
        mask = roiMask(gtXY, roi) & isfinite(positionErrorCm) & isfinite(yawErrorDeg);

        if ismember('collision', T.Properties.VariableNames)
            mask = mask & double(T.collision) == 0;
        end
        if minSpeedMps > 0 && all(ismember({'gt_vx', 'gt_vy'}, T.Properties.VariableNames))
            speed = hypot(double(T.gt_vx), double(T.gt_vy));
            mask = mask & speed >= minSpeedMps;
        end

        idx = find(mask);
        if isempty(idx)
            continue;
        end

        runName = relativePathName(csvPath, simRunRootDir);
        samples = [samples; table( ...
            repmat("Simulation ROI", numel(idx), 1), ...
            repmat(string(runName), numel(idx), 1), ...
            repmat(NaN, numel(idx), 1), ...
            gtXY(idx, 1), gtXY(idx, 2), ...
            positionErrorCm(idx), yawErrorDeg(idx), ...
            repmat(false, numel(idx), 1), ...
            repmat("Sim ground truth", numel(idx), 1), ...
            repmat(0, numel(idx), 1), ...
            'VariableNames', samples.Properties.VariableNames)]; %#ok<AGROW>
    end
end
end

function T = emptySampleTable()
T = table('Size', [0 10], ...
    'VariableTypes', {'string', 'string', 'double', 'double', 'double', ...
    'double', 'double', 'logical', 'string', 'double'}, ...
    'VariableNames', {'source', 'run', 'particle_count', 'map_x_m', 'map_y_m', ...
    'position_error_cm', 'yaw_error_deg', 'is_real', 'reference', ...
    'real_optitrack_issue_samples_rejected'});
end

function summary = summarizeRoiSamples(samples)
groups = unique(samples.source, 'stable');
summary = table('Size', [0 12], ...
    'VariableTypes', {'string', 'double', 'double', 'double', 'double', ...
    'double', 'double', 'double', 'double', 'double', 'double', 'double'}, ...
    'VariableNames', {'source', 'n_samples', 'n_runs', 'median_position_cm', ...
    'mean_position_cm', 'p95_position_cm', 'rmse_position_cm', ...
    'median_abs_yaw_deg', 'mean_abs_yaw_deg', 'p95_abs_yaw_deg', ...
    'rmse_yaw_deg', 'real_optitrack_issue_samples_rejected'});

for i = 1:numel(groups)
    mask = samples.source == groups(i);
    pos = samples.position_error_cm(mask);
    yawAbs = abs(samples.yaw_error_deg(mask));
    row = table(groups(i), nnz(mask), numel(unique(samples.run(mask))), ...
        median(pos, 'omitnan'), mean(pos, 'omitnan'), prctile(pos, 95), ...
        sqrt(mean(pos .^ 2, 'omitnan')), median(yawAbs, 'omitnan'), ...
        mean(yawAbs, 'omitnan'), prctile(yawAbs, 95), ...
        sqrt(mean(yawAbs .^ 2, 'omitnan')), ...
        max(samples.real_optitrack_issue_samples_rejected(mask)), ...
        'VariableNames', summary.Properties.VariableNames);
    summary = [summary; row]; %#ok<AGROW>
end
end

function figurePath = plotRoiComparison(samples, outputDir, reportImageDir, showPlots)
visibleState = 'off';
if showPlots
    visibleState = 'on';
end

sourceOrder = ["Real OptiTrack ROI", "Simulation ROI"];
group = categorical(samples.source, sourceOrder, sourceOrder);

fig = figure('Color', 'w', 'Visible', visibleState, 'Position', [100 100 1200 460]);
tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

ax1 = nexttile;
boxplot(ax1, samples.position_error_cm, group, 'Symbol', 'k+');
grid(ax1, 'on');
ylabel(ax1, 'position error [cm]');
title(ax1, 'Position error in ROI');

ax2 = nexttile;
boxplot(ax2, abs(samples.yaw_error_deg), group, 'Symbol', 'k+');
grid(ax2, 'on');
ylabel(ax2, 'absolute yaw error [deg]');
title(ax2, 'Yaw error in ROI');

figurePath = fullfile(outputDir, 'amcl_real_vs_sim_roi_error.png');
exportgraphics(fig, figurePath, 'Resolution', 300);
copyfile(figurePath, fullfile(reportImageDir, 'amcl_real_vs_sim_roi_error.png'));
if ~showPlots
    close(fig);
end
end

function issueTable = loadOptiTrackIssues(issueCsvPath)
issueTable = table();
if isfile(issueCsvPath)
    issueTable = readtable(issueCsvPath, 'VariableNamingRule', 'preserve');
end
end

function mask = roiMask(xy, roi)
mask = all(isfinite(xy), 2) & ...
    xy(:, 1) >= roi.minX & xy(:, 1) <= roi.maxX & ...
    xy(:, 2) >= roi.minY & xy(:, 2) <= roi.maxY;
end

function nearMask = markTimesNear(t, issueTimes, toleranceS)
nearMask = false(numel(t), 1);
issueTimes = sort(issueTimes(isfinite(issueTimes(:))));
if isempty(issueTimes)
    return;
end

j = 1;
for i = 1:numel(t)
    if ~isfinite(t(i))
        continue;
    end
    while j <= numel(issueTimes) && issueTimes(j) < t(i) - toleranceS
        j = j + 1;
    end
    if j <= numel(issueTimes) && issueTimes(j) <= t(i) + toleranceS
        nearMask(i) = true;
    end
end
end

function csvFiles = findSimGroundTruthCsvs(simRunRootDir, localizerName)
localizerDir = fullfile(simRunRootDir, char(localizerName));
if ~isfolder(localizerDir)
    localizerDir = simRunRootDir;
end

files = dir(fullfile(localizerDir, '**', 'groundtruth_at_ekf.csv'));
if isempty(files)
    files = dir(fullfile(localizerDir, '**', 'AMCL_benchmark.csv'));
end
csvFiles = cell(numel(files), 1);
for i = 1:numel(files)
    csvFiles{i} = fullfile(files(i).folder, files(i).name);
end
csvFiles = sort(csvFiles);
end

function assertColumns(T, names, csvPath)
missing = setdiff(names, T.Properties.VariableNames);
if ~isempty(missing)
    error('Missing required column(s) in %s: %s', csvPath, strjoin(missing, ', '));
end
end

function runName = relativePathName(pathName, rootDir)
pathName = char(pathName);
rootDir = stripTrailingSeparator(char(rootDir));
prefix = [rootDir filesep];
if startsWith(pathName, prefix)
    runName = pathName((numel(prefix) + 1):end);
else
    runName = pathName;
end
runName = regexprep(runName, [filesep 'groundtruth_at_ekf.csv$'], '');
runName = regexprep(runName, [filesep 'AMCL_benchmark.csv$'], '');
end

function ensureDirectory(pathName)
if ~isfolder(pathName)
    mkdir(pathName);
end
end

function a = wrapAnglePiLocal(a)
a = mod(a + pi, 2 * pi) - pi;
end

function out = stripTrailingSeparator(in)
out = char(in);
while numel(out) > 1 && (out(end) == filesep || out(end) == '/' || out(end) == '\')
    out(end) = [];
end
end
