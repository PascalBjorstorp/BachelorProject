function plot_algorithm_speed_maps(bagsRoot, racelineRoot, mapRoot, plotsRootDir, showPlots)
clc;
close all;
% PLOT_ALGORITHM_SPEED_MAPS
% Plot one map per algorithm/run with:
% - Occupancy map from /map
% - Reference raceline from CSV
% - EKF trajectory colored by speed from /ego_racecar/odom
%
% Defaults target the repository NormalMap benchmark folders. Optional inputs:
%   bagsRoot     - folder with bag run folders, default bags/NormalMap/SpeedBags
%   racelineRoot - folder with raceline CSV, default bags/NormalMap/Raceline
%   mapRoot      - folder with ROS map YAML/PGM, default bags/NormalMap/Map
%   plotsRootDir - output root, default Matlab/plots/NormalMapReports/SpeedMaps
%   showPlots    - true to keep figures open, false to save only

%% USER INPUT
scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(fileparts(scriptDir)));
normalMapRoot = fullfile(repoRoot, 'bags', 'NormalMap');
if ~isfolder(normalMapRoot)
    normalMapRoot = fullfile(pwd, 'bags', 'NormalMap');
end

% Path where SpeedBags bag directories are stored.
if nargin < 1 || isempty(bagsRoot)
    bagsRoot = fullfile(normalMapRoot, 'SpeedBags');
end

% Raceline CSV folder for NormalMap.
if nargin < 2 || isempty(racelineRoot)
    racelineRoot = fullfile(normalMapRoot, 'Raceline');
end

% Map YAML/PGM folder for NormalMap. Used when bag has no /map topic.
if nargin < 3 || isempty(mapRoot)
    mapRoot = fullfile(normalMapRoot, 'Map');
end

% Output plots and summary tables.
if nargin < 4 || isempty(plotsRootDir)
    plotsRootDir = fullfile(scriptDir, 'plots', 'NormalMapReports', 'SpeedMaps');
end
if nargin < 5 || isempty(showPlots)
    showPlots = false;
end
ensureDirectoryLocal(plotsRootDir);

% Default raceline CSV (used for all discovered runs unless overridden).
defaultRacelineCsv = findDefaultRacelineCsv(racelineRoot);
defaultMapData = loadMapDataFromFolder(mapRoot);

% Lap filtering: skip the first completed lap and ignore data after the
% final completed lap crossing.
ignoreFirstCompleteLap = true;

% Lap detection setting (for full-lap-only metrics).
minLapTimeSeconds = 5.0;

% Display trajectory start/end markers on plots.
showStartEndMarkers = false;

runs = discoverSpeedLapRuns(bagsRoot, racelineRoot, defaultRacelineCsv);

%% Validate configuration
if ~isfolder(bagsRoot)
    error('SpeedBags bag root does not exist: %s', bagsRoot);
end
if isempty(defaultRacelineCsv) || ~isfile(defaultRacelineCsv)
    error('No raceline CSV found in: %s', racelineRoot);
end
if isempty(runs)
    error('No ROS 2 bag folders with metadata.yaml found under: %s', bagsRoot);
end
fprintf('SpeedBags bag root : %s\n', bagsRoot);
fprintf('Raceline CSV       : %s\n', defaultRacelineCsv);
fprintf('Map folder         : %s\n', mapRoot);
fprintf('Discovered runs    : %d\n', numel(runs));
fprintf('Output folder      : %s\n', plotsRootDir);

%% Load and process all runs
results = struct( ...
    'algorithm', {}, ...
    'bagName', {}, ...
    'bagPath', {}, ...
    'lapCount', {}, ...
    'lapMeanS', {}, ...
    'lapBestS', {}, ...
    'speedMean', {}, ...
    'speedMax', {}, ...
    'devMean', {}, ...
    'devMax', {}, ...
    'mapData', {}, ...
    'racelineXY', {}, ...
    'trajX', {}, ...
    'trajY', {}, ...
    'trajSpeed', {} ...
    );

for i = 1:numel(runs)
    cfg = runs(i);
    bagPath = fullfile(bagsRoot, cfg.bagName);
    if ~isfolder(bagPath)
        warning('Bag folder not found: %s', bagPath);
        continue;
    end
    if ~isfile(fullfile(bagPath, 'metadata.yaml'))
        warning('metadata.yaml missing in bag folder: %s', bagPath);
        continue;
    end

    try
        bag = ros2bagreader(bagPath);
    catch ME
        warning('Failed to open bag %s: %s', cfg.bagName, ME.message);
        continue;
    end

    allTopics = getTopicNames(bag.AvailableTopics);
    ekfTopic = pickTopic(allTopics, '/ekf_pose', 'ekf_pose');
    odomTopic = pickTopic(allTopics, '/ego_racecar/odom', 'odom');
    mapTopic = pickTopic(allTopics, '/map', 'map');

    if isempty(ekfTopic)
        warning('No ekf pose topic found in %s', cfg.bagName);
        continue;
    end
    if isempty(odomTopic)
        warning('No odom topic found in %s', cfg.bagName);
        continue;
    end

    ekfMsgs = readMessages(select(bag, 'Topic', ekfTopic));
    odomMsgs = readMessages(select(bag, 'Topic', odomTopic));
    mapMsgs = {};
    if ~isempty(mapTopic)
        mapMsgs = readMessages(select(bag, 'Topic', mapTopic));
    end

    [tEkf, xyEkf] = extractXYSeries(ekfMsgs);
    [tOdom, speedOdom] = extractOdomSpeedSeries(odomMsgs);

    if numel(tEkf) < 2 || numel(tOdom) < 2
        warning('Not enough ekf/odom data in %s', cfg.bagName);
        continue;
    end

    speedAtEkf = interp1(tOdom, speedOdom, tEkf, 'linear', 'extrap');
    valid = isfinite(xyEkf(:, 1)) & isfinite(xyEkf(:, 2)) & isfinite(speedAtEkf);
    tEkf = tEkf(valid);
    xyEkf = xyEkf(valid, :);
    speedAtEkf = speedAtEkf(valid);

    if numel(tEkf) < 2
        warning('No valid aligned trajectory samples in %s', cfg.bagName);
        continue;
    end

    raceCsv = cfg.racelineCsv;
    if isempty(raceCsv)
        raceCsv = defaultRacelineCsv;
    end
    racelineXY = loadRacelineXY(raceCsv);

    [lapDurS, lapCrossingsS] = detectFullLapDurations( ...
        tEkf, xyEkf(:, 1), xyEkf(:, 2), racelineXY, minLapTimeSeconds);

    if isempty(lapDurS)
        warning('No full laps detected in %s.', cfg.bagName);
        continue;
    end

    if ignoreFirstCompleteLap
        if numel(lapDurS) < 2 || numel(lapCrossingsS) < 3
            warning('Not enough full laps in %s to ignore first lap.', cfg.bagName);
            continue;
        end
        lapDurS = lapDurS(2:end);
        lapCrossingsS = lapCrossingsS(2:end);
    end

    keepFullLaps = tEkf >= lapCrossingsS(1) & tEkf <= lapCrossingsS(end);
    if nnz(keepFullLaps) < 2
        warning('Full-lap window leaves too few samples in %s', cfg.bagName);
        continue;
    end
    tEkf = tEkf(keepFullLaps);
    xyEkf = xyEkf(keepFullLaps, :);
    speedAtEkf = speedAtEkf(keepFullLaps);

    mapData = [];
    if ~isempty(mapMsgs)
        mapData = decodeOccupancyGridMsg(mapMsgs{end});
    end
    if isempty(mapData)
        mapData = defaultMapData;
    end

    trajDev = computeDeviationToRaceline(xyEkf(:, 1), xyEkf(:, 2), racelineXY);

    r = struct();
    r.algorithm = cfg.algorithm;
    r.bagName = cfg.bagName;
    r.bagPath = bagPath;
    r.lapCount = numel(lapDurS);
    r.lapMeanS = mean(lapDurS);
    r.lapBestS = min(lapDurS);
    r.speedMean = mean(speedAtEkf, 'omitnan');
    r.speedMax = max(speedAtEkf);
    r.devMean = mean(trajDev, 'omitnan');
    r.devMax = max(trajDev);
    r.mapData = mapData;
    r.racelineXY = racelineXY;
    r.trajX = xyEkf(:, 1);
    r.trajY = xyEkf(:, 2);
    r.trajSpeed = speedAtEkf;
    results(end + 1) = r; %#ok<AGROW>
end

if isempty(results)
    error('No valid runs were processed. Check runs configuration and topics.');
end

%% Plot one map panel per algorithm
speedAll = vertcat(results.trajSpeed);
cMin = min(speedAll);
cMax = max(speedAll);
if ~isfinite(cMin) || ~isfinite(cMax) || cMin == cMax
    cMin = 0;
    cMax = max(1, cMax + 1);
end

fig = figure('Name', 'Algorithm Trajectory Speed Maps', 'Color', 'w', ...
    'Visible', figureVisibility(showPlots));
tiledlayout(numel(results), 1, 'Padding', 'compact', 'TileSpacing', 'compact');
speedCmap = redToGreenMap(256);

for i = 1:numel(results)
    nexttile;
    hold on;
    axis equal;
    grid off;
    box on;

    m = results(i).mapData;
    if ~isempty(m)
        image('XData', m.xWorld, 'YData', m.yWorld, 'CData', m.rgb);
        set(gca, 'YDir', 'normal');
    end

    rl = results(i).racelineXY;
    if ~isempty(rl)
        plot(rl(:, 1), rl(:, 2), 'b--', 'LineWidth', 1.8, 'DisplayName', 'raceline');
    end

    scatter(results(i).trajX, results(i).trajY, 10, results(i).trajSpeed, 'filled', ...
        'MarkerFaceAlpha', 0.9, 'DisplayName', 'trajectory (speed-colored)');

    colormap(gca, speedCmap);

    if showStartEndMarkers
        plot(results(i).trajX(1), results(i).trajY(1), 'go', 'MarkerFaceColor', 'g', ...
            'DisplayName', 'start');
        plot(results(i).trajX(end), results(i).trajY(end), 'ro', 'MarkerFaceColor', 'r', ...
            'DisplayName', 'end');
    end

    caxis([cMin, cMax]);
    set(gca, 'XTick', [], 'YTick', []);
    addScaleBarBelowAxes(fig, gca, 1.0, '1 m');
    title(sprintf('%s | %s | laps %d | best %.3f s', ...
        results(i).algorithm, results(i).bagName, results(i).lapCount, results(i).lapBestS), ...
        'Interpreter', 'none');
    hold off;
end

cb = colorbar;
cb.Layout.Tile = 'north';
cb.Label.String = 'Speed [m/s]';

%% Combined summary table for all algorithms
alg = cellstr(string({results.algorithm})');
bag = cellstr(string({results.bagName})');
lapCount = [results.lapCount]';
lapMean = [results.lapMeanS]';
lapBest = [results.lapBestS]';
vMean = [results.speedMean]';
vMax = [results.speedMax]';
devMean = [results.devMean]';
devMax = [results.devMax]';

summaryT = table(alg, bag, lapCount, lapMean, lapBest, vMean, vMax, devMean, devMax, ...
    'VariableNames', {'Algorithm', 'BagName', 'LapCount', 'LapMean_s', 'LapBest_s', ...
    'SpeedMean_mps', 'SpeedMax_mps', ...
    'DevMean_m', 'DevMax_m'});

disp(' ');
disp('=== Combined Summary Across Algorithms ===');
disp(summaryT);

summaryPath = fullfile(plotsRootDir, 'Speed_Map_Summary.csv');
writetable(summaryT, summaryPath);

saveFigureLocal(fig, plotsRootDir, 'Algorithm_Trajectory_Speed_Maps', showPlots);

figTable = renderSummaryTableFigure(summaryT, showPlots);
saveFigureLocal(figTable, plotsRootDir, 'Algorithm_Summary_Table', showPlots);

fprintf('Summary saved to %s\n', summaryPath);
fprintf('Figures saved to %s\n', plotsRootDir);

	end

	function csvPath = findDefaultRacelineCsv(racelineRoot)
	csvPath = '';
	if ~isfolder(racelineRoot)
	    return;
	end

	preferred = fullfile(racelineRoot, 'my_track_raceline.csv');
	if isfile(preferred)
	    csvPath = preferred;
	    return;
	end

	candidates = dir(fullfile(racelineRoot, '*.csv'));
	if isempty(candidates)
	    return;
	end

	[~, idx] = max([candidates.datenum]);
	csvPath = fullfile(candidates(idx).folder, candidates(idx).name);
	end

	function runs = discoverSpeedLapRuns(bagsRoot, racelineRoot, defaultRacelineCsv)
	runs = struct('algorithm', {}, 'bagName', {}, 'racelineCsv', {});
	if ~isfolder(bagsRoot)
	    return;
	end

	metadataFiles = findFilesRecursive(bagsRoot, 'metadata.yaml');
	for k = 1:numel(metadataFiles)
	    bagPath = fileparts(metadataFiles{k});
	    if isSameOrChildPath(bagPath, racelineRoot)
	        continue;
	    end

	    bagName = relativePathFromRoot(bagPath, bagsRoot);
	    if isempty(bagName)
	        [~, bagName] = fileparts(bagPath);
	    end

	    runs(end + 1) = struct( ... %#ok<AGROW>
	        'algorithm', inferAlgorithmFromBagName(bagName), ...
	        'bagName', bagName, ...
	        'racelineCsv', defaultRacelineCsv);
	end
	end

	function files = findFilesRecursive(rootDir, fileName)
	files = {};
	if ~isfolder(rootDir)
	    return;
	end

	entries = dir(rootDir);
	for i = 1:numel(entries)
	    entry = entries(i);
	    if entry.isdir
	        if strcmp(entry.name, '.') || strcmp(entry.name, '..')
	            continue;
	        end
	        childFiles = findFilesRecursive(fullfile(rootDir, entry.name), fileName);
	        files = [files, childFiles]; %#ok<AGROW>
	    elseif strcmp(entry.name, fileName)
	        files{end + 1} = fullfile(rootDir, entry.name); %#ok<AGROW>
	    end
	end
	end

	function tf = isSameOrChildPath(pathToCheck, rootPath)
	tf = false;
	if isempty(rootPath)
	    return;
	end
	pathToCheck = normalizeFolderPath(pathToCheck);
	rootPath = normalizeFolderPath(rootPath);
	tf = strcmp(pathToCheck, rootPath) || startsWith(pathToCheck, [rootPath filesep]);
	end

	function rel = relativePathFromRoot(pathToConvert, rootPath)
	pathToConvert = normalizeFolderPath(pathToConvert);
	rootPath = normalizeFolderPath(rootPath);
	prefix = [rootPath filesep];
	if startsWith(pathToConvert, prefix)
	    rel = pathToConvert((numel(prefix) + 1):end);
	elseif strcmp(pathToConvert, rootPath)
	    rel = '';
	else
	    [~, rel] = fileparts(pathToConvert);
	end
	end

	function out = normalizeFolderPath(in)
	if isempty(in)
	    out = '';
	    return;
	end
	out = char(in);
	while numel(out) > 1 && (out(end) == filesep || out(end) == '/' || out(end) == '\')
	    out(end) = [];
	end
	end

	function algorithm = inferAlgorithmFromBagName(bagName)
	nameLower = lower(strrep(char(bagName), filesep, '_'));
	if contains(nameLower, 'fpga')
	    algorithm = 'MPC FPGA';
	elseif contains(nameLower, 'mpc')
	    algorithm = 'MPC';
	elseif contains(nameLower, 'pure') || contains(nameLower, 'pp')
	    algorithm = 'Pure Pursuit';
	elseif contains(nameLower, 'stanley')
	    algorithm = 'Stanley';
	elseif contains(nameLower, 'ftg')
	    algorithm = 'FTG';
	else
	    algorithm = char(bagName);
	end
	end

	function names = getTopicNames(topicsTbl)
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

function topic = pickTopic(allTopics, preferred, fallbackContains)
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

function [t, xy] = extractXYSeries(msgs)
n = numel(msgs);
t = zeros(n, 1);
xy = nan(n, 2);

for k = 1:n
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end

    t(k) = extractHeaderTime(m, k);
    [x, y, ok] = extractPositionXY(m);
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

function [t, speed] = extractOdomSpeedSeries(msgs)
n = numel(msgs);
t = zeros(n, 1);
speed = nan(n, 1);

for k = 1:n
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end

    t(k) = extractHeaderTime(m, k);
    [vx, vy, ok] = extractLinearVelocityXY(m);
    if ok
        speed(k) = hypot(vx, vy);
    end
end

valid = isfinite(t) & isfinite(speed);
t = t(valid);
speed = speed(valid);

[t, idx] = sort(t);
speed = speed(idx);
[t, iUnique] = unique(t);
speed = speed(iUnique);
end

function [lapDurS, crossingTimes] = detectFullLapDurations(t, x, y, racelineXY, minLapTimeS)
lapDurS = [];
crossingTimes = [];

if numel(t) < 3 || isempty(racelineXY) || size(racelineXY, 1) < 2
    return;
end

[trackS, trackLength, closedRacelineXY] = buildRacelineArclength(racelineXY);
if trackLength <= 1e-9
    return;
end

progress = projectTrajectoryToRacelineProgress([x(:), y(:)], closedRacelineXY, trackS, trackLength);
t = t(:);
valid = isfinite(t) & isfinite(progress);
t = t(valid);
progress = progress(valid);

if numel(t) < 3
    return;
end

[t, sortIdx] = sort(t);
progress = progress(sortIdx);
[t, uniqueIdx] = unique(t, 'stable');
progress = progress(uniqueIdx);

unwrappedProgress = unwrapCircularProgress(progress, trackLength);
if unwrappedProgress(end) < unwrappedProgress(1)
    unwrappedProgress = -unwrappedProgress;
end

crossT = detectProgressCrossings(t, unwrappedProgress, trackLength);
crossingTimes = debounceCrossingTimes(crossT, minLapTimeS);

if numel(crossingTimes) < 2
    crossingTimes = [];
    return;
end

lapDurS = diff(crossingTimes);
valid = isfinite(lapDurS) & lapDurS >= minLapTimeS;
lapDurS = lapDurS(valid);
if isempty(lapDurS)
    crossingTimes = [];
end
end

function [trackS, trackLength, closedXY] = buildRacelineArclength(racelineXY)
closedXY = racelineXY(all(isfinite(racelineXY), 2), :);
trackS = [];
trackLength = 0;

if size(closedXY, 1) < 2
    return;
end

if hypot(closedXY(end, 1) - closedXY(1, 1), closedXY(end, 2) - closedXY(1, 2)) > 1e-6
    closedXY(end + 1, :) = closedXY(1, :);
end

segLen = hypot(diff(closedXY(:, 1)), diff(closedXY(:, 2)));
keepPoint = [true; segLen > 1e-9];
closedXY = closedXY(keepPoint, :);

if size(closedXY, 1) < 2
    return;
end

segLen = hypot(diff(closedXY(:, 1)), diff(closedXY(:, 2)));
trackS = [0; cumsum(segLen)];
trackLength = trackS(end);
end

function progress = projectTrajectoryToRacelineProgress(trajXY, racelineXY, trackS, trackLength)
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

function unwrappedProgress = unwrapCircularProgress(progress, trackLength)
phase = 2 * pi * progress(:) / trackLength;
unwrappedProgress = unwrap(phase) * trackLength / (2 * pi);
end

function crossingTimes = detectProgressCrossings(t, unwrappedProgress, trackLength)
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

function crossingTimes = debounceCrossingTimes(rawCrossings, minLapTimeS)
crossingTimes = [];
rawCrossings = sort(rawCrossings(:));
rawCrossings = rawCrossings(isfinite(rawCrossings));

for i = 1:numel(rawCrossings)
    if isempty(crossingTimes) || rawCrossings(i) - crossingTimes(end) >= minLapTimeS
        crossingTimes(end + 1, 1) = rawCrossings(i); %#ok<AGROW>
    end
end
end

function out = decodeOccupancyGridMsg(msg)
if ~isstruct(msg)
    msg = struct(msg);
end

[info, okInfo] = getFieldIgnoreCase(msg, 'info');
[dataVec, okData] = getFieldIgnoreCase(msg, 'data');
if ~okInfo || ~okData
    out = [];
    return;
end

[width, okW] = getNumericFieldIgnoreCase(info, 'width');
[height, okH] = getNumericFieldIgnoreCase(info, 'height');
[res, okR] = getNumericFieldIgnoreCase(info, 'resolution');
[origin, okOrigin] = getFieldIgnoreCase(info, 'origin');
if ~(okW && okH && okR && okOrigin)
    out = [];
    return;
end

[originPos, okPos] = getFieldIgnoreCase(origin, 'position');
if ~okPos
    out = [];
    return;
end

[x0, okX0] = getNumericFieldIgnoreCase(originPos, 'x');
[y0, okY0] = getNumericFieldIgnoreCase(originPos, 'y');
if ~(okX0 && okY0)
    out = [];
    return;
end

width = double(width);
height = double(height);
res = double(res);

dataVec = double(dataVec(:));
if numel(dataVec) < width * height
    out = [];
    return;
end
dataVec = dataVec(1:width * height);

gridOcc = reshape(dataVec, [width, height])';

% Build a light, clean RGB map background:
% free=white, occupied=dark gray, unknown=light gray.
rgb = ones(height, width, 3);
isUnknown = (gridOcc == -1);
isOccupied = (gridOcc >= 65);

for c = 1:3
    ch = rgb(:, :, c);
    ch(isUnknown) = 0.93;
    ch(isOccupied) = 0.12;
    rgb(:, :, c) = ch;
end

xWorld = x0 + (0:width-1) * res;
yWorld = y0 + (0:height-1) * res;

out = struct('rgb', rgb, 'xWorld', xWorld, 'yWorld', yWorld);
end

function cmap = redToGreenMap(n)
if nargin < 1
    n = 256;
end
n = max(2, floor(n));
r = linspace(1.0, 0.0, n)';
g = linspace(0.0, 0.75, n)';
b = zeros(n, 1);
cmap = [r g b];
end

function addScaleBarBelowAxes(fig, ax, barLenMeters, labelText)
if nargin < 1 || isempty(fig)
    fig = gcf;
end
if nargin < 2 || isempty(ax)
    ax = gca;
end
if nargin < 3 || isempty(barLenMeters)
    barLenMeters = 1.0;
end
if nargin < 4
    labelText = '1 m';
end

xl = xlim(ax);
dx = diff(xl);
if dx <= 0 || barLenMeters <= 0
    return;
end

axPos = ax.Position; % normalized figure units
barLenNorm = (barLenMeters / dx) * axPos(3);
barLenNorm = min(barLenNorm, 0.22 * axPos(3));
barLenNorm = max(barLenNorm, 0.05 * axPos(3));

x2 = axPos(1) + axPos(3) - 0.02;
x1 = x2 - barLenNorm;
y = max(0.01, axPos(2) - 0.03);

annotation(fig, 'line', [x1 x2], [y y], 'Color', 'k', 'LineWidth', 3);
annotation(fig, 'textbox', [x1, y + 0.004, barLenNorm, 0.02], ...
    'String', labelText, 'LineStyle', 'none', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', 'FontWeight', 'bold', 'Color', 'k');
end

function mapData = loadMapDataFromFolder(mapRoot)
mapData = [];
if isempty(mapRoot) || ~isfolder(mapRoot)
    return;
end

yamlFiles = dir(fullfile(mapRoot, '*.yaml'));
if isempty(yamlFiles)
    yamlFiles = dir(fullfile(mapRoot, '*.yml'));
end
if isempty(yamlFiles)
    return;
end

[~, idx] = max([yamlFiles.datenum]);
yamlPath = fullfile(yamlFiles(idx).folder, yamlFiles(idx).name);
txt = fileread(yamlPath);

imageName = parseYamlStringLocal(txt, 'image');
resolution = parseYamlScalarLocal(txt, 'resolution', NaN);
origin = parseYamlVectorLocal(txt, 'origin', [0, 0, 0]);
if isempty(imageName) || ~isfinite(resolution) || resolution <= 0
    return;
end

imagePath = imageName;
if ~isfile(imagePath)
    imagePath = fullfile(fileparts(yamlPath), imageName);
end
if ~isfile(imagePath)
    return;
end

try
    img = imread(imagePath);
catch ME
    warning('Failed to read map image %s: %s', imagePath, ME.message);
    return;
end

if ndims(img) == 3
    img = mean(double(img), 3);
else
    img = double(img);
end

img = flipud(img);
imgMin = min(img(:));
imgMax = max(img(:));
if isfinite(imgMin) && isfinite(imgMax) && imgMax > imgMin
    img = (img - imgMin) ./ (imgMax - imgMin);
else
    img = ones(size(img));
end

rgb = repmat(img, 1, 1, 3);
[height, width] = size(img);
xWorld = origin(1) + (0:width-1) * resolution;
yWorld = origin(2) + (0:height-1) * resolution;

mapData = struct('rgb', rgb, 'xWorld', xWorld, 'yWorld', yWorld);
end

function value = parseYamlStringLocal(txt, key)
value = '';
expr = ['(?m)^\s*', regexptranslate('escape', key), '\s*:\s*([^\r\n#]+)'];
tok = regexp(txt, expr, 'tokens', 'once');
if isempty(tok)
    return;
end
value = strtrim(tok{1});
value = regexprep(value, '^[''"]|[''"]$', '');
end

function value = parseYamlScalarLocal(txt, key, defaultValue)
value = defaultValue;
raw = parseYamlStringLocal(txt, key);
if isempty(raw)
    return;
end
num = sscanf(raw, '%f', 1);
if ~isempty(num)
    value = double(num);
end
end

function value = parseYamlVectorLocal(txt, key, defaultValue)
value = defaultValue;
raw = parseYamlStringLocal(txt, key);
if isempty(raw)
    return;
end
raw = regexprep(raw, '[\[\],]', ' ');
nums = sscanf(raw, '%f').';
if numel(nums) >= numel(defaultValue)
    value = nums(1:numel(defaultValue));
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

function visibility = figureVisibility(showPlots)
if showPlots
    visibility = 'on';
else
    visibility = 'off';
end
end

function saveFigureLocal(fig, outputDir, fileStem, showPlots)
ensureDirectoryLocal(outputDir);
set(fig, 'Color', 'w');
set(fig, 'InvertHardcopy', 'off');
set(fig, 'Units', 'pixels');
pos = get(fig, 'Position');
if pos(3) < 1600 || pos(4) < 900
    set(fig, 'Position', [pos(1), pos(2), 1800, 1000]);
end

pngPath = fullfile(outputDir, [fileStem, '.png']);
drawnow;
try
    exportgraphics(fig, pngPath, 'Resolution', 450);
catch
    print(fig, pngPath, '-dpng', '-r450');
end

if ~showPlots && isgraphics(fig, 'figure')
    close(fig);
end
end

function fig = renderSummaryTableFigure(summaryT, showPlots)
fig = figure('Name', 'Algorithm Summary Table', 'Color', 'w', ...
    'Visible', figureVisibility(showPlots));
ax = axes(fig);
axis(ax, 'off');

headers = {'Algorithm', 'Bag', 'Laps', 'Mean lap [s]', 'Best lap [s]', ...
    'Mean speed [m/s]', 'Max speed [m/s]', 'Mean dev [m]', 'Max dev [m]'};
rows = cell(height(summaryT), numel(headers));
for i = 1:height(summaryT)
    rows{i, 1} = char(summaryT.Algorithm{i});
    rows{i, 2} = char(summaryT.BagName{i});
    rows{i, 3} = sprintf('%d', summaryT.LapCount(i));
    rows{i, 4} = sprintf('%.3f', summaryT.LapMean_s(i));
    rows{i, 5} = sprintf('%.3f', summaryT.LapBest_s(i));
    rows{i, 6} = sprintf('%.3f', summaryT.SpeedMean_mps(i));
    rows{i, 7} = sprintf('%.3f', summaryT.SpeedMax_mps(i));
    rows{i, 8} = sprintf('%.3f', summaryT.DevMean_m(i));
    rows{i, 9} = sprintf('%.3f', summaryT.DevMax_m(i));
end

tableText = [strjoin(headers, '    '), newline, repmat('-', 1, 150), newline];
for i = 1:size(rows, 1)
    tableText = [tableText, sprintf('%-16s %-12s %5s %13s %12s %16s %15s %13s %11s\n', rows{i, :})]; %#ok<AGROW>
end

text(ax, 0.02, 0.92, 'Algorithm Speed Summary', ...
    'Units', 'normalized', 'FontWeight', 'bold', 'FontSize', 18, ...
    'Interpreter', 'none', 'VerticalAlignment', 'top');
text(ax, 0.02, 0.82, tableText, ...
    'Units', 'normalized', 'FontName', 'monospaced', 'FontSize', 12, ...
    'Interpreter', 'none', 'VerticalAlignment', 'top');
end

function xy = loadRacelineXY(csvPath)
xy = [];
if isempty(csvPath) || ~isfile(csvPath)
    return;
end

try
    T = readtable(csvPath, 'VariableNamingRule', 'preserve');
catch
    M = readmatrix(csvPath);
    if size(M, 2) >= 2
        xy = M(:, 1:2);
        xy = xy(all(isfinite(xy), 2), :);
    end
    return;
end

names = string(T.Properties.VariableNames);
low = lower(names);

idxX = find(contains(low, "x"), 1, 'first');
idxY = find(contains(low, "y"), 1, 'first');

if isempty(idxX) || isempty(idxY) || idxX == idxY
    numIdx = find(varfun(@isnumeric, T, 'OutputFormat', 'uniform'));
    if numel(numIdx) >= 2
        idxX = numIdx(1);
        idxY = numIdx(2);
    else
        return;
    end
end

xy = [double(T{:, idxX}), double(T{:, idxY})];
xy = xy(all(isfinite(xy), 2), :);
end

function t = extractHeaderTime(m, fallback)
t = fallback;
[header, hasHeader] = getFieldIgnoreCase(m, 'header');
if ~hasHeader || ~isstruct(header)
    return;
end
[stamp, hasStamp] = getFieldIgnoreCase(header, 'stamp');
if ~hasStamp || ~isstruct(stamp)
    return;
end

[secVal, hasSec] = getNumericFieldIgnoreCase(stamp, 'sec');
[nsecVal, hasNSec] = getNumericFieldIgnoreCase(stamp, 'nanosec');
if ~hasNSec
    [nsecVal, hasNSec] = getNumericFieldIgnoreCase(stamp, 'nsec');
end

if hasSec && hasNSec
    t = double(secVal) + double(nsecVal) * 1e-9;
elseif hasSec
    t = double(secVal);
end
end

function [x, y, ok] = extractPositionXY(m)
ok = false;
x = NaN;
y = NaN;

[poseField, hasPose] = getFieldIgnoreCase(m, 'pose');
poseStruct = [];
if hasPose && isstruct(poseField)
    [innerPose, hasInnerPose] = getFieldIgnoreCase(poseField, 'pose');
    if hasInnerPose && isstruct(innerPose)
        poseStruct = innerPose;
    else
        poseStruct = poseField;
    end
end

[pos, hasPos] = getFieldIgnoreCase(poseStruct, 'position');
if ~hasPos
    [pos, hasPos] = getFieldIgnoreCase(m, 'position');
end
if ~hasPos || ~isstruct(pos)
    return;
end

[xv, hasX] = getNumericFieldIgnoreCase(pos, 'x');
[yv, hasY] = getNumericFieldIgnoreCase(pos, 'y');
if ~(hasX && hasY)
    return;
end

x = double(xv);
y = double(yv);
ok = true;
end

function [vx, vy, ok] = extractLinearVelocityXY(m)
ok = false;
vx = NaN;
vy = NaN;

[twistField, hasTwist] = getFieldIgnoreCase(m, 'twist');
twistStruct = [];
if hasTwist && isstruct(twistField)
    [innerTwist, hasInner] = getFieldIgnoreCase(twistField, 'twist');
    if hasInner && isstruct(innerTwist)
        twistStruct = innerTwist;
    else
        twistStruct = twistField;
    end
end
if isempty(twistStruct)
    return;
end

[lin, hasLin] = getFieldIgnoreCase(twistStruct, 'linear');
if ~hasLin || ~isstruct(lin)
    return;
end

[vxv, hasVx] = getNumericFieldIgnoreCase(lin, 'x');
[vyv, hasVy] = getNumericFieldIgnoreCase(lin, 'y');
if ~(hasVx && hasVy)
    return;
end

vx = double(vxv);
vy = double(vyv);
ok = true;
end

function [value, found] = getFieldIgnoreCase(s, name)
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

function [value, found] = getNumericFieldIgnoreCase(s, name)
[v, found] = getFieldIgnoreCase(s, name);
if found
    value = double(v);
else
    value = NaN;
end
end

function dev = computeDeviationToRaceline(trajX, trajY, racelineXY)
n = numel(trajX);
dev = nan(n, 1);

if isempty(racelineXY) || n == 0
    return;
end

for i = 1:n
    dx = racelineXY(:, 1) - trajX(i);
    dy = racelineXY(:, 2) - trajY(i);
    dev(i) = sqrt(min(dx .* dx + dy .* dy));
end
end
