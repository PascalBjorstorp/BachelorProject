function plot_algorithm_speed_maps()
clc;
clear all;
close all;
% PLOT_ALGORITHM_SPEED_MAPS
% Plot one map per algorithm/run with:
% - Occupancy map from /map
% - Reference raceline from CSV
% - EKF trajectory colored by speed from /ego_racecar/odom
%
% Configure runs in the "USER INPUT" section below.

%% USER INPUT
scriptDir = fileparts(mfilename('fullpath'));
repoRoot = char(java.io.File(fullfile(scriptDir, '..', '..', '..')).getCanonicalPath());

% Path where bag directories are stored.
bagsRoot = fullfile(repoRoot, 'bags');

% Default raceline CSV (used if a run does not set racelineCsv).
defaultRacelineCsv = fullfile(bagsRoot, 'Raceline', 'my_track_raceline_31_03.csv');

% Keep data from [drive detected + startDelayAfterDriveSeconds] and
% until [last active drive command - endTrimBeforeStopSeconds].
startDelayAfterDriveSeconds = 3.0;
endTrimBeforeStopSeconds = 1.0;

% Drive command threshold used to detect active driving.
driveCommandThreshold = 1e-3;

% Lap detection settings (for full-lap-only metrics).
lapGateHalfLengthM = 2.0;
minLapTimeSeconds = 5.0;

% Display trajectory start/end markers on plots.
showStartEndMarkers = false;

% Add one entry per run/algorithm.

runs = [ ...
    struct('algorithm', 'MPC', 'bagName', 'MPC_SPEEDTEST_8_laps', 'racelineCsv', ''), ...
    struct('algorithm', 'Pure Pursuit', 'bagName', 'PP_SPEEDTEST3_8_laps', 'racelineCsv', ''), ...
    struct('algorithm', 'Pure Pursuit', 'bagName', 'PP_SPEEDTEST4_8_laps', 'racelineCsv', ''), ...
    ];

runs = [ ...
    struct('algorithm', 'MPC', 'bagName', 'MPCTestNoSweepNewMap', 'racelineCsv', '')];

%% Validate configuration
if isempty(runs)
    error('No runs configured in USER INPUT section.');
end

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
    cmdTopic = pickDriveCommandTopic(allTopics);

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
    cmdSel = [];
    cmdMsgs = {};
    if ~isempty(mapTopic)
        mapMsgs = readMessages(select(bag, 'Topic', mapTopic));
    end
    if ~isempty(cmdTopic)
        cmdSel = select(bag, 'Topic', cmdTopic);
        cmdMsgs = readMessages(cmdSel);
    end

    [tEkf, xyEkf] = extractXYSeries(ekfMsgs);
    [tOdom, speedOdom] = extractOdomSpeedSeries(odomMsgs);
    [tCmd, speedCmd] = extractDriveCommandSpeedSeries(cmdMsgs, cmdSel);

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

    tCmdForWindow = tCmd;
    tEkfForWindow = tEkf;
    if ~isempty(tCmdForWindow) && ~isempty(tEkfForWindow)
        if abs(tEkfForWindow(1) - tCmdForWindow(1)) > 1e3
            tCmdForWindow = tCmdForWindow - tCmdForWindow(1);
            tEkfForWindow = tEkfForWindow - tEkfForWindow(1);
        end
    end

    [tWindowStart, tWindowEnd, hasDriveWindow] = computeDriveWindow( ...
        tCmdForWindow, speedCmd, startDelayAfterDriveSeconds, endTrimBeforeStopSeconds, driveCommandThreshold);

    if hasDriveWindow
        keep = tEkfForWindow >= tWindowStart & tEkfForWindow <= tWindowEnd;
        if nnz(keep) < 2
            % Fallback: use full command stream bounds if active-window bounds are too tight.
            if numel(tCmdForWindow) >= 2
                tFallbackStart = tCmdForWindow(1) + max(0, startDelayAfterDriveSeconds);
                tFallbackEnd = tCmdForWindow(end) - max(0, endTrimBeforeStopSeconds);
                keepFallback = tEkfForWindow >= tFallbackStart & tEkfForWindow <= tFallbackEnd;
                if nnz(keepFallback) >= 2
                    keep = keepFallback;
                else
                    warning('Drive window leaves too few samples in %s; using untrimmed aligned data.', cfg.bagName);
                    keep = true(size(tEkf));
                end
            else
                warning('Drive window leaves too few samples in %s; using untrimmed aligned data.', cfg.bagName);
                keep = true(size(tEkf));
            end
        end
        tEkf = tEkf(keep);
        xyEkf = xyEkf(keep, :);
        speedAtEkf = speedAtEkf(keep);
    else
        warning('No active drive window found in %s; using untrimmed aligned data.', cfg.bagName);
    end

    raceCsv = cfg.racelineCsv;
    if isempty(raceCsv)
        raceCsv = defaultRacelineCsv;
    end
    racelineXY = loadRacelineXY(raceCsv);

    [lapDurS, lapCrossingsS] = detectFullLapDurations( ...
        tEkf, xyEkf(:, 1), xyEkf(:, 2), racelineXY, lapGateHalfLengthM, minLapTimeSeconds);

    if isempty(lapDurS)
        warning('No full laps detected in %s after trimming.', cfg.bagName);
        continue;
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

fig = figure('Name', 'Algorithm Trajectory Speed Maps', 'Color', 'w');
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

figTable = figure('Name', 'Algorithm Summary Table', 'Color', 'w');
uitable(figTable, 'Data', table2cell(summaryT), ...
    'ColumnName', summaryT.Properties.VariableNames, ...
    'Units', 'normalized', 'Position', [0 0 1 1]);

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

function topic = pickDriveCommandTopic(allTopics)
% Prefer built-in message types to avoid custom-message dependencies.
topic = pickTopic(allTopics, '/commands/motor/speed', 'commands/motor/speed');
if ~isempty(topic)
    return;
end

topic = pickTopic(allTopics, '/ackermann_cmd', 'ackermann_cmd');
if ~isempty(topic)
    return;
end

topic = pickTopic(allTopics, '/drive', 'drive');
if ~isempty(topic)
    return;
end

topic = pickTopic(allTopics, '/teleop', 'teleop');
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

function [t, speedCmd] = extractDriveCommandSpeedSeries(msgs, sel)
n = numel(msgs);
t = zeros(n, 1);
speedCmd = nan(n, 1);

msgList = [];
if nargin >= 2 && ~isempty(sel)
    msgList = sel.MessageList;
end

for k = 1:n
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end

    tHeader = extractHeaderTime(m, NaN);
    if isfinite(tHeader)
        t(k) = tHeader;
    else
        t(k) = extractSelectionTime(msgList, k);
    end

    if ~isfinite(t(k))
        t(k) = k;
    end

    [vCmd, ok] = extractDriveCommandSpeed(m);
    if ok
        speedCmd(k) = vCmd;
    end
end

valid = isfinite(t) & isfinite(speedCmd);
t = t(valid);
speedCmd = speedCmd(valid);

[t, idx] = sort(t);
speedCmd = speedCmd(idx);
[t, iUnique] = unique(t);
speedCmd = speedCmd(iUnique);
end

function [tStart, tEnd, ok] = computeDriveWindow(tCmd, speedCmd, startDelayS, endTrimS, speedThresh)
tStart = NaN;
tEnd = NaN;
ok = false;

if isempty(tCmd) || isempty(speedCmd)
    return;
end

active = isfinite(speedCmd) & abs(speedCmd) > speedThresh;
if any(active)
    tFirstActive = tCmd(find(active, 1, 'first'));
    tLastActive = tCmd(find(active, 1, 'last'));
else
    % Fallback when all commands are near zero: use command-stream bounds.
    tFirstActive = tCmd(1);
    tLastActive = tCmd(end);
end

tStart = tFirstActive + max(0, startDelayS);
tEnd = tLastActive - max(0, endTrimS);

if ~isfinite(tStart) || ~isfinite(tEnd) || tEnd <= tStart
    ok = false;
    return;
end

ok = true;
end

function [lapDurS, crossingTimes] = detectFullLapDurations(t, x, y, racelineXY, gateHalfLengthM, minLapTimeS)
lapDurS = [];
crossingTimes = [];

if numel(t) < 3 || isempty(racelineXY) || size(racelineXY, 1) < 2
    return;
end

p0 = racelineXY(1, :);
p1 = racelineXY(2, :);
dirVec = p1 - p0;
dirNorm = hypot(dirVec(1), dirVec(2));
if dirNorm <= 1e-9
    return;
end

tangent = dirVec / dirNorm;
normal = [-tangent(2), tangent(1)];

dx = x - p0(1);
dy = y - p0(2);
along = dx * tangent(1) + dy * tangent(2);
lat = dx * normal(1) + dy * normal(2);

crossT = [];
crossDir = [];
for i = 1:(numel(t) - 1)
    if ~isfinite(lat(i)) || ~isfinite(lat(i + 1)) || ~isfinite(t(i)) || ~isfinite(t(i + 1))
        continue;
    end

    alongMid = 0.5 * (along(i) + along(i + 1));
    if abs(alongMid) > gateHalfLengthM
        continue;
    end

    crossing = false;
    alpha = 0.0;
    if lat(i) == 0
        crossing = true;
        alpha = 0.0;
    elseif lat(i + 1) == 0
        crossing = true;
        alpha = 1.0;
    elseif (lat(i) < 0 && lat(i + 1) > 0) || (lat(i) > 0 && lat(i + 1) < 0)
        crossing = true;
        alpha = lat(i) / (lat(i) - lat(i + 1));
    end

    if ~crossing
        continue;
    end

    tCross = t(i) + alpha * (t(i + 1) - t(i));
    if ~isfinite(tCross)
        continue;
    end

    crossT(end + 1, 1) = tCross; %#ok<AGROW>
    crossDir(end + 1, 1) = sign(lat(i + 1) - lat(i)); %#ok<AGROW>
end

if numel(crossT) < 2
    return;
end

nonZeroDir = crossDir(crossDir ~= 0);
if isempty(nonZeroDir)
    return;
end

dominantDir = mode(nonZeroDir);
crossT = crossT(crossDir == dominantDir);
if numel(crossT) < 2
    return;
end

crossingTimes = crossT(1);
for i = 2:numel(crossT)
    if crossT(i) - crossingTimes(end) >= minLapTimeS
        crossingTimes(end + 1, 1) = crossT(i); %#ok<AGROW>
    end
end

if numel(crossingTimes) < 2
    crossingTimes = [];
    return;
end

lapDurS = diff(crossingTimes);
valid = isfinite(lapDurS) & lapDurS > 0;
lapDurS = lapDurS(valid);
if isempty(lapDurS)
    crossingTimes = [];
end
end

function t = extractSelectionTime(msgList, idx)
t = NaN;
if isempty(msgList) || idx < 1 || idx > height(msgList)
    return;
end

vars = string(msgList.Properties.VariableNames);
idxTs = find(strcmpi(vars, 'Timestamp'), 1, 'first');
idxTime = find(strcmpi(vars, 'Time'), 1, 'first');

if ~isempty(idxTs)
    v = msgList{idx, idxTs};
elseif ~isempty(idxTime)
    v = msgList{idx, idxTime};
else
    return;
end

if isa(v, 'duration')
    t = seconds(v);
    return;
end

if isa(v, 'datetime')
    t = posixtime(v);
    return;
end

if iscell(v)
    v = v{1};
end

v = double(v);
if ~isfinite(v)
    return;
end

if v > 1e12
    t = v * 1e-9;
else
    t = v;
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

function [vCmd, ok] = extractDriveCommandSpeed(m)
ok = false;
vCmd = NaN;

% std_msgs/msg/Float64 style command topic.
[dataVal, hasData] = getNumericFieldIgnoreCase(m, 'data');
if hasData && isfinite(dataVal)
    vCmd = double(dataVal);
    ok = true;
    return;
end

% ackermann_msgs/msg/AckermannDriveStamped style command topic.
[drive, hasDrive] = getFieldIgnoreCase(m, 'drive');
if hasDrive && isstruct(drive)
    [speedVal, hasSpeed] = getNumericFieldIgnoreCase(drive, 'speed');
    if hasSpeed && isfinite(speedVal)
        vCmd = double(speedVal);
        ok = true;
        return;
    end
end

% Fallback for flat speed field naming.
[speedVal, hasSpeed] = getNumericFieldIgnoreCase(m, 'speed');
if hasSpeed && isfinite(speedVal)
    vCmd = double(speedVal);
    ok = true;
end
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