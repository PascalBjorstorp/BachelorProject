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
defaultRacelineCsv = fullfile(bagsRoot, 'Raceline', 'my_track_raceline_20_03.csv');

% Add one entry per run/algorithm.
runs = [ ...
    struct('algorithm', 'MPC', 'bagName', 'MPC_SPEEDTEST_8_laps', 'racelineCsv', ''), ...
    struct('algorithm', 'Pure Pursuit', 'bagName', 'PP_SPEEDTEST_8_laps', 'racelineCsv', ''), ...
    struct('algorithm', 'Pure Pursuit', 'bagName', 'PP_SPEEDTEST4_8_laps', 'racelineCsv', ''), ...
    ];

%% Validate configuration
if isempty(runs)
    error('No runs configured in USER INPUT section.');
end

%% Load and process all runs
results = struct( ...
    'algorithm', {}, ...
    'bagName', {}, ...
    'bagPath', {}, ...
    'lapTimeS', {}, ...
    'distanceM', {}, ...
    'speedMean', {}, ...
    'speedP95', {}, ...
    'speedMax', {}, ...
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

    dt = diff(tEkf);
    dt(dt <= 0 | ~isfinite(dt)) = 0;
    lapTimeS = tEkf(end) - tEkf(1);
    distanceM = sum(speedAtEkf(1:end-1) .* dt);

    raceCsv = cfg.racelineCsv;
    if isempty(raceCsv)
        raceCsv = defaultRacelineCsv;
    end
    racelineXY = loadRacelineXY(raceCsv);

    mapData = [];
    if ~isempty(mapMsgs)
        mapData = decodeOccupancyGridMsg(mapMsgs{end});
    end

    r = struct();
    r.algorithm = cfg.algorithm;
    r.bagName = cfg.bagName;
    r.bagPath = bagPath;
    r.lapTimeS = lapTimeS;
    r.distanceM = distanceM;
    r.speedMean = mean(speedAtEkf, 'omitnan');
    r.speedP95 = prctile(speedAtEkf, 95);
    r.speedMax = max(speedAtEkf);
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

    plot(results(i).trajX(1), results(i).trajY(1), 'go', 'MarkerFaceColor', 'g', ...
        'DisplayName', 'start');
    plot(results(i).trajX(end), results(i).trajY(end), 'ro', 'MarkerFaceColor', 'r', ...
        'DisplayName', 'end');

    caxis([cMin, cMax]);
    set(gca, 'XTick', [], 'YTick', []);
    addScaleBarBelowAxes(fig, gca, 1.0, '1 m');
    title(sprintf('%s | %s', results(i).algorithm, results(i).bagName), 'Interpreter', 'none');
    hold off;
end

cb = colorbar;
cb.Layout.Tile = 'north';
cb.Label.String = 'Speed [m/s]';

%% Combined summary table for all algorithms
alg = cellstr(string({results.algorithm})');
bag = cellstr(string({results.bagName})');
lap = [results.lapTimeS]';
dist = [results.distanceM]';
vMean = [results.speedMean]';
vP95 = [results.speedP95]';
vMax = [results.speedMax]';

summaryT = table(alg, bag, lap, dist, vMean, vP95, vMax, ...
    'VariableNames', {'Algorithm', 'BagName', 'LapTime_s', 'Distance_m', 'SpeedMean_mps', 'SpeedP95_mps', 'SpeedMax_mps'});

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