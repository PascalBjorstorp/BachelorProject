function analyze_benchmark_bags()
% ANALYZE_BENCHMARK_BAGS  Aggregate analysis of GroundTruth test bags.
%
% Windowing rule per bag:
%   - Start: 1 second before first non-zero drive command
%   - End:   30 seconds after first non-zero drive command
%
% Outputs:
%   - No per-bag figures
%   - Combined mean error curves (position + angle)
%   - Speed-group boxplots (1.0, 1.5, 2.0, all-runs)
%
% Expected bag naming:
%   GroundTruthTest_<speed>_<run>
% where speed uses "dot" notation, e.g. GroundTruthTest_1dot5_2

%% Configuration
scriptDir = fileparts(mfilename('fullpath'));
rootDir = fullfile(scriptDir, '..', '..', '..');
rootDir = char(java.io.File(rootDir).getCanonicalPath());

bagsRoot = fullfile(rootDir, 'bags');
bagPattern = 'GroundTruthTest_*';

% Topics
carPoseTopicPattern = '/vrpn_mocap/car_pos/pose';
egoPoseSubstring = 'ego_pose_world';
driveTopicPattern = '/drive';

% Window relative to first drive command
tPre = 1.0;
tPost = 30.0;

% Common time vector for aggregate mean/std curves
tRelCommon = (0:0.1:tPost)';

%% Discover bags
bagInfo = discoverBagDirs(bagsRoot, bagPattern);
if isempty(bagInfo)
    error('No bags found under %s matching %s', bagsRoot, bagPattern);
end

fprintf('Found %d bag(s) in %s\n', numel(bagInfo), bagsRoot);

%% Collect per-bag metrics and aligned errors
results = struct( ...
    'bagName', {}, ...
    'bagPath', {}, ...
    'speed', {}, ...
    'runId', {}, ...
    'nSamples', {}, ...
    'meanPosError', {}, ...
    'rmsePosError', {}, ...
    'meanAbsYawError', {}, ...
    'rmseYawError', {}, ...
    'tRel', {}, ...
    'posErr', {}, ...
    'absYawErr', {} ...
);

for i = 1:numel(bagInfo)
    bagPath = bagInfo(i).path;
    bagName = bagInfo(i).name;
    fprintf('\n=== Processing bag %d/%d: %s ===\n', i, numel(bagInfo), bagName);

    try
        bag = ros2bagreader(bagPath);
    catch ME
        warning('Failed to open bag at %s: %s', bagPath, ME.message);
        continue;
    end

    allTopics = getTopicNames(bag.AvailableTopics);

    carTopic = resolveCarTopic(allTopics, carPoseTopicPattern);
    if isempty(carTopic)
        warning('No car pose topic found in %s', bagName);
        continue;
    end

    egoTopic = resolveEgoTopic(allTopics, egoPoseSubstring);
    if isempty(egoTopic)
        warning('No ego topic containing "%s" found in %s', egoPoseSubstring, bagName);
        continue;
    end

    driveTopic = resolveDriveTopic(allTopics, driveTopicPattern);
    if isempty(driveTopic)
        warning('No drive topic found in %s', bagName);
        continue;
    end

    fprintf('Using car topic:   %s\n', carTopic);
    fprintf('Using ego topic:   %s\n', egoTopic);
    fprintf('Using drive topic: %s\n', driveTopic);

    carMsgs = readMessages(select(bag, 'Topic', carTopic));
    egoMsgs = readMessages(select(bag, 'Topic', egoTopic));
    drvMsgs = readMessages(select(bag, 'Topic', driveTopic));

    if isempty(carMsgs) || isempty(egoMsgs) || isempty(drvMsgs)
        warning('One or more required topics are empty in %s', bagName);
        continue;
    end

    [tCar, posCar, yawCar] = extractPoseSeries(carMsgs);
    [tEgo, posEgo, yawEgo] = extractPoseSeries(egoMsgs);
    [tDrv, speedDrv] = extractDriveSeries(drvMsgs);

    if isempty(tCar) || isempty(tEgo) || isempty(tDrv)
        warning('Failed to extract data from %s', bagName);
        continue;
    end

    idxMove = find(abs(speedDrv) > 0.05, 1, 'first');
    if isempty(idxMove)
        warning('No non-zero drive command found in %s', bagName);
        continue;
    end

    tDrive0 = tDrv(idxMove);
    tStart = tDrive0 - tPre;
    tEnd = tDrive0 + tPost;

    [tCarW, posCarW, yawCarW] = trimPoseWindow(tCar, posCar, yawCar, tStart, tEnd);
    [tEgoW, posEgoW, yawEgoW] = trimPoseWindow(tEgo, posEgo, yawEgo, tStart, tEnd);

    if numel(tCarW) < 2 || numel(tEgoW) < 2
        warning('Insufficient windowed pose data in %s', bagName);
        continue;
    end

    [tCommon, posCarI, yawCarI, posEgoI, yawEgoI] = alignPoseSeriesWithYaw( ...
        tCarW, posCarW, yawCarW, tEgoW, posEgoW, yawEgoW);

    if isempty(tCommon)
        warning('No common aligned samples in %s', bagName);
        continue;
    end

    tRel = tCommon - tDrive0;
    keep = tRel >= 0 & tRel <= tPost;
    tRel = tRel(keep);
    posCarI = posCarI(keep, :);
    posEgoI = posEgoI(keep, :);
    yawCarI = yawCarI(keep);
    yawEgoI = yawEgoI(keep);

    if isempty(tRel)
        warning('No samples in [0, %.1f]s window after alignment for %s', tPost, bagName);
        continue;
    end

    posErr = vecnorm(posCarI - posEgoI, 2, 2);
    yawErr = wrapToPi(yawCarI - yawEgoI);
    absYawErr = abs(yawErr);

    [speedVal, runId] = parseSpeedRunFromName(bagName);

    r = struct();
    r.bagName = bagName;
    r.bagPath = bagPath;
    r.speed = speedVal;
    r.runId = runId;
    r.nSamples = numel(tRel);
    r.meanPosError = mean(posErr);
    r.rmsePosError = sqrt(mean(posErr .^ 2));
    r.meanAbsYawError = mean(absYawErr);
    r.rmseYawError = sqrt(mean(yawErr .^ 2));
    r.tRel = tRel;
    r.posErr = posErr;
    r.absYawErr = absYawErr;

    results(end + 1) = r; %#ok<AGROW>

    fprintf('  mean pos err: %.4f m | rmse pos err: %.4f m\n', r.meanPosError, r.rmsePosError);
    fprintf('  mean |yaw|:   %.4f rad | rmse yaw:     %.4f rad\n', r.meanAbsYawError, r.rmseYawError);
end

if isempty(results)
    error('No valid bag results were generated.');
end

%% Aggregate mean/std curves across all runs
[posMean, posStd, yawMean, yawStd] = aggregateMeanCurves(results, tRelCommon);

fig1 = figure('Name', 'Combined Mean Error Curves', 'NumberTitle', 'off');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plotWithBand(tRelCommon, posMean, posStd, [0.1 0.4 0.9]);
grid on;
xlabel('time since first drive command [s]');
ylabel('position error [m]');
title('Combined Position Error Mean \pm Std Across Runs');

nexttile;
plotWithBand(tRelCommon, yawMean, yawStd, [0.9 0.3 0.2]);
grid on;
xlabel('time since first drive command [s]');
ylabel('|yaw error| [rad]');
title('Combined Angle Error Mean \pm Std Across Runs');

%% Speed comparison boxplots (3 speeds + all)
[speedLabels, posGroups] = buildSpeedGroups(results, 'meanPosError');
[~, yawGroups] = buildSpeedGroups(results, 'meanAbsYawError');

fig2 = figure('Name', 'Speed Group Comparison', 'NumberTitle', 'off');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
boxplot(posGroups.values, posGroups.groupIds, 'Labels', speedLabels);
grid on;
ylabel('mean position error per bag [m]');
title('Position Error Comparison: 1.0, 1.5, 2.0 m/s, and All Runs');

nexttile;
boxplot(yawGroups.values, yawGroups.groupIds, 'Labels', speedLabels);
grid on;
ylabel('mean |yaw error| per bag [rad]');
title('Angle Error Comparison: 1.0, 1.5, 2.0 m/s, and All Runs');

%% Optional distribution overview (all sample points)
allPosSamples = vertcat(results.posErr);
allYawSamples = vertcat(results.absYawErr);

fig3 = figure('Name', 'All-Sample Error Distributions', 'NumberTitle', 'off');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
histogram(allPosSamples, 80);
grid on;
xlabel('position error [m]');
ylabel('count');
title('All Bags Position Error Distribution');

nexttile;
histogram(allYawSamples, 80);
grid on;
xlabel('|yaw error| [rad]');
ylabel('count');
title('All Bags Angle Error Distribution');

%% Console summary
fprintf('\n=== Combined Summary (%d valid bags) ===\n', numel(results));
meanPosAll = mean([results.meanPosError]);
meanYawAll = mean([results.meanAbsYawError]);
fprintf('Mean of per-bag mean position error: %.4f m\n', meanPosAll);
fprintf('Mean of per-bag mean |yaw| error:   %.4f rad\n', meanYawAll);

for s = [1.0, 1.5, 2.0]
    idx = abs([results.speed] - s) < 1e-6;
    if any(idx)
        fprintf('Speed %.1f m/s: n=%d, mean pos=%.4f m, mean |yaw|=%.4f rad\n', ...
            s, nnz(idx), mean([results(idx).meanPosError]), mean([results(idx).meanAbsYawError]));
    end
end

% Keep figures open for inspection.
if ishghandle(fig1), figure(fig1); end
if ishghandle(fig2), figure(fig2); end
if ishghandle(fig3), figure(fig3); end

end

function info = discoverBagDirs(bagsRoot, bagPattern)
if ~isfolder(bagsRoot)
    info = struct('name', {}, 'path', {});
    return;
end

d = dir(fullfile(bagsRoot, bagPattern));
d = d([d.isdir]);
d = d(~ismember({d.name}, {'.', '..'}));

info = struct('name', {}, 'path', {});
for i = 1:numel(d)
    bagPath = fullfile(d(i).folder, d(i).name);
    if isfile(fullfile(bagPath, 'metadata.yaml'))
        info(end + 1).name = d(i).name; %#ok<AGROW>
        info(end).path = bagPath;
    end
end

% Stable order
if ~isempty(info)
    [~, idx] = sort({info.name});
    info = info(idx);
end
end

function allTopics = getTopicNames(topicsTbl)
vars = topicsTbl.Properties.VariableNames;
if ismember("TopicName", vars)
    allTopics = string(topicsTbl.TopicName);
elseif ismember("Name", vars)
    allTopics = string(topicsTbl.Name);
elseif ismember("Topic", vars)
    allTopics = string(topicsTbl.Topic);
else
    rowNames = topicsTbl.Properties.RowNames;
    if ~isempty(rowNames)
        allTopics = string(rowNames);
    else
        error('Unknown AvailableTopics table format.');
    end
end
end

function carTopic = resolveCarTopic(allTopics, desired)
carTopic = '';
if any(strcmp(allTopics, desired))
    carTopic = char(desired);
    return;
end

idx = contains(allTopics, 'car_pos') & contains(allTopics, 'pose');
if any(idx)
    carTopic = char(allTopics(find(idx, 1, 'first')));
end
end

function egoTopic = resolveEgoTopic(allTopics, egoPoseSubstring)
egoTopic = '';
idx = contains(allTopics, egoPoseSubstring);
if any(idx)
    egoTopic = char(allTopics(find(idx, 1, 'first')));
end
end

function driveTopic = resolveDriveTopic(allTopics, desired)
driveTopic = '';
if any(strcmp(allTopics, desired))
    driveTopic = char(desired);
    return;
end

idx = contains(allTopics, 'drive');
if any(idx)
    driveTopic = char(allTopics(find(idx, 1, 'first')));
end
end

function [t, pos, yaw] = extractPoseSeries(msgs)
num = numel(msgs);
t = zeros(num, 1);
pos = zeros(num, 3);
yaw = zeros(num, 1);

for k = 1:num
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end

    t(k) = k;
    [header, hasHeader] = getFieldIgnoreCase(m, 'header');
    if hasHeader && isstruct(header)
        [stamp, hasStamp] = getFieldIgnoreCase(header, 'stamp');
        if hasStamp && isstruct(stamp)
            [secVal, hasSec] = getNumericFieldIgnoreCase(stamp, 'sec');
            [nsecVal, hasNSec] = getNumericFieldIgnoreCase(stamp, 'nanosec');
            if ~hasNSec
                [nsecVal, hasNSec] = getNumericFieldIgnoreCase(stamp, 'nsec');
            end
            if hasSec && hasNSec
                t(k) = double(secVal) + double(nsecVal) * 1e-9;
            elseif hasSec
                t(k) = double(secVal);
            end
        end
    end

    poseStruct = [];
    [poseField, hasPose] = getFieldIgnoreCase(m, 'pose');
    if hasPose && isstruct(poseField)
        [innerPose, hasInnerPose] = getFieldIgnoreCase(poseField, 'pose');
        if hasInnerPose && isstruct(innerPose)
            poseStruct = innerPose;
        else
            poseStruct = poseField;
        end
    end

    [position, hasPosition] = getFieldIgnoreCase(poseStruct, 'position');
    if ~hasPosition
        [position, hasPosition] = getFieldIgnoreCase(m, 'position');
    end

    if hasPosition && isstruct(position)
        [px, hasX] = getNumericFieldIgnoreCase(position, 'x');
        [py, hasY] = getNumericFieldIgnoreCase(position, 'y');
        [pz, hasZ] = getNumericFieldIgnoreCase(position, 'z');
        if hasX, pos(k,1) = double(px); else, pos(k,1) = NaN; end
        if hasY, pos(k,2) = double(py); else, pos(k,2) = NaN; end
        if hasZ, pos(k,3) = double(pz); else, pos(k,3) = NaN; end
    else
        pos(k,:) = [NaN, NaN, NaN];
    end

    [orientation, hasOri] = getFieldIgnoreCase(poseStruct, 'orientation');
    if hasOri && isstruct(orientation)
        [qx, hasQx] = getNumericFieldIgnoreCase(orientation, 'x');
        [qy, hasQy] = getNumericFieldIgnoreCase(orientation, 'y');
        [qz, hasQz] = getNumericFieldIgnoreCase(orientation, 'z');
        [qw, hasQw] = getNumericFieldIgnoreCase(orientation, 'w');
        if hasQx && hasQy && hasQz && hasQw
            yaw(k) = quatToYaw(double(qx), double(qy), double(qz), double(qw));
        else
            yaw(k) = NaN;
        end
    else
        yaw(k) = NaN;
    end
end

valid = all(isfinite(pos), 2) & isfinite(yaw);
t = t(valid);
pos = pos(valid, :);
yaw = yaw(valid);

[t, idx] = sort(t);
pos = pos(idx, :);
yaw = yaw(idx);

[t, idxUnique] = unique(t);
pos = pos(idxUnique, :);
yaw = yaw(idxUnique);
end

function [t, speed] = extractDriveSeries(msgs)
num = numel(msgs);
t = zeros(num, 1);
speed = zeros(num, 1);

for k = 1:num
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end

    t(k) = k;
    [header, hasHeader] = getFieldIgnoreCase(m, 'header');
    if hasHeader && isstruct(header)
        [stamp, hasStamp] = getFieldIgnoreCase(header, 'stamp');
        if hasStamp && isstruct(stamp)
            [secVal, hasSec] = getNumericFieldIgnoreCase(stamp, 'sec');
            [nsecVal, hasNSec] = getNumericFieldIgnoreCase(stamp, 'nanosec');
            if ~hasNSec
                [nsecVal, hasNSec] = getNumericFieldIgnoreCase(stamp, 'nsec');
            end
            if hasSec && hasNSec
                t(k) = double(secVal) + double(nsecVal) * 1e-9;
            elseif hasSec
                t(k) = double(secVal);
            end
        end
    end

    v = NaN;
    [drive, hasDrive] = getFieldIgnoreCase(m, 'drive');
    if hasDrive && isstruct(drive)
        [vTmp, hasV] = getNumericFieldIgnoreCase(drive, 'speed');
        if hasV
            v = double(vTmp);
        end
    end

    speed(k) = v;
end

valid = isfinite(speed);
t = t(valid);
speed = speed(valid);

[t, idx] = sort(t);
speed = speed(idx);

[t, idxUnique] = unique(t);
speed = speed(idxUnique);
end

function [tW, posW, yawW] = trimPoseWindow(t, pos, yaw, tStart, tEnd)
mask = (t >= tStart) & (t <= tEnd);
tW = t(mask);
posW = pos(mask, :);
yawW = yaw(mask);
end

function [tCommon, posAInterp, yawAInterp, posBInterp, yawBInterp] = alignPoseSeriesWithYaw(tA, posA, yawA, tB, posB, yawB)
tStart = max(min(tA), min(tB));
tEnd = min(max(tA), max(tB));

if tEnd <= tStart
    tCommon = [];
    posAInterp = [];
    yawAInterp = [];
    posBInterp = [];
    yawBInterp = [];
    return;
end

maskA = tA >= tStart & tA <= tEnd;
tCommon = tA(maskA);
posAInterp = posA(maskA, :);
yawAInterp = yawA(maskA);

[tCommon, sortIdx] = sort(tCommon);
posAInterp = posAInterp(sortIdx, :);
yawAInterp = yawAInterp(sortIdx);

[tBUnique, ia] = unique(tB(:));
posBUnique = posB(ia, :);
yawBUnique = yawB(ia);

posBInterp = zeros(numel(tCommon), 3);
for dim = 1:3
    posBInterp(:, dim) = interp1(tBUnique, posBUnique(:, dim), tCommon, 'linear', 'extrap');
end

yawBInterp = interpYaw(tBUnique, yawBUnique, tCommon);
end

function yawInterp = interpYaw(t, yaw, tq)
yawUnwrapped = unwrap(yaw);
yawInterp = interp1(t, yawUnwrapped, tq, 'linear', 'extrap');
yawInterp = wrapToPi(yawInterp);
end

function [posMean, posStd, yawMean, yawStd] = aggregateMeanCurves(results, tRelCommon)
numRuns = numel(results);
numT = numel(tRelCommon);
posMat = nan(numT, numRuns);
yawMat = nan(numT, numRuns);

for i = 1:numRuns
    t = results(i).tRel;
    pe = results(i).posErr;
    ye = results(i).absYawErr;

    [tUnique, idx] = unique(t(:));
    pe = pe(idx);
    ye = ye(idx);

    if numel(tUnique) < 2
        continue;
    end

    posMat(:, i) = interp1(tUnique, pe, tRelCommon, 'linear', NaN);
    yawMat(:, i) = interp1(tUnique, ye, tRelCommon, 'linear', NaN);
end

posMean = mean(posMat, 2, 'omitnan');
posStd = std(posMat, 0, 2, 'omitnan');
yawMean = mean(yawMat, 2, 'omitnan');
yawStd = std(yawMat, 0, 2, 'omitnan');
end

function plotWithBand(t, m, s, colorRGB)
upper = m + s;
lower = m - s;

fill([t; flipud(t)], [upper; flipud(lower)], colorRGB, ...
    'FaceAlpha', 0.2, 'EdgeColor', 'none');
hold on;
plot(t, m, 'Color', colorRGB, 'LineWidth', 1.8);
end

function [labels, out] = buildSpeedGroups(results, metricField)
labels = {'1.0 m/s', '1.5 m/s', '2.0 m/s', 'All'};

g1 = [results(abs([results.speed] - 1.0) < 1e-6).(metricField)];
g2 = [results(abs([results.speed] - 1.5) < 1e-6).(metricField)];
g3 = [results(abs([results.speed] - 2.0) < 1e-6).(metricField)];
gAll = [results.(metricField)];

vals = [g1(:); g2(:); g3(:); gAll(:)];
groupIds = [ ...
    ones(numel(g1), 1); ...
    2 * ones(numel(g2), 1); ...
    3 * ones(numel(g3), 1); ...
    4 * ones(numel(gAll), 1) ...
];

out.values = vals;
out.groupIds = groupIds;
end

function [speedVal, runId] = parseSpeedRunFromName(name)
speedVal = NaN;
runId = NaN;

% GroundTruthTest_1dot5_2
pat = '^GroundTruthTest_(\d+)dot(\d+)_(\d+)$';
t = regexp(name, pat, 'tokens', 'once');
if ~isempty(t)
    speedVal = str2double([t{1} '.' t{2}]);
    runId = str2double(t{3});
    return;
end

% Fallback: try GroundTruthTest_1_2
pat2 = '^GroundTruthTest_(\d+)_(\d+)$';
t2 = regexp(name, pat2, 'tokens', 'once');
if ~isempty(t2)
    speedVal = str2double(t2{1});
    runId = str2double(t2{2});
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

function yaw = quatToYaw(x, y, z, w)
siny_cosp = 2.0 * (w * z + x * y);
cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
yaw = atan2(siny_cosp, cosy_cosp);
end
