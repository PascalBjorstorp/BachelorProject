function results = loadOptiTrackBenchmarkBags(bagRootDir, config)
%LOADOPTITRACKBENCHMARKBAGS Load bags and compute EKF minus OptiTrack errors.

bagDirs = discoverOptiTrackBagDirs(bagRootDir);
if isempty(bagDirs)
    error('No rosbag metadata.yaml files found under %s', bagRootDir);
end

fprintf('Found %d bag(s)\n', numel(bagDirs));

results = struct( ...
    'bagName', {}, ...
    'bagPath', {}, ...
    'durationS', {}, ...
    'nSamples', {}, ...
    'trimApplied', {}, ...
    'lapBoundariesS', {}, ...
    't', {}, ...
    'tRel', {}, ...
    'gtPos', {}, ...
    'gtYaw', {}, ...
    'ekfPos', {}, ...
    'ekfYaw', {}, ...
    'xError', {}, ...
    'yError', {}, ...
    'xyError', {}, ...
    'yawError', {}, ...
    'meanXError', {}, ...
    'stdXError', {}, ...
    'meanYError', {}, ...
    'stdYError', {}, ...
    'meanXYError', {}, ...
    'stdXYError', {}, ...
    'rmseXYError', {}, ...
    'meanAbsYawError', {}, ...
    'stdYawError', {}, ...
    'rmseYawError', {}, ...
    'maxXYError', {}, ...
    'maxAbsYawError', {} ...
);

for i = 1:numel(bagDirs)
    bagPath = bagDirs{i};
    [~, bagName] = fileparts(bagPath);
    fprintf('\n=== Processing %d/%d: %s ===\n', i, numel(bagDirs), bagName);

    try
        bag = ros2bagreader(bagPath);
    catch ME
        warning('Could not open bag %s: %s', bagPath, ME.message);
        continue;
    end

    topics = getTopicNames(bag.AvailableTopics);
    if ~any(strcmp(topics, config.ekfTopic))
        warning('Skipping %s: missing %s', bagName, config.ekfTopic);
        continue;
    end
    if ~any(strcmp(topics, config.optitrackTopic))
        warning('Skipping %s: missing %s', bagName, config.optitrackTopic);
        continue;
    end

    try
        tf = readStaticMapToWorldTransform(bag, config);
    catch ME
        warning('Skipping %s: failed to read map->world TF: %s', bagName, ME.message);
        continue;
    end

    try
        ekfSel = select(bag, 'Topic', config.ekfTopic);
        gtSel = select(bag, 'Topic', config.optitrackTopic);
        ekfMsgs = readMessages(ekfSel);
        gtMsgs = readMessages(gtSel);
    catch ME
        warning('Skipping %s: failed to read pose topics: %s', bagName, ME.message);
        continue;
    end

    tEkfBag = getSelectionTimes(ekfSel, numel(ekfMsgs));
    tGtBag = getSelectionTimes(gtSel, numel(gtMsgs));

    [tEkf, ekfPos, ekfYaw] = extractPoseSeries(ekfMsgs, tEkfBag);
    [tGtRaw, gtPosRaw, gtQuatRaw] = extractPoseSeriesWithQuaternion(gtMsgs, tGtBag);
    if numel(tEkf) < 2 || numel(tGtRaw) < 2
        warning('Skipping %s: insufficient pose samples', bagName);
        continue;
    end

    [gtPos, gtQuat] = transformPoseSeries(tf, gtPosRaw, gtQuatRaw);
    gtYaw = quatArrayToYaw(gtQuat);

    [tCommon, gtPosI, gtYawI, ekfPosI, ekfYawI] = alignOnGroundTruthTime( ...
        tGtRaw, gtPos, gtYaw, tEkf, ekfPos, ekfYaw);
    if isempty(tCommon)
        warning('Skipping %s: no overlapping EKF/OptiTrack time window', bagName);
        continue;
    end

    keepMask = true(size(tCommon));
    lapBoundariesS = [];
    trimApplied = false;
    if isfield(config, 'skipStartupAndIncompleteLaps') && config.skipStartupAndIncompleteLaps
        [keepMask, lapBoundariesS, trimApplied] = buildLapTrimMask(tCommon, gtPosI, config);
    end

    tCommon = tCommon(keepMask);
    gtPosI = gtPosI(keepMask, :);
    gtYawI = gtYawI(keepMask);
    ekfPosI = ekfPosI(keepMask, :);
    ekfYawI = ekfYawI(keepMask);

    if numel(tCommon) < 2
        warning('Skipping %s: insufficient samples after trimming', bagName);
        continue;
    end

    tRel = tCommon - tCommon(1);
    xErr = ekfPosI(:, 1) - gtPosI(:, 1);
    yErr = ekfPosI(:, 2) - gtPosI(:, 2);
    xyErr = hypot(xErr, yErr);
    yawErr = wrapAnglePi(ekfYawI - gtYawI);

    r = struct();
    r.bagName = bagName;
    r.bagPath = bagPath;
    r.durationS = tRel(end) - tRel(1);
    r.nSamples = numel(tRel);
    r.trimApplied = trimApplied;
    r.lapBoundariesS = lapBoundariesS;
    r.t = tCommon;
    r.tRel = tRel;
    r.gtPos = gtPosI;
    r.gtYaw = gtYawI;
    r.ekfPos = ekfPosI;
    r.ekfYaw = ekfYawI;
    r.xError = xErr;
    r.yError = yErr;
    r.xyError = xyErr;
    r.yawError = yawErr;
    r.meanXError = mean(xErr, 'omitnan');
    r.stdXError = std(xErr, 0, 'omitnan');
    r.meanYError = mean(yErr, 'omitnan');
    r.stdYError = std(yErr, 0, 'omitnan');
    r.meanXYError = mean(xyErr, 'omitnan');
    r.stdXYError = std(xyErr, 0, 'omitnan');
    r.rmseXYError = sqrt(mean(xyErr .^ 2, 'omitnan'));
    r.meanAbsYawError = mean(abs(yawErr), 'omitnan');
    r.stdYawError = std(yawErr, 0, 'omitnan');
    r.rmseYawError = sqrt(mean(yawErr .^ 2, 'omitnan'));
    r.maxXYError = max(xyErr);
    r.maxAbsYawError = max(abs(yawErr));

    results(end + 1) = r; %#ok<AGROW>

    fprintf('Samples: %d | duration: %.2f s | trim: %s\n', ...
        r.nSamples, r.durationS, mat2str(r.trimApplied));
    fprintf('Mean x/y error: %.4f / %.4f m | RMSE XY: %.4f m | mean |yaw|: %.4f rad\n', ...
        r.meanXError, r.meanYError, r.rmseXYError, r.meanAbsYawError);
end
end

function bagDirs = discoverOptiTrackBagDirs(rootDir)
meta = dir(fullfile(rootDir, '**', 'metadata.yaml'));
bagDirs = cell(numel(meta), 1);
for i = 1:numel(meta)
    bagDirs{i} = meta(i).folder;
end
bagDirs = unique(bagDirs, 'stable');
end

function topicNames = getTopicNames(availableTopics)
if istable(availableTopics)
    vars = availableTopics.Properties.VariableNames;
    idx = find(strcmpi(vars, 'TopicName') | strcmpi(vars, 'Topic') | strcmpi(vars, 'Name'), 1, 'first');
    if ~isempty(idx)
        topicNames = string(availableTopics.(vars{idx}));
    else
        rowNames = availableTopics.Properties.RowNames;
        if isempty(rowNames)
            topicNames = strings(0, 1);
        else
            topicNames = string(rowNames);
        end
    end
elseif isstruct(availableTopics)
    [names, found] = getFieldIgnoreCase(availableTopics, 'Name');
    if ~found
        [names, found] = getFieldIgnoreCase(availableTopics, 'Topic');
    end
    if found
        topicNames = string(names);
    else
        topicNames = strings(0, 1);
    end
else
    topicNames = string(availableTopics);
end
topicNames = cellstr(topicNames(:));
end

function tf = readStaticMapToWorldTransform(bag, config)
msgs = readMessages(select(bag, 'Topic', config.staticTfTopic));
if isempty(msgs)
    error('topic %s is empty', config.staticTfTopic);
end

for i = 1:numel(msgs)
    if ~isstruct(msgs{i})
        msgs{i} = struct(msgs{i});
    end
    transforms = getTransformsFromTfMessage(msgs{i});
    for j = 1:numel(transforms)
        tr = transforms(j);
        [header, hasHeader] = getFieldIgnoreCase(tr, 'header');
        [child, hasChild] = getTextFieldIgnoreCase(tr, 'child_frame_id');
        if ~hasChild
            [child, hasChild] = getTextFieldIgnoreCase(tr, 'childframeid');
        end
        if ~hasHeader || ~hasChild
            continue;
        end
        [parent, hasParent] = getTextFieldIgnoreCase(header, 'frame_id');
        if ~hasParent
            [parent, hasParent] = getTextFieldIgnoreCase(header, 'frameid');
        end
        if hasParent && strcmp(stripFrame(parent), config.mapFrame) && ...
                strcmp(stripFrame(child), config.optitrackFrame)
            [transform, hasTransform] = getFieldIgnoreCase(tr, 'transform');
            if ~hasTransform
                continue;
            end
            tf = transformStructToSe3(transform);
            return;
        end
    end
end

error('no %s -> %s transform found in %s', ...
    config.mapFrame, config.optitrackFrame, config.staticTfTopic);
end

function transforms = getTransformsFromTfMessage(msg)
[transforms, found] = getFieldIgnoreCase(msg, 'transforms');
if ~found
    transforms = struct([]);
    return;
end
if iscell(transforms)
    transforms = [transforms{:}];
end
end

function tf = transformStructToSe3(transform)
[translation, hasT] = getFieldIgnoreCase(transform, 'translation');
[rotation, hasR] = getFieldIgnoreCase(transform, 'rotation');
if ~hasT || ~hasR
    error('Transform message missing translation or rotation');
end
[tx, okX] = getNumericFieldIgnoreCase(translation, 'x');
[ty, okY] = getNumericFieldIgnoreCase(translation, 'y');
[tz, okZ] = getNumericFieldIgnoreCase(translation, 'z');
[qx, okQx] = getNumericFieldIgnoreCase(rotation, 'x');
[qy, okQy] = getNumericFieldIgnoreCase(rotation, 'y');
[qz, okQz] = getNumericFieldIgnoreCase(rotation, 'z');
[qw, okQw] = getNumericFieldIgnoreCase(rotation, 'w');
if ~(okX && okY && okZ && okQx && okQy && okQz && okQw)
    error('Transform contains invalid numeric fields');
end
tf.t = [tx, ty, tz];
tf.q = normalizeQuat([qx, qy, qz, qw]);
tf.R = quatToRotmLocal(tf.q);
end

function t = getSelectionTimes(selection, n)
t = nan(n, 1);
try
    raw = selection.MessageList.Time;
    if isduration(raw)
        raw = seconds(raw);
    elseif isdatetime(raw)
        raw = posixtime(raw);
    else
        raw = double(raw);
    end
    raw = raw(:);
    count = min(n, numel(raw));
    t(1:count) = raw(1:count);
catch
end
end

function [t, pos, yaw] = extractPoseSeries(msgs, tOverride)
if nargin < 2
    tOverride = [];
end
[t, pos, quat] = extractPoseSeriesWithQuaternion(msgs, tOverride);
yaw = quatArrayToYaw(quat);
end

function [t, pos, quat] = extractPoseSeriesWithQuaternion(msgs, tOverride)
if nargin < 2
    tOverride = [];
end
n = numel(msgs);
t = nan(n, 1);
pos = nan(n, 3);
quat = nan(n, 4);

for k = 1:n
    m = msgs{k};
    if ~isstruct(m)
        m = struct(m);
    end
    if numel(tOverride) == n && isfinite(tOverride(k))
        t(k) = tOverride(k);
    else
        t(k) = extractHeaderTime(m);
    end
    poseStruct = findPoseStruct(m);
    if isempty(poseStruct)
        continue;
    end

    [position, hasPosition] = getFieldIgnoreCase(poseStruct, 'position');
    [orientation, hasOrientation] = getFieldIgnoreCase(poseStruct, 'orientation');
    if hasPosition
        [px, okX] = getNumericFieldIgnoreCase(position, 'x');
        [py, okY] = getNumericFieldIgnoreCase(position, 'y');
        [pz, okZ] = getNumericFieldIgnoreCase(position, 'z');
        if okX && okY && okZ
            pos(k, :) = [px, py, pz];
        end
    end
    if hasOrientation
        [qx, okX] = getNumericFieldIgnoreCase(orientation, 'x');
        [qy, okY] = getNumericFieldIgnoreCase(orientation, 'y');
        [qz, okZ] = getNumericFieldIgnoreCase(orientation, 'z');
        [qw, okW] = getNumericFieldIgnoreCase(orientation, 'w');
        if okX && okY && okZ && okW
            quat(k, :) = normalizeQuat([qx, qy, qz, qw]);
        end
    end
end

valid = isfinite(t) & all(isfinite(pos), 2) & all(isfinite(quat), 2);
t = t(valid);
pos = pos(valid, :);
quat = quat(valid, :);

[t, order] = sort(t);
pos = pos(order, :);
quat = quat(order, :);
[t, uniqueIdx] = unique(t, 'stable');
pos = pos(uniqueIdx, :);
quat = quat(uniqueIdx, :);
end

function t = extractHeaderTime(msg)
t = NaN;
[header, hasHeader] = getFieldIgnoreCase(msg, 'header');
if ~hasHeader
    return;
end
[stamp, hasStamp] = getFieldIgnoreCase(header, 'stamp');
if ~hasStamp
    return;
end
[sec, okSec] = getNumericFieldIgnoreCase(stamp, 'sec');
[nsec, okNsec] = getNumericFieldIgnoreCase(stamp, 'nanosec');
if ~okNsec
    [nsec, okNsec] = getNumericFieldIgnoreCase(stamp, 'nsec');
end
if okSec && okNsec
    t = sec + nsec * 1e-9;
elseif okSec
    t = sec;
end
end

function poseStruct = findPoseStruct(msg)
poseStruct = [];
[poseField, hasPose] = getFieldIgnoreCase(msg, 'pose');
if ~hasPose
    return;
end
[innerPose, hasInnerPose] = getFieldIgnoreCase(poseField, 'pose');
if hasInnerPose
    poseStruct = innerPose;
else
    poseStruct = poseField;
end
end

function [posOut, quatOut] = transformPoseSeries(tf, posIn, quatIn)
posOut = (tf.R * posIn')' + tf.t;
quatOut = zeros(size(quatIn));
for i = 1:size(quatIn, 1)
    quatOut(i, :) = normalizeQuat(quatMultiply(tf.q, quatIn(i, :)));
end
end

function [tCommon, gtPosI, gtYawI, ekfPosI, ekfYawI] = alignOnGroundTruthTime( ...
    tGt, gtPos, gtYaw, tEkf, ekfPos, ekfYaw)
tStart = max(min(tGt), min(tEkf));
tEnd = min(max(tGt), max(tEkf));
if tEnd <= tStart
    tCommon = [];
    gtPosI = [];
    gtYawI = [];
    ekfPosI = [];
    ekfYawI = [];
    return;
end

mask = tGt >= tStart & tGt <= tEnd;
tCommon = tGt(mask);
gtPosI = gtPos(mask, :);
gtYawI = gtYaw(mask);

[tEkfU, idx] = unique(tEkf(:), 'stable');
ekfPosU = ekfPos(idx, :);
ekfYawU = ekfYaw(idx);

ekfPosI = zeros(numel(tCommon), 3);
for dim = 1:3
    ekfPosI(:, dim) = interp1(tEkfU, ekfPosU(:, dim), tCommon, 'linear');
end
ekfYawI = interpYaw(tEkfU, ekfYawU, tCommon);

valid = all(isfinite(ekfPosI), 2) & isfinite(ekfYawI);
tCommon = tCommon(valid);
gtPosI = gtPosI(valid, :);
gtYawI = gtYawI(valid);
ekfPosI = ekfPosI(valid, :);
ekfYawI = ekfYawI(valid);
end

function yawI = interpYaw(t, yaw, tq)
yawI = interp1(t, unwrap(yaw), tq, 'linear');
yawI = wrapAnglePi(yawI);
end

function [keepMask, lapBoundariesS, trimApplied] = buildLapTrimMask(t, pos, config)
xy = pos(:, 1:2);
startPoint = xy(1, :);
distToStart = vecnorm(xy - startPoint, 2, 2);
stepDist = [0; vecnorm(diff(xy, 1, 1), 2, 2)];
cumDist = cumsum(stepDist);

closeRadius = config.lapCloseRadiusM;
if isfield(config, 'lapRearmRadiusM')
    rearmRadius = config.lapRearmRadiusM;
else
    rearmRadius = 2 * closeRadius;
end

boundaryIdx = 1;
lastBoundary = 1;
armed = false;
for i = 2:numel(t)
    if distToStart(i) > rearmRadius
        armed = true;
    end
    enoughTime = (t(i) - t(lastBoundary)) >= config.minLapDurationS;
    enoughDistance = (cumDist(i) - cumDist(lastBoundary)) >= config.minLapDistanceM;
    if armed && distToStart(i) <= closeRadius && enoughTime && enoughDistance
        boundaryIdx(end + 1) = i; %#ok<AGROW>
        lastBoundary = i;
        armed = false;
    end
end

lapBoundariesS = t(boundaryIdx) - t(1);
keepMask = true(size(t));
trimApplied = false;

if numel(boundaryIdx) < 3
    warning('Lap trimming requested but only %d lap boundary/boundaries found. Keeping full bag.', numel(boundaryIdx));
    return;
end

keepMask(:) = false;
keepMask(boundaryIdx(2):boundaryIdx(end)) = true;
trimApplied = true;
end

function yaw = quatArrayToYaw(q)
yaw = atan2(2 .* (q(:, 4) .* q(:, 3) + q(:, 1) .* q(:, 2)), ...
    1 - 2 .* (q(:, 2) .^ 2 + q(:, 3) .^ 2));
end

function R = quatToRotmLocal(q)
q = normalizeQuat(q);
x = q(1); y = q(2); z = q(3); w = q(4);
R = [ ...
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w); ...
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w); ...
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
end

function q = quatMultiply(a, b)
q = [ ...
    a(4) * b(1) + a(1) * b(4) + a(2) * b(3) - a(3) * b(2), ...
    a(4) * b(2) - a(1) * b(3) + a(2) * b(4) + a(3) * b(1), ...
    a(4) * b(3) + a(1) * b(2) - a(2) * b(1) + a(3) * b(4), ...
    a(4) * b(4) - a(1) * b(1) - a(2) * b(2) - a(3) * b(3)];
end

function q = normalizeQuat(q)
q = double(q);
n = sqrt(sum(q .^ 2));
if n <= eps
    q = [0, 0, 0, 1];
else
    q = q ./ n;
end
end

function a = wrapAnglePi(a)
a = atan2(sin(a), cos(a));
end

function frame = stripFrame(frame)
frame = char(frame);
if startsWith(frame, '/')
    frame = frame(2:end);
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

function [value, found] = getTextFieldIgnoreCase(s, name)
[value, found] = getFieldIgnoreCase(s, name);
if found
    value = char(string(value));
else
    value = '';
end
end

function [value, found] = getNumericFieldIgnoreCase(s, name)
[value, found] = getFieldIgnoreCase(s, name);
if found
    value = double(value);
else
    value = NaN;
end
end
