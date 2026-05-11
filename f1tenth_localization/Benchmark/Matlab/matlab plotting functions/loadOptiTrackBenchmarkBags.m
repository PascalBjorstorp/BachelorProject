function results = loadOptiTrackBenchmarkBags(bagRootDir, config)
%LOADOPTITRACKBENCHMARKBAGS Load bags and compute lap-wise EKF minus OptiTrack errors.

bagDirs = discoverOptiTrackBagDirs(bagRootDir);
bagDirs = filterBagDirsByStartEpoch(bagDirs, config);
if isempty(bagDirs)
    error('No rosbag metadata.yaml files found under %s', bagRootDir);
end

fprintf('Found %d bag(s)\n', numel(bagDirs));

results = struct( ...
    'bagName', {}, ...
    'sourceBagName', {}, ...
    'bagPath', {}, ...
    'mapData', {}, ...
    'particleCount', {}, ...
    'durationS', {}, ...
    'nSamples', {}, ...
    'nMetricSamples', {}, ...
    'nExcludedSamples', {}, ...
    'nLaps', {}, ...
    'lapDurationsS', {}, ...
    'trimApplied', {}, ...
    'lapBoundariesS', {}, ...
    'laps', {}, ...
    'nYawIsolatedSamplesRemoved', {}, ...
    'nYawOutlierLapsExcluded', {}, ...
    'yawOutlierLapsExcluded', {}, ...
    'optitrackSamplesRaw', {}, ...
    'optitrackSamplesValid', {}, ...
    'optitrackSamplesRejected', {}, ...
    't', {}, ...
    'tRel', {}, ...
    'metricMask', {}, ...
    'excludedMask', {}, ...
    'gtPos', {}, ...
    'gtQuat', {}, ...
    'gtRoll', {}, ...
    'gtPitch', {}, ...
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
    'stdRmseXYError', {}, ...
    'meanAbsYawError', {}, ...
    'stdYawError', {}, ...
    'rmseYawError', {}, ...
    'stdRmseYawError', {}, ...
    'maxXYError', {}, ...
    'maxAbsYawError', {} ...
);

[fallbackTf, fallbackTfSource] = readFirstAvailableMapToWorldTransform(bagDirs, config);
if ~isempty(fallbackTf)
    fprintf('Fallback map->world TF source: %s\n', fallbackTfSource);
end

for i = 1:numel(bagDirs)
    bagPath = bagDirs{i};
    [bagName, sourceBagName, particleCount] = getOptiTrackBagNames(bagPath);
    fprintf('\n=== Processing %d/%d: %s (%s) ===\n', ...
        i, numel(bagDirs), bagName, sourceBagName);

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
    mapData = readMapDataFromBag(bag, config, topics);

    try
        tf = readStaticMapToWorldTransform(bag, config);
        fallbackTf = tf;
        fallbackTfSource = bagName;
    catch ME
        if isempty(fallbackTf)
            warning('Skipping %s: failed to read map->world TF: %s', bagName, ME.message);
            continue;
        end
        warning('Using %s map->world TF for %s because this bag has no usable TF: %s', ...
            fallbackTfSource, bagName, ME.message);
        tf = fallbackTf;
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

    [tGtRaw, gtPosRaw, gtQuatRaw, optiStats] = filterOptiTrackSeries( ...
        tGtRaw, gtPosRaw, gtQuatRaw, config);
    if numel(tGtRaw) < 2
        warning('Skipping %s: insufficient valid OptiTrack samples after filtering', bagName);
        continue;
    end

    [gtPos, gtQuat] = transformPoseSeries(tf, gtPosRaw, gtQuatRaw);
    [gtRoll, gtPitch, gtYaw] = quatArrayToRpy(gtQuat);

    [tCommon, gtPosI, gtQuatI, gtRollI, gtPitchI, gtYawI, ekfPosI, ekfYawI] = alignOnGroundTruthTime( ...
        tGtRaw, gtPos, gtQuat, gtRoll, gtPitch, gtYaw, tEkf, ekfPos, ekfYaw);
    if isempty(tCommon)
        warning('Skipping %s: no overlapping EKF/OptiTrack time window', bagName);
        continue;
    end

    [lapSegments, lapBoundariesS, trimApplied] = buildLapSegments(tCommon, gtPosI, config);
    if isempty(lapSegments)
        warning('Skipping %s: no usable complete laps found', bagName);
        continue;
    end

    xErrAll = ekfPosI(:, 1) - gtPosI(:, 1);
    yErrAll = ekfPosI(:, 2) - gtPosI(:, 2);
    xyErrAll = hypot(xErrAll, yErrAll);
    yawErrAll = wrapAnglePi(ekfYawI - gtYawI);
    [yawErrAll, yawSpikeFilter] = filterIsolatedYawOutliers(yawErrAll, config);
    metricMaskAll = buildMetricMask(gtPosI, config);
    xErrMetricAll = setInvalidToNan(xErrAll, metricMaskAll);
    yErrMetricAll = setInvalidToNan(yErrAll, metricMaskAll);
    xyErrMetricAll = setInvalidToNan(xyErrAll, metricMaskAll);
    yawErrMetricAll = setInvalidToNan(yawErrAll, metricMaskAll);

    laps = buildLapResults(lapSegments, tCommon, gtPosI, gtYawI, ekfPosI, ekfYawI, ...
        xErrMetricAll, yErrMetricAll, xyErrMetricAll, yawErrMetricAll, metricMaskAll);
    [laps, lapSegments, yawOutlierExclusion] = excludeYawOutlierLaps( ...
        laps, lapSegments, config, particleCount);
    if isempty(laps)
        warning('Skipping %s: no lap samples survived filtering', bagName);
        continue;
    end

    keepMask = buildMaskFromLapSegments(numel(tCommon), lapSegments);
    if nnz(keepMask) < 2
        warning('Skipping %s: insufficient samples after lap trimming', bagName);
        continue;
    end

    tTrim = tCommon(keepMask);
    gtPosTrim = gtPosI(keepMask, :);
    gtQuatTrim = gtQuatI(keepMask, :);
    gtRollTrim = gtRollI(keepMask);
    gtPitchTrim = gtPitchI(keepMask);
    gtYawTrim = gtYawI(keepMask);
    ekfPosTrim = ekfPosI(keepMask, :);
    ekfYawTrim = ekfYawI(keepMask);
    metricMask = metricMaskAll(keepMask);
    xErr = xErrMetricAll(keepMask);
    yErr = yErrMetricAll(keepMask);
    xyErr = xyErrMetricAll(keepMask);
    yawErr = yawErrMetricAll(keepMask);
    tRel = tTrim - tTrim(1);

    r = struct();
    r.bagName = bagName;
    r.sourceBagName = sourceBagName;
    r.bagPath = bagPath;
    r.mapData = mapData;
    r.particleCount = particleCount;
    r.nSamples = numel(tRel);
    r.nMetricSamples = nnz(metricMask);
    r.nExcludedSamples = nnz(~metricMask);
    r.nLaps = numel(laps);
    r.lapDurationsS = [laps.durationS];
    r.durationS = sum(r.lapDurationsS, 'omitnan');
    r.trimApplied = trimApplied;
    r.lapBoundariesS = lapBoundariesS;
    r.laps = laps;
    r.nYawIsolatedSamplesRemoved = yawSpikeFilter.nSamples;
    r.nYawOutlierLapsExcluded = yawOutlierExclusion.nLaps;
    r.yawOutlierLapsExcluded = yawOutlierExclusion.lapNumbers;
    r.optitrackSamplesRaw = optiStats.rawSamples;
    r.optitrackSamplesValid = optiStats.validSamples;
    r.optitrackSamplesRejected = optiStats.rejectedSamples;
    r.t = tTrim;
    r.tRel = tRel;
    r.metricMask = metricMask;
    r.excludedMask = ~metricMask;
    r.gtPos = gtPosTrim;
    r.gtQuat = gtQuatTrim;
    r.gtRoll = gtRollTrim;
    r.gtPitch = gtPitchTrim;
    r.gtYaw = gtYawTrim;
    r.ekfPos = ekfPosTrim;
    r.ekfYaw = ekfYawTrim;
    r.xError = xErr;
    r.yError = yErr;
    r.xyError = xyErr;
    r.yawError = yawErr;
    r.meanXError = meanMetric(laps, 'meanXError');
    r.stdXError = stdMetric(laps, 'meanXError');
    r.meanYError = meanMetric(laps, 'meanYError');
    r.stdYError = stdMetric(laps, 'meanYError');
    r.meanXYError = meanMetric(laps, 'meanXYError');
    r.stdXYError = stdMetric(laps, 'meanXYError');
    r.rmseXYError = meanMetric(laps, 'rmseXYError');
    r.stdRmseXYError = stdMetric(laps, 'rmseXYError');
    r.meanAbsYawError = meanMetric(laps, 'meanAbsYawError');
    r.stdYawError = stdMetric(laps, 'meanAbsYawError');
    r.rmseYawError = meanMetric(laps, 'rmseYawError');
    r.stdRmseYawError = stdMetric(laps, 'rmseYawError');
    r.maxXYError = maxMetric(laps, 'maxXYError');
    r.maxAbsYawError = maxMetric(laps, 'maxAbsYawError');

    results(end + 1) = r; %#ok<AGROW>

    fprintf('Laps used: %d | samples: %d | duration: %.2f s | trim: %s\n', ...
        r.nLaps, r.nSamples, r.durationS, mat2str(r.trimApplied));
    fprintf('OptiTrack samples: raw %d | valid %d | rejected %d\n', ...
        r.optitrackSamplesRaw, r.optitrackSamplesValid, r.optitrackSamplesRejected);
    if r.nYawIsolatedSamplesRemoved > 0
        fprintf('Removed isolated yaw spike samples: %d\n', r.nYawIsolatedSamplesRemoved);
    end
    if r.nYawOutlierLapsExcluded > 0
        fprintf('Excluded yaw-outlier laps: %s\n', mat2str(r.yawOutlierLapsExcluded));
    end
    fprintf('Metric samples: %d | grey/excluded samples: %d\n', ...
        r.nMetricSamples, r.nExcludedSamples);
    fprintf('Lap mean x/y error: %.4f / %.4f m | lap mean XY: %.4f +/- %.4f m | lap mean |yaw|: %.4f rad\n', ...
        r.meanXError, r.meanYError, r.meanXYError, r.stdXYError, r.meanAbsYawError);
end
end

function bagDirs = discoverOptiTrackBagDirs(rootDir)
meta = dir(fullfile(rootDir, '**', 'metadata.yaml'));
bagDirs = cell(numel(meta), 1);
for i = 1:numel(meta)
    bagDirs{i} = meta(i).folder;
end
bagDirs = unique(bagDirs, 'stable');

particleCounts = nan(numel(bagDirs), 1);
for i = 1:numel(bagDirs)
    [~, ~, particleCounts(i)] = getOptiTrackBagNames(bagDirs{i});
end
sortKey = particleCounts;
sortKey(~isfinite(sortKey)) = inf;
[~, order] = sort(sortKey);
bagDirs = bagDirs(order);
end

function [bagName, sourceBagName, particleCount] = getOptiTrackBagNames(bagPath)
[particleDir, sourceBagName] = fileparts(bagPath);
[~, particleName] = fileparts(particleDir);
particleCount = parseParticleCount(particleName);
if isfinite(particleCount)
    bagName = particleName;
else
    bagName = sourceBagName;
end
end

function mapData = readMapDataFromBag(bag, config, topics)
mapData = [];
mapTopic = '/map';
if isfield(config, 'mapTopic') && ~isempty(config.mapTopic)
    mapTopic = char(string(config.mapTopic));
end

if ~any(strcmp(topics, mapTopic))
    return;
end

try
    mapMsgs = readMessages(select(bag, 'Topic', mapTopic));
    if ~isempty(mapMsgs)
        mapData = decodeOccupancyGridMsg(mapMsgs{end});
    end
catch ME
    warning('Could not read map topic %s: %s', mapTopic, ME.message);
end
end

function out = decodeOccupancyGridMsg(msg)
out = [];
if ~isstruct(msg)
    msg = struct(msg);
end

[info, okInfo] = getFieldIgnoreCase(msg, 'info');
[dataVec, okData] = getFieldIgnoreCase(msg, 'data');
if ~okInfo || ~okData
    return;
end

[width, okW] = getNumericFieldIgnoreCase(info, 'width');
[height, okH] = getNumericFieldIgnoreCase(info, 'height');
[res, okR] = getNumericFieldIgnoreCase(info, 'resolution');
[origin, okOrigin] = getFieldIgnoreCase(info, 'origin');
if ~(okW && okH && okR && okOrigin)
    return;
end

[originPos, okPos] = getFieldIgnoreCase(origin, 'position');
if ~okPos
    return;
end

[x0, okX0] = getNumericFieldIgnoreCase(originPos, 'x');
[y0, okY0] = getNumericFieldIgnoreCase(originPos, 'y');
if ~(okX0 && okY0)
    return;
end

width = double(width);
height = double(height);
res = double(res);
dataVec = double(dataVec(:));
if numel(dataVec) < width * height
    return;
end

gridOcc = reshape(dataVec(1:width * height), [width, height])';
rgb = ones(height, width, 3);
isUnknown = (gridOcc == -1);
isOccupied = (gridOcc >= 65);

for c = 1:3
    channel = rgb(:, :, c);
    channel(isUnknown) = 0.93;
    channel(isOccupied) = 0.12;
    rgb(:, :, c) = channel;
end

xWorld = x0 + (0:width-1) * res;
yWorld = y0 + (0:height-1) * res;
out = struct('rgb', rgb, 'xWorld', xWorld, 'yWorld', yWorld);
end

function bagDirs = filterBagDirsByStartEpoch(bagDirs, config)
if ~isfield(config, 'bagStartEpochSeconds') || isempty(config.bagStartEpochSeconds)
    return;
end

targetSeconds = double(config.bagStartEpochSeconds(:));
targetSeconds = targetSeconds(isfinite(targetSeconds));
if isempty(targetSeconds)
    return;
end

toleranceS = getConfigScalar(config, 'bagStartEpochToleranceS', 2.0);
keep = false(numel(bagDirs), 1);
for i = 1:numel(bagDirs)
    startS = readBagStartEpochSeconds(bagDirs{i});
    keep(i) = isfinite(startS) && any(abs(startS - targetSeconds) <= toleranceS);
end

if ~any(keep)
    warning('No bags matched the configured bagStartEpochSeconds filter.');
    return;
end

bagDirs = bagDirs(keep);
end

function startS = readBagStartEpochSeconds(bagDir)
startS = NaN;
metadataPath = fullfile(bagDir, 'metadata.yaml');
if ~isfile(metadataPath)
    return;
end

text = fileread(metadataPath);
token = regexp(text, 'nanoseconds_since_epoch:\s*(\d+)', 'tokens', 'once');
if isempty(token)
    return;
end
startS = str2double(token{1}) * 1e-9;
end

function particleCount = parseParticleCount(name)
tokens = regexp(char(name), 'ParticleCount(\d+)', 'tokens', 'once');
if isempty(tokens)
    particleCount = NaN;
else
    particleCount = str2double(tokens{1});
end
end

function [tf, sourceName] = readFirstAvailableMapToWorldTransform(bagDirs, config)
tf = [];
sourceName = '';
for i = 1:numel(bagDirs)
    [bagName, ~, ~] = getOptiTrackBagNames(bagDirs{i});
    try
        bag = ros2bagreader(bagDirs{i});
        topics = getTopicNames(bag.AvailableTopics);
        if ~any(strcmp(topics, config.staticTfTopic))
            continue;
        end
        tf = readStaticMapToWorldTransform(bag, config);
        sourceName = bagName;
        return;
    catch
    end
end
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
            if getConfigLogical(config, 'invertStaticMapWorldTransform', false)
                tf = invertSe3(tf);
            end
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

function tfInv = invertSe3(tf)
tfInv = struct();
tfInv.R = tf.R';
tfInv.t = -(tfInv.R * tf.t')';
tfInv.q = rotmToQuatLocal(tfInv.R);
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
    t(k) = extractHeaderTime(m);
    if ~isfinite(t(k)) && numel(tOverride) == n && isfinite(tOverride(k))
        t(k) = tOverride(k);
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

function [t, pos, quat, stats] = filterOptiTrackSeries(t, pos, quat, config)
stats = struct();
stats.rawSamples = numel(t);

valid = isfinite(t) & all(isfinite(pos), 2) & all(isfinite(quat), 2);
zeroRadius = getConfigScalar(config, 'optitrackDropoutZeroRadiusM', 1e-6);
if zeroRadius >= 0
    valid = valid & vecnorm(pos(:, 1:2), 2, 2) > zeroRadius;
end

t = t(valid);
pos = pos(valid, :);
quat = quat(valid, :);

freezeDistance = getConfigScalar(config, 'optitrackFreezeDistanceM', 1e-6);
if numel(t) > 1 && freezeDistance >= 0
    repeatedPosition = [false; vecnorm(diff(pos(:, 1:2), 1, 1), 2, 2) <= freezeDistance];
    t = t(~repeatedPosition);
    pos = pos(~repeatedPosition, :);
    quat = quat(~repeatedPosition, :);
end

maxSpeed = getConfigScalar(config, 'optitrackMaxSpeedMps', inf);
if isfinite(maxSpeed) && maxSpeed > 0
    for pass = 1:3
        if numel(t) < 2
            break;
        end
        dt = diff(t);
        stepDist = vecnorm(diff(pos(:, 1:2), 1, 1), 2, 2);
        speed = stepDist ./ max(dt, eps);
        badJump = [false; dt > 0 & speed > maxSpeed];
        if ~any(badJump)
            break;
        end
        t = t(~badJump);
        pos = pos(~badJump, :);
        quat = quat(~badJump, :);
    end
end

stats.validSamples = numel(t);
stats.rejectedSamples = stats.rawSamples - stats.validSamples;
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

function [tCommon, gtPosI, gtQuatI, gtRollI, gtPitchI, gtYawI, ekfPosI, ekfYawI] = alignOnGroundTruthTime( ...
    tGt, gtPos, gtQuat, gtRoll, gtPitch, gtYaw, tEkf, ekfPos, ekfYaw)
tStart = max(min(tGt), min(tEkf));
tEnd = min(max(tGt), max(tEkf));
if tEnd <= tStart
    tCommon = [];
    gtPosI = [];
    gtQuatI = [];
    gtRollI = [];
    gtPitchI = [];
    gtYawI = [];
    ekfPosI = [];
    ekfYawI = [];
    return;
end

mask = tGt >= tStart & tGt <= tEnd;
tCommon = tGt(mask);
gtPosI = gtPos(mask, :);
gtQuatI = gtQuat(mask, :);
gtRollI = gtRoll(mask);
gtPitchI = gtPitch(mask);
gtYawI = gtYaw(mask);

[tEkfU, idx] = unique(tEkf(:), 'stable');
ekfPosU = ekfPos(idx, :);
ekfYawU = ekfYaw(idx);

ekfPosI = zeros(numel(tCommon), 3);
for dim = 1:3
    ekfPosI(:, dim) = interp1(tEkfU, ekfPosU(:, dim), tCommon, 'linear');
end
ekfYawI = interpYaw(tEkfU, ekfYawU, tCommon);

valid = all(isfinite(ekfPosI), 2) & isfinite(ekfYawI) & ...
    all(isfinite(gtQuatI), 2) & isfinite(gtRollI) & isfinite(gtPitchI) & isfinite(gtYawI);
tCommon = tCommon(valid);
gtPosI = gtPosI(valid, :);
gtQuatI = gtQuatI(valid, :);
gtRollI = gtRollI(valid);
gtPitchI = gtPitchI(valid);
gtYawI = gtYawI(valid);
ekfPosI = ekfPosI(valid, :);
ekfYawI = ekfYawI(valid);
end

function metricMask = buildMetricMask(gtPos, config)
metricMask = all(isfinite(gtPos(:, 1:2)), 2);
if isfield(config, 'metricExcludeXLessThanM')
    xMin = config.metricExcludeXLessThanM;
    if isnumeric(xMin) && isscalar(xMin) && isfinite(xMin)
        metricMask = metricMask & gtPos(:, 1) >= double(xMin);
    end
end
end

function values = setInvalidToNan(values, validMask)
values(~validMask) = NaN;
end

function yawI = interpYaw(t, yaw, tq)
yawI = interp1(t, unwrap(yaw), tq, 'linear');
yawI = wrapAnglePi(yawI);
end

function [lapSegments, lapBoundariesS, trimApplied] = buildLapSegments(t, pos, config)
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
lapSegments = struct('sampleIndex', {}, 'lapNumber', {}, 'startIdx', {}, 'endIdx', {}, ...
    'startTimeS', {}, 'endTimeS', {});
trimApplied = false;

if numel(boundaryIdx) < 2
    if isfield(config, 'skipStartupAndIncompleteLaps') && config.skipStartupAndIncompleteLaps
        warning('Lap trimming requested but no complete laps found.');
        return;
    end
    lapSegments(1) = makeLapSegment(1, 1, 1, numel(t), t);
    return;
end

nCompleteLaps = numel(boundaryIdx) - 1;
if isfield(config, 'skipStartupAndIncompleteLaps') && config.skipStartupAndIncompleteLaps
    usableLapNumbers = 2:(nCompleteLaps - 1);
    trimApplied = true;
else
    usableLapNumbers = 1:nCompleteLaps;
end

if isempty(usableLapNumbers)
    warning('Lap trimming requested but only %d complete lap(s) found. Need at least 3 to drop first and last.', ...
        nCompleteLaps);
    return;
end

for k = 1:numel(usableLapNumbers)
    lapNumber = usableLapNumbers(k);
    lapSegments(k) = makeLapSegment(k, lapNumber, boundaryIdx(lapNumber), ...
        boundaryIdx(lapNumber + 1), t);
end
end

function segment = makeLapSegment(sampleIndex, lapNumber, startIdx, endIdx, t)
segment = struct();
segment.sampleIndex = sampleIndex;
segment.lapNumber = lapNumber;
segment.startIdx = startIdx;
segment.endIdx = endIdx;
segment.startTimeS = t(startIdx) - t(1);
segment.endTimeS = t(endIdx) - t(1);
end

function [lapsOut, lapSegmentsOut, info] = excludeYawOutlierLaps(laps, lapSegments, config, particleCount)
lapsOut = laps;
lapSegmentsOut = lapSegments;
info = defaultYawOutlierExclusionInfo();

if isempty(laps) || ~getConfigLogical(config, 'excludeYawOutlierLaps', false)
    return;
end

particleCounts = getConfigNumericVector(config, 'yawOutlierLapParticleCounts', []);
if ~isempty(particleCounts) && (~isfinite(particleCount) || ~any(particleCounts == particleCount))
    return;
end

threshold = getConfigScalar(config, 'yawOutlierLapThresholdRad', inf);
if ~isfinite(threshold) || threshold <= 0
    return;
end

maxYaw = [laps.maxAbsYawError];
dropMask = isfinite(maxYaw) & maxYaw >= threshold;
if ~any(dropMask)
    return;
end

info.nLaps = nnz(dropMask);
info.lapNumbers = [laps(dropMask).lapNumber];
lapsOut = laps(~dropMask);
lapSegmentsOut = lapSegments(~dropMask);
end

function info = defaultYawOutlierExclusionInfo()
info = struct();
info.nLaps = 0;
info.lapNumbers = [];
end

function [yawOut, info] = filterIsolatedYawOutliers(yawErr, config)
yawOut = yawErr;
info = defaultYawSpikeFilterInfo();

if ~getConfigLogical(config, 'yawIsolatedOutlierFilterEnabled', false)
    return;
end

threshold = getConfigScalar(config, 'yawIsolatedOutlierThresholdRad', inf);
maxRunLength = max(0, round(getConfigScalar(config, 'yawIsolatedOutlierMaxRunLength', 2)));
if ~isfinite(threshold) || threshold <= 0 || maxRunLength <= 0
    return;
end

bad = isfinite(yawOut(:)) & abs(yawOut(:)) >= threshold;
if ~any(bad)
    return;
end

removeMask = false(size(bad));
runStarts = find(diff([false; bad]) == 1);
runEnds = find(diff([bad; false]) == -1);
for i = 1:numel(runStarts)
    idx = runStarts(i):runEnds(i);
    if numel(idx) <= maxRunLength
        removeMask(idx) = true;
    end
end

yawOut(removeMask) = NaN;
info.nSamples = nnz(removeMask);
end

function info = defaultYawSpikeFilterInfo()
info = struct();
info.nSamples = 0;
end

function keepMask = buildMaskFromLapSegments(n, lapSegments)
keepMask = false(n, 1);
for i = 1:numel(lapSegments)
    keepMask(lapSegments(i).startIdx:lapSegments(i).endIdx) = true;
end
end

function laps = buildLapResults(lapSegments, t, gtPos, gtYaw, ekfPos, ekfYaw, ...
    xErr, yErr, xyErr, yawErr, metricMask)

laps = struct( ...
    'sampleIndex', {}, ...
    'lapNumber', {}, ...
    'durationS', {}, ...
    'nSamples', {}, ...
    'nMetricSamples', {}, ...
    'nExcludedSamples', {}, ...
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
    'meanYError', {}, ...
    'meanXYError', {}, ...
    'rmseXYError', {}, ...
    'meanAbsYawError', {}, ...
    'rmseYawError', {}, ...
    'maxXYError', {}, ...
    'maxAbsYawError', {} ...
);

for i = 1:numel(lapSegments)
    idx = lapSegments(i).startIdx:lapSegments(i).endIdx;
    if numel(idx) < 2
        continue;
    end

    lap = struct();
    lap.sampleIndex = lapSegments(i).sampleIndex;
    lap.lapNumber = lapSegments(i).lapNumber;
    lap.durationS = t(idx(end)) - t(idx(1));
    lap.nSamples = numel(idx);
    lap.nMetricSamples = nnz(metricMask(idx));
    lap.nExcludedSamples = nnz(~metricMask(idx));
    lap.t = t(idx);
    lap.tRel = t(idx) - t(idx(1));
    lap.gtPos = gtPos(idx, :);
    lap.gtYaw = gtYaw(idx);
    lap.ekfPos = ekfPos(idx, :);
    lap.ekfYaw = ekfYaw(idx);
    lap.xError = xErr(idx);
    lap.yError = yErr(idx);
    lap.xyError = xyErr(idx);
    lap.yawError = yawErr(idx);
    lap.meanXError = mean(lap.xError, 'omitnan');
    lap.meanYError = mean(lap.yError, 'omitnan');
    lap.meanXYError = mean(lap.xyError, 'omitnan');
    lap.rmseXYError = sqrt(mean(lap.xyError .^ 2, 'omitnan'));
    lap.meanAbsYawError = mean(abs(lap.yawError), 'omitnan');
    lap.rmseYawError = sqrt(mean(lap.yawError .^ 2, 'omitnan'));
    lap.maxXYError = max(lap.xyError, [], 'omitnan');
    lap.maxAbsYawError = max(abs(lap.yawError), [], 'omitnan');

    laps(end + 1) = lap; %#ok<AGROW>
end
end

function value = meanMetric(laps, fieldName)
values = metricValues(laps, fieldName);
if isempty(values)
    value = NaN;
else
    value = mean(values, 'omitnan');
end
end

function value = stdMetric(laps, fieldName)
values = metricValues(laps, fieldName);
if isempty(values)
    value = NaN;
elseif numel(values) <= 1
    value = 0;
else
    value = std(values, 0, 'omitnan');
end
end

function value = maxMetric(laps, fieldName)
values = metricValues(laps, fieldName);
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function values = metricValues(laps, fieldName)
if isempty(laps)
    values = [];
else
    values = [laps.(fieldName)];
    values = values(isfinite(values));
end
end

function [roll, pitch, yaw] = quatArrayToRpy(q)
x = q(:, 1);
y = q(:, 2);
z = q(:, 3);
w = q(:, 4);

roll = atan2(2 .* (w .* x + y .* z), ...
    1 - 2 .* (x .^ 2 + y .^ 2));

pitchArg = 2 .* (w .* y - z .* x);
pitchArg = max(-1, min(1, pitchArg));
pitch = asin(pitchArg);

yaw = atan2(2 .* (w .* z + x .* y), ...
    1 - 2 .* (y .^ 2 + z .^ 2));
end

function yaw = quatArrayToYaw(q)
[~, ~, yaw] = quatArrayToRpy(q);
end

function R = quatToRotmLocal(q)
q = normalizeQuat(q);
x = q(1); y = q(2); z = q(3); w = q(4);
R = [ ...
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w); ...
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w); ...
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
end

function q = rotmToQuatLocal(R)
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1.0) * 2;
    qw = 0.25 * s;
    qx = (R(3, 2) - R(2, 3)) / s;
    qy = (R(1, 3) - R(3, 1)) / s;
    qz = (R(2, 1) - R(1, 2)) / s;
elseif R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)
    s = sqrt(1.0 + R(1, 1) - R(2, 2) - R(3, 3)) * 2;
    qw = (R(3, 2) - R(2, 3)) / s;
    qx = 0.25 * s;
    qy = (R(1, 2) + R(2, 1)) / s;
    qz = (R(1, 3) + R(3, 1)) / s;
elseif R(2, 2) > R(3, 3)
    s = sqrt(1.0 + R(2, 2) - R(1, 1) - R(3, 3)) * 2;
    qw = (R(1, 3) - R(3, 1)) / s;
    qx = (R(1, 2) + R(2, 1)) / s;
    qy = 0.25 * s;
    qz = (R(2, 3) + R(3, 2)) / s;
else
    s = sqrt(1.0 + R(3, 3) - R(1, 1) - R(2, 2)) * 2;
    qw = (R(2, 1) - R(1, 2)) / s;
    qx = (R(1, 3) + R(3, 1)) / s;
    qy = (R(2, 3) + R(3, 2)) / s;
    qz = 0.25 * s;
end
q = normalizeQuat([qx, qy, qz, qw]);
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

function value = getConfigScalar(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if isnumeric(candidate) && isscalar(candidate)
        value = double(candidate);
    end
end
end

function value = getConfigLogical(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if islogical(candidate) && isscalar(candidate)
        value = candidate;
    elseif isnumeric(candidate) && isscalar(candidate)
        value = candidate ~= 0;
    end
end
end

function value = getConfigNumericVector(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    candidate = config.(fieldName);
    if isnumeric(candidate)
        value = double(candidate(:))';
        value = value(isfinite(value));
    end
end
end
