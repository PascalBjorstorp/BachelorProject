%% AnalyzeOptiTrackAMCL
% Compare 240 Hz EKF poses against OptiTrack poses for AMCL benchmark bags.
% Uses each bag's /map and /tf_static map->world transform.

clear; clc; close all;

matlabRoot = fileparts(mfilename('fullpath'));

config = struct();
config.bagRootDir = fullfile(matlabRoot, '..', 'bags', 'OptitrackBags', 'AMCL');
config.outputDir = fullfile(matlabRoot, 'plots', 'OptiTrackAMCL');

config.ekfTopic = '/ekf_pose';
config.optitrackTopic = '/vrpn_mocap/Car2/pose';
config.mapTopic = '/map';
config.mapFrame = 'map';
config.optitrackFrame = 'world';

config.lapDetectionHz = 50;
config.lapCandidateStepS = 0.50;
config.lapIgnoreEdgeS = 2.0;
config.lapRadiusCandidatesM = [0.35 0.45 0.60 0.80];
config.minLapTimeS = 8.0;
config.maxLapTimeS = 20.0;
config.maxCompleteLapsToUse = 5;
config.minCompleteLapsExpected = 4;

config.heatmapCellSizeM = 0.20;
config.minSamplesPerHeatmapCell = 3;
config.cropMinX = 3.0;
config.optitrackFreezeStepM = 1e-6;
config.optitrackFreezeMinDurationS = 0.04;

if ~exist(config.outputDir, 'dir')
    mkdir(config.outputDir);
end

fprintf('Bag root: %s\n', config.bagRootDir);
fprintf('Output  : %s\n', config.outputDir);
fprintf('Crop    : map x >= %.2f m\n', config.cropMinX);
fprintf('Freeze  : OptiTrack step <= %.1g m for >= %.2f s excluded\n', ...
    config.optitrackFreezeStepM, config.optitrackFreezeMinDurationS);

bagFolders = discoverBagFolders(config.bagRootDir);

if isempty(bagFolders)
    error('No ROS 2 bag folders with metadata.yaml found under %s', config.bagRootDir);
end

config.referenceTransform = findReferenceBagTransform(bagFolders, config);

results = repmat(emptyResult(), numel(bagFolders), 1);
allErrors = [];
allGroups = strings(0, 1);

for k = 1:numel(bagFolders)
    bagPath = bagFolders{k};
    [~, runName] = fileparts(bagPath);
    [particleFolder, particleCount] = particleLabelFromPath(bagPath);
    bagLabel = particleFolder;

    fprintf('\n[%d/%d] %s (%s)\n', k, numel(bagFolders), bagLabel, runName);
    result = processBag(bagPath, bagLabel, particleCount, config);
    results(k) = result;

    if result.ok
        allErrors = [allErrors; result.errorM(:)]; %#ok<AGROW>
        allGroups = [allGroups; repmat(string(result.bagLabel), numel(result.errorM), 1)]; %#ok<AGROW>
        plotBagHeatmap(result, config);
        plotSingleBagBoxplot(result, config);
        fprintf('  laps=%d, samples=%d, mean=%.4f m, rmse=%.4f m, p95=%.4f m\n', ...
            result.nLaps, numel(result.errorM), result.meanErrorM, ...
            result.rmseErrorM, result.p95ErrorM);
    else
        fprintf('  skipped: %s\n', result.message);
    end
end

plotCombinedBoxplot(allErrors, allGroups, config);
writeSummaryCsv(results, config);

fprintf('\nDone. Plots saved in %s\n', config.outputDir);

%% Local functions

function result = processBag(bagPath, bagLabel, particleCount, config)
    result = emptyResult();
    result.bagLabel = bagLabel;
    result.bagPath = string(bagPath);
    result.particleCount = particleCount;

    try
        bag = ros2bagreader(bagPath);
        ekf = readPoseWithCovarianceTopic(bag, config.ekfTopic);
        opti = readPoseStampedTopic(bag, config.optitrackTopic);
        mapData = readMapTopic(bag, config.mapTopic);
        bagTf = readStaticTransformFromBag(bag, config.mapFrame, config.optitrackFrame);
    catch err
        result.message = string(err.message);
        return;
    end

    if bagTf.ok
        tf = bagTf;
    elseif config.referenceTransform.ok
        tf = config.referenceTransform;
        tf.source = "sibling bag /tf_static";
    else
        result.message = sprintf("missing /tf_static %s -> %s transform", ...
            config.mapFrame, config.optitrackFrame);
        return;
    end

    if numel(ekf.t) < 2
        result.message = "not enough EKF pose messages";
        return;
    end
    if numel(opti.t) < 2
        result.message = "not enough OptiTrack pose messages";
        return;
    end

    optiMapXYZ = (tf.R * opti.xyz.' + tf.t(:)).';
    optiXY = optiMapXYZ(:, 1:2);
    [opti.t, optiXY] = cleanTimeSeries(opti.t, optiXY);
    [ekf.t, ekf.xy] = cleanTimeSeries(ekf.t, ekf.xy);

    if numel(opti.t) < 2 || numel(ekf.t) < 2
        result.message = "not enough finite pose samples after cleanup";
        return;
    end

    lapInfo = findCompleteLapWindow(opti.t, optiXY, config);
    if ~lapInfo.ok
        result.message = lapInfo.message;
        return;
    end

    visualMask = opti.t >= lapInfo.startTime & opti.t <= lapInfo.endTime;
    visualXY = optiXY(visualMask, :);
    freezeInfo = findOptitrackFreezeIntervals(opti.t, optiXY, config);

    validEkf = ekf.t >= lapInfo.startTime & ekf.t <= lapInfo.endTime & ...
        ekf.t >= min(opti.t) & ekf.t <= max(opti.t);
    ekfT = ekf.t(validEkf);
    ekfXY = ekf.xy(validEkf, :);

    if numel(ekfT) < 2
        result.message = "no EKF samples inside complete-lap window";
        return;
    end

    gtX = interp1(opti.t, optiXY(:, 1), ekfT, 'linear');
    gtY = interp1(opti.t, optiXY(:, 2), ekfT, 'linear');
    gtXY = [gtX gtY];

    good = all(isfinite(gtXY), 2) & all(isfinite(ekfXY), 2);
    ekfT = ekfT(good);
    ekfXY = ekfXY(good, :);
    gtXY = gtXY(good, :);

    if isempty(ekfT)
        result.message = "no synchronized samples after interpolation";
        return;
    end

    nSamplesBeforeFreezeFilter = numel(ekfT);
    frozenReferenceMask = isInHalfOpenIntervals(ekfT, freezeInfo.intervals);
    ekfT = ekfT(~frozenReferenceMask);
    ekfXY = ekfXY(~frozenReferenceMask, :);
    gtXY = gtXY(~frozenReferenceMask, :);
    nSamplesRejectedFrozenOptitrack = sum(frozenReferenceMask);

    if isempty(ekfT)
        result.message = "no synchronized samples after removing frozen OptiTrack intervals";
        return;
    end

    nSamplesBeforeCrop = numel(ekfT);
    cropMask = gtXY(:, 1) >= config.cropMinX;
    ekfT = ekfT(cropMask);
    ekfXY = ekfXY(cropMask, :);
    gtXY = gtXY(cropMask, :);

    if isempty(ekfT)
        result.message = sprintf("no synchronized samples after x >= %.2f crop", ...
            config.cropMinX);
        return;
    end

    delta = ekfXY - gtXY;
    errorM = vecnorm(delta, 2, 2);

    result.ok = true;
    result.message = "ok";
    result.runName = string(getLastPathPart(bagPath));
    result.nLaps = lapInfo.nLaps;
    result.lapStartTime = lapInfo.startTime;
    result.lapEndTime = lapInfo.endTime;
    result.lapDurationS = lapInfo.endTime - lapInfo.startTime;
    result.lapCrossTimes = lapInfo.crossTimes(:);
    result.lapRadiusM = lapInfo.radiusM;
    result.transformSource = tf.source;
    result.cropMinX = config.cropMinX;
    result.visualXY = visualXY;
    result.nSamplesBeforeFreezeFilter = nSamplesBeforeFreezeFilter;
    result.nSamplesRejectedFrozenOptitrack = nSamplesRejectedFrozenOptitrack;
    result.nSamplesBeforeCrop = nSamplesBeforeCrop;
    result.optitrackFreezeIntervals = freezeInfo.intervals;
    result.optitrackFreezeIntervalCount = freezeInfo.intervalCount;
    result.optitrackFreezeDurationS = freezeInfo.durationS;
    result.optitrackFreezeSamples = freezeInfo.sampleCount;
    result.ekfT = ekfT;
    result.ekfXY = ekfXY;
    result.gtXY = gtXY;
    result.errorM = errorM;
    result.map = mapData;
    result.meanErrorM = mean(errorM, 'omitnan');
    result.medianErrorM = median(errorM, 'omitnan');
    result.rmseErrorM = sqrt(mean(errorM.^2, 'omitnan'));
    result.stdErrorM = std(errorM, 'omitnan');
    result.p95ErrorM = prctile(errorM, 95);
    result.maxErrorM = max(errorM);
end

function pose = readPoseWithCovarianceTopic(bag, topicName)
    selection = select(bag, 'Topic', topicName);
    msgs = readMessages(selection);
    t = selection.MessageList.Time;
    n = numel(msgs);
    xy = nan(n, 2);

    for i = 1:n
        p = msgs{i}.pose.pose.position;
        xy(i, :) = [double(p.x), double(p.y)];
    end

    pose = struct('t', double(t(:)), 'xy', xy);
end

function pose = readPoseStampedTopic(bag, topicName)
    selection = select(bag, 'Topic', topicName);
    msgs = readMessages(selection);
    t = selection.MessageList.Time;
    n = numel(msgs);
    xyz = nan(n, 3);

    for i = 1:n
        p = msgs{i}.pose.position;
        xyz(i, :) = [double(p.x), double(p.y), double(p.z)];
    end

    pose = struct('t', double(t(:)), 'xyz', xyz);
end

function mapData = readMapTopic(bag, topicName)
    mapData = struct('ok', false);
    selection = select(bag, 'Topic', topicName);
    if height(selection.MessageList) < 1
        return;
    end

    msgs = readMessages(selection, 1);
    msg = msgs{1};
    width = double(msg.info.width);
    heightMap = double(msg.info.height);
    resolution = double(msg.info.resolution);
    origin = msg.info.origin.position;
    data = double(msg.data(:));

    if numel(data) ~= width * heightMap
        return;
    end

    occ = reshape(data, [width, heightMap]).';
    mapData = struct();
    mapData.ok = true;
    mapData.width = width;
    mapData.height = heightMap;
    mapData.resolution = resolution;
    mapData.originX = double(origin.x);
    mapData.originY = double(origin.y);
    mapData.occupancy = occ;
end

function tf = readStaticTransformFromBag(bag, parentFrame, childFrame)
    tf = struct('ok', false, 'R', eye(3), 't', zeros(3, 1), 'source', "");
    selection = select(bag, 'Topic', '/tf_static');
    if height(selection.MessageList) < 1
        return;
    end

    msgs = readMessages(selection);
    for i = 1:numel(msgs)
        transforms = msgs{i}.transforms;
        for j = 1:numel(transforms)
            tr = transforms(j);
            frameId = string(tr.header.frame_id);
            childId = string(tr.child_frame_id);

            if frameId == string(parentFrame) && childId == string(childFrame)
                q = tr.transform.rotation;
                p = tr.transform.translation;
                tf.ok = true;
                tf.R = quaternionToRotationMatrix([double(q.x), double(q.y), ...
                    double(q.z), double(q.w)]);
                tf.t = [double(p.x); double(p.y); double(p.z)];
                tf.source = "bag /tf_static";
                return;
            elseif frameId == string(childFrame) && childId == string(parentFrame)
                q = tr.transform.rotation;
                p = tr.transform.translation;
                R = quaternionToRotationMatrix([double(q.x), double(q.y), ...
                    double(q.z), double(q.w)]);
                t = [double(p.x); double(p.y); double(p.z)];
                tf.ok = true;
                tf.R = R.';
                tf.t = -R.' * t;
                tf.source = "bag /tf_static inverted";
                return;
            end
        end
    end
end

function referenceTf = findReferenceBagTransform(bagFolders, config)
    referenceTf = struct('ok', false, 'R', eye(3), 't', zeros(3, 1), 'source', "");

    for i = 1:numel(bagFolders)
        try
            bag = ros2bagreader(bagFolders{i});
            tf = readStaticTransformFromBag(bag, config.mapFrame, config.optitrackFrame);
        catch
            tf = referenceTf;
        end

        if tf.ok
            referenceTf = tf;
            referenceTf.source = "sibling bag /tf_static";
            fprintf('Reference %s -> %s transform found in %s\n', ...
                config.mapFrame, config.optitrackFrame, bagFolders{i});
            return;
        end
    end

    warning('No %s -> %s transform found in any bag /tf_static.', ...
        config.mapFrame, config.optitrackFrame);
end

function R = quaternionToRotationMatrix(q)
    x = q(1);
    y = q(2);
    z = q(3);
    w = q(4);
    n = x*x + y*y + z*z + w*w;
    if n <= eps
        R = eye(3);
        return;
    end

    s = 2.0 / n;
    xx = x*x*s; yy = y*y*s; zz = z*z*s;
    xy = x*y*s; xz = x*z*s; yz = y*z*s;
    wx = w*x*s; wy = w*y*s; wz = w*z*s;

    R = [1 - yy - zz, xy - wz, xz + wy; ...
         xy + wz, 1 - xx - zz, yz - wx; ...
         xz - wy, yz + wx, 1 - xx - yy];
end

function [tClean, xyClean] = cleanTimeSeries(t, xy)
    t = double(t(:));
    xy = double(xy);
    valid = isfinite(t) & all(isfinite(xy), 2);
    t = t(valid);
    xy = xy(valid, :);
    [t, order] = sort(t);
    xy = xy(order, :);
    [tClean, uniqueIdx] = unique(t, 'stable');
    xyClean = xy(uniqueIdx, :);
end

function freezeInfo = findOptitrackFreezeIntervals(t, xy, config)
    freezeInfo = struct('intervals', zeros(0, 2), 'intervalCount', 0, ...
        'durationS', 0, 'sampleCount', 0);

    if numel(t) < 2
        return;
    end

    step = vecnorm(diff(xy), 2, 2);
    repeated = step <= config.optitrackFreezeStepM;
    staleSamples = false(size(t));
    intervals = zeros(0, 2);
    runStart = [];

    for i = 1:numel(repeated)
        if repeated(i) && isempty(runStart)
            runStart = i;
        end

        if (~repeated(i) || i == numel(repeated)) && ~isempty(runStart)
            if repeated(i) && i == numel(repeated)
                runEnd = i;
            else
                runEnd = i - 1;
            end

            staleStartIdx = runStart + 1;
            staleEndIdx = runEnd + 1;
            firstFreshAfterIdx = min(staleEndIdx + 1, numel(t));
            durationS = t(firstFreshAfterIdx) - t(runStart);

            if durationS >= config.optitrackFreezeMinDurationS
                intervals(end + 1, :) = [t(staleStartIdx), t(firstFreshAfterIdx)]; %#ok<AGROW>
                staleSamples(staleStartIdx:staleEndIdx) = true;
            end

            runStart = [];
        end
    end

    freezeInfo.intervals = intervals;
    freezeInfo.intervalCount = size(intervals, 1);
    if ~isempty(intervals)
        freezeInfo.durationS = sum(intervals(:, 2) - intervals(:, 1));
    end
    freezeInfo.sampleCount = sum(staleSamples);
end

function inside = isInHalfOpenIntervals(t, intervals)
    inside = false(size(t));
    if isempty(intervals)
        return;
    end

    for i = 1:size(intervals, 1)
        inside = inside | (t >= intervals(i, 1) & t < intervals(i, 2));
    end
end

function lapInfo = findCompleteLapWindow(t, xy, config)
    lapInfo = struct('ok', false, 'message', "no complete laps found", ...
        'startTime', nan, 'endTime', nan, 'crossTimes', [], 'nLaps', 0, ...
        'radiusM', nan);

    if numel(t) < 10
        lapInfo.message = "not enough OptiTrack samples for lap detection";
        return;
    end

    tq = (t(1):1/config.lapDetectionHz:t(end)).';
    if numel(tq) < 10
        lapInfo.message = "OptiTrack duration too short for lap detection";
        return;
    end

    xq = interp1(t, xy(:, 1), tq, 'linear');
    yq = interp1(t, xy(:, 2), tq, 'linear');
    xyq = [xq yq];

    smoothWindow = max(3, 2 * floor(0.12 * config.lapDetectionHz / 2) + 1);
    xyq(:, 1) = smoothdata(xyq(:, 1), 'movmedian', smoothWindow);
    xyq(:, 2) = smoothdata(xyq(:, 2), 'movmedian', smoothWindow);

    best = struct('score', [-inf -inf -inf -inf -inf], 'crossTimes', [], ...
        'radiusM', nan);
    candidateStep = max(1, round(config.lapCandidateStepS * config.lapDetectionHz));
    firstCandidate = find(tq >= tq(1) + config.lapIgnoreEdgeS, 1, 'first');
    lastCandidate = find(tq <= tq(end) - config.lapIgnoreEdgeS, 1, 'last');

    if isempty(firstCandidate) || isempty(lastCandidate) || firstCandidate >= lastCandidate
        lapInfo.message = "lap candidate window too short";
        return;
    end

    for radiusM = config.lapRadiusCandidatesM
        for candidateIdx = firstCandidate:candidateStep:lastCandidate
            point = xyq(candidateIdx, :);
            [crossTimes, crossDistances] = crossingsForPoint(tq, xyq, point, ...
                radiusM, config.minLapTimeS);

            if numel(crossTimes) < 2
                continue;
            end

            intervals = diff(crossTimes);
            intervalOk = intervals >= config.minLapTimeS & intervals <= config.maxLapTimeS;
            [runStart, runEnd] = longestTrueRun(intervalOk);
            if runEnd < runStart
                continue;
            end

            selectedCrossTimes = crossTimes(runStart:runEnd + 1);
            selectedDistances = crossDistances(runStart:runEnd + 1);
            nLaps = numel(selectedCrossTimes) - 1;

            if nLaps > config.maxCompleteLapsToUse
                selectedCrossTimes = selectedCrossTimes(end - config.maxCompleteLapsToUse:end);
                selectedDistances = selectedDistances(end - config.maxCompleteLapsToUse:end);
                nLaps = config.maxCompleteLapsToUse;
            end

            lapIntervals = diff(selectedCrossTimes);
            score = [nLaps, selectedCrossTimes(end) - selectedCrossTimes(1), ...
                -std(lapIntervals), -mean(selectedDistances), -radiusM];

            if compareScore(score, best.score)
                best.score = score;
                best.crossTimes = selectedCrossTimes(:);
                best.radiusM = radiusM;
            end
        end

        if isfinite(best.score(1)) && best.score(1) >= config.minCompleteLapsExpected
            break;
        end
    end

    if ~isfinite(best.score(1))
        lapInfo.message = "no repeated complete-lap crossings found";
        return;
    end

    lapInfo.ok = true;
    lapInfo.message = "ok";
    lapInfo.crossTimes = best.crossTimes(:);
    lapInfo.nLaps = numel(best.crossTimes) - 1;
    lapInfo.startTime = best.crossTimes(1);
    lapInfo.endTime = best.crossTimes(end);
    lapInfo.radiusM = best.radiusM;
end

function [crossTimes, crossDistances] = crossingsForPoint(t, xy, point, radiusM, minLapTimeS)
    distance = vecnorm(xy - point, 2, 2);
    inside = distance <= radiusM;
    edge = diff([false; inside; false]);
    startIdx = find(edge == 1);
    endIdx = find(edge == -1) - 1;

    crossTimes = [];
    crossDistances = [];

    for i = 1:numel(startIdx)
        [minDistance, localIdx] = min(distance(startIdx(i):endIdx(i)));
        idx = startIdx(i) + localIdx - 1;
        crossTime = t(idx);

        if isempty(crossTimes)
            crossTimes(end + 1, 1) = crossTime; %#ok<AGROW>
            crossDistances(end + 1, 1) = minDistance; %#ok<AGROW>
            continue;
        end

        if crossTime - crossTimes(end) < 0.5 * minLapTimeS
            if minDistance < crossDistances(end)
                crossTimes(end) = crossTime;
                crossDistances(end) = minDistance;
            end
        else
            crossTimes(end + 1, 1) = crossTime; %#ok<AGROW>
            crossDistances(end + 1, 1) = minDistance; %#ok<AGROW>
        end
    end
end

function [runStart, runEnd] = longestTrueRun(values)
    runStart = 1;
    runEnd = 0;
    currentStart = [];

    for i = 1:numel(values)
        if values(i) && isempty(currentStart)
            currentStart = i;
        end

        if (~values(i) || i == numel(values)) && ~isempty(currentStart)
            if values(i) && i == numel(values)
                currentEnd = i;
            else
                currentEnd = i - 1;
            end

            if currentEnd - currentStart > runEnd - runStart
                runStart = currentStart;
                runEnd = currentEnd;
            end
            currentStart = [];
        end
    end
end

function bagFolders = discoverBagFolders(rootDir)
    metadataFiles = dir(fullfile(rootDir, '**', 'metadata.yaml'));
    bagFolders = strings(numel(metadataFiles), 1);

    for i = 1:numel(metadataFiles)
        bagFolders(i) = string(metadataFiles(i).folder);
    end

    bagFolders = unique(bagFolders);
    bagFolders = sortByParticleCount(cellstr(bagFolders));
end

function sortedFolders = sortByParticleCount(folders)
    counts = nan(numel(folders), 1);
    for i = 1:numel(folders)
        [~, counts(i)] = particleLabelFromPath(folders{i});
    end
    [~, order] = sort(counts);
    sortedFolders = folders(order);
end

function [label, particleCount] = particleLabelFromPath(pathValue)
    parts = split(string(pathValue), filesep);
    idx = find(startsWith(parts, "ParticleCount"), 1, 'last');
    if isempty(idx)
        label = string(getLastPathPart(pathValue));
        particleCount = nan;
        return;
    end

    label = parts(idx);
    token = regexp(label, '\d+', 'match', 'once');
    particleCount = str2double(token);
end

function value = getLastPathPart(pathValue)
    parts = split(string(pathValue), filesep);
    parts(parts == "") = [];
    value = char(parts(end));
end

function yes = compareScore(score, bestScore)
    yes = false;
    for i = 1:numel(score)
        if score(i) > bestScore(i)
            yes = true;
            return;
        elseif score(i) < bestScore(i)
            return;
        end
    end
end

function plotCombinedBoxplot(allErrors, allGroups, config)
    if isempty(allErrors)
        warning('No errors available for combined boxplot.');
        return;
    end

    fig = figure('Color', 'w', 'Position', [100 100 1100 560]);
    ax = axes(fig);
    boxchart(ax, categorical(allGroups), allErrors, 'BoxFaceColor', [0.20 0.38 0.72], ...
        'MarkerStyle', '.', 'MarkerColor', [0.20 0.20 0.20]);
    grid(ax, 'on');
    ylabel(ax, 'EKF position error vs OptiTrack (m)');
    xlabel(ax, 'Bag');
    title(ax, sprintf('EKF localization error by AMCL bag (x >= %.1f m)', ...
        config.cropMinX));
    set(ax, 'FontSize', 12);
    saveFigure(fig, fullfile(config.outputDir, 'ekf_vs_optitrack_error_boxplot_all_bags.png'));
end

function plotSingleBagBoxplot(result, config)
    fig = figure('Color', 'w', 'Position', [120 120 520 560]);
    ax = axes(fig);
    boxchart(ax, categorical(repmat(string(result.bagLabel), numel(result.errorM), 1)), ...
        result.errorM, 'BoxFaceColor', [0.20 0.38 0.72], ...
        'MarkerStyle', '.', 'MarkerColor', [0.20 0.20 0.20]);
    grid(ax, 'on');
    ylabel(ax, 'EKF position error vs OptiTrack (m)');
    title(ax, sprintf('%s localization error', result.bagLabel), 'Interpreter', 'none');
    subtitle(ax, sprintf('%d complete laps, x >= %.1f m, mean %.3f m, RMSE %.3f m', ...
        result.nLaps, config.cropMinX, result.meanErrorM, result.rmseErrorM));
    set(ax, 'FontSize', 12);
    saveFigure(fig, fullfile(config.outputDir, sprintf('%s_error_boxplot.png', ...
        sanitizeFileName(result.bagLabel))));
end

function plotBagHeatmap(result, config)
    xy = result.gtXY;
    visualXY = result.visualXY;
    errorM = result.errorM;
    cellSize = config.heatmapCellSizeM;

    if result.map.ok
        xMin = result.map.originX;
        yMin = result.map.originY;
        xMax = result.map.originX + result.map.width * result.map.resolution;
        yMax = yMin + result.map.height * result.map.resolution;
    else
        pad = 0.5;
        boundsXY = [xy; visualXY];
        xMin = floor((min(boundsXY(:, 1)) - pad) / cellSize) * cellSize;
        xMax = ceil((max(boundsXY(:, 1)) + pad) / cellSize) * cellSize;
        yMin = floor((min(boundsXY(:, 2)) - pad) / cellSize) * cellSize;
        yMax = ceil((max(boundsXY(:, 2)) + pad) / cellSize) * cellSize;
    end

    xEdges = xMin:cellSize:xMax;
    yEdges = yMin:cellSize:yMax;
    if numel(xEdges) < 2 || numel(yEdges) < 2
        warning('Heatmap bounds invalid for %s.', result.bagLabel);
        return;
    end

    xBin = discretize(xy(:, 1), xEdges);
    yBin = discretize(xy(:, 2), yEdges);
    valid = ~isnan(xBin) & ~isnan(yBin) & isfinite(errorM);
    heatSize = [numel(yEdges) - 1, numel(xEdges) - 1];
    lin = sub2ind(heatSize, yBin(valid), xBin(valid));
    sumGrid = accumarray(lin, errorM(valid), [prod(heatSize), 1], @sum, nan);
    countGrid = accumarray(lin, 1, [prod(heatSize), 1], @sum, 0);
    meanGrid = reshape(sumGrid ./ countGrid, heatSize);
    countGrid = reshape(countGrid, heatSize);
    meanGrid(countGrid < config.minSamplesPerHeatmapCell) = nan;

    xCenters = xEdges(1:end-1) + diff(xEdges) / 2;
    yCenters = yEdges(1:end-1) + diff(yEdges) / 2;

    fig = figure('Color', 'w', 'Position', [100 100 900 780]);
    ax = axes(fig);
    hold(ax, 'on');

    if result.map.ok
        drawMapBackground(ax, result.map);
    end

    if ~isempty(visualXY)
        plot(ax, visualXY(:, 1), visualXY(:, 2), '-', 'Color', [0.45 0.45 0.45], ...
            'LineWidth', 0.35);
    end

    alpha = double(~isnan(meanGrid)) * 0.85;
    imagesc(ax, xCenters, yCenters, meanGrid, 'AlphaData', alpha);
    plot(ax, xy(:, 1), xy(:, 2), '-', 'Color', [0 0 0], 'LineWidth', 0.45);
    xline(ax, config.cropMinX, '--', 'Color', [0.10 0.10 0.10], 'LineWidth', 0.8);

    axis(ax, 'equal');
    xlim(ax, [xMin xMax]);
    ylim(ax, [yMin yMax]);
    set(ax, 'YDir', 'normal', 'FontSize', 12);
    grid(ax, 'on');
    colormap(ax, turbo(256));
    cb = colorbar(ax);
    cb.Label.String = 'Mean EKF position error (m)';
    xlabel(ax, 'map x (m)');
    ylabel(ax, 'map y (m)');
    title(ax, sprintf('%s mean EKF/OptiTrack divergence', result.bagLabel), ...
        'Interpreter', 'none');
    subtitle(ax, sprintf('%d laps shown, stats: x >= %.1f m + fresh OptiTrack, mean %.3f m', ...
        result.nLaps, config.cropMinX, result.meanErrorM));

    saveFigure(fig, fullfile(config.outputDir, sprintf('%s_error_heatmap.png', ...
        sanitizeFileName(result.bagLabel))));
end

function drawMapBackground(ax, mapData)
    occ = mapData.occupancy;
    gray = 0.86 * ones(size(occ));
    gray(occ >= 50) = 0.18;
    gray(occ >= 0 & occ < 50) = 0.97;
    rgb = repmat(gray, [1 1 3]);
    xLimits = [mapData.originX, mapData.originX + mapData.width * mapData.resolution];
    yLimits = [mapData.originY, mapData.originY + mapData.height * mapData.resolution];
    image(ax, xLimits, yLimits, rgb);
    set(ax, 'YDir', 'normal');
end

function writeSummaryCsv(results, config)
    n = numel(results);
    bagLabel = strings(n, 1);
    runName = strings(n, 1);
    bagPath = strings(n, 1);
    ok = false(n, 1);
    message = strings(n, 1);
    particleCount = nan(n, 1);
    nLaps = nan(n, 1);
    nSamples = nan(n, 1);
    nSamplesBeforeFreezeFilter = nan(n, 1);
    nSamplesRejectedFrozenOptitrack = nan(n, 1);
    nSamplesBeforeCrop = nan(n, 1);
    lapDurationS = nan(n, 1);
    cropMinX = nan(n, 1);
    optitrackFreezeIntervalCount = nan(n, 1);
    optitrackFreezeDurationS = nan(n, 1);
    optitrackFreezeSamples = nan(n, 1);
    meanErrorM = nan(n, 1);
    medianErrorM = nan(n, 1);
    rmseErrorM = nan(n, 1);
    stdErrorM = nan(n, 1);
    p95ErrorM = nan(n, 1);
    maxErrorM = nan(n, 1);
    lapRadiusM = nan(n, 1);
    transformSource = strings(n, 1);

    for i = 1:n
        bagLabel(i) = string(results(i).bagLabel);
        runName(i) = string(results(i).runName);
        bagPath(i) = string(results(i).bagPath);
        ok(i) = results(i).ok;
        message(i) = string(results(i).message);
        particleCount(i) = results(i).particleCount;
        nLaps(i) = results(i).nLaps;
        nSamples(i) = numel(results(i).errorM);
        nSamplesBeforeFreezeFilter(i) = results(i).nSamplesBeforeFreezeFilter;
        nSamplesRejectedFrozenOptitrack(i) = results(i).nSamplesRejectedFrozenOptitrack;
        nSamplesBeforeCrop(i) = results(i).nSamplesBeforeCrop;
        lapDurationS(i) = results(i).lapDurationS;
        cropMinX(i) = results(i).cropMinX;
        optitrackFreezeIntervalCount(i) = results(i).optitrackFreezeIntervalCount;
        optitrackFreezeDurationS(i) = results(i).optitrackFreezeDurationS;
        optitrackFreezeSamples(i) = results(i).optitrackFreezeSamples;
        meanErrorM(i) = results(i).meanErrorM;
        medianErrorM(i) = results(i).medianErrorM;
        rmseErrorM(i) = results(i).rmseErrorM;
        stdErrorM(i) = results(i).stdErrorM;
        p95ErrorM(i) = results(i).p95ErrorM;
        maxErrorM(i) = results(i).maxErrorM;
        lapRadiusM(i) = results(i).lapRadiusM;
        transformSource(i) = string(results(i).transformSource);
    end

    summary = table(bagLabel, runName, particleCount, ok, message, nLaps, ...
        nSamples, nSamplesBeforeFreezeFilter, nSamplesRejectedFrozenOptitrack, ...
        nSamplesBeforeCrop, lapDurationS, cropMinX, optitrackFreezeIntervalCount, ...
        optitrackFreezeDurationS, optitrackFreezeSamples, meanErrorM, medianErrorM, rmseErrorM, ...
        stdErrorM, p95ErrorM, maxErrorM, lapRadiusM, transformSource, bagPath);
    writetable(summary, fullfile(config.outputDir, 'OptiTrackAMCL_summary.csv'));
end

function saveFigure(fig, outputPath)
    exportgraphics(fig, outputPath, 'Resolution', 220);
    close(fig);
end

function name = sanitizeFileName(value)
    name = char(regexprep(string(value), '[^\w.-]+', '_'));
end

function result = emptyResult()
    result = struct();
    result.ok = false;
    result.message = "";
    result.bagLabel = "";
    result.runName = "";
    result.bagPath = "";
    result.particleCount = nan;
    result.nLaps = nan;
    result.lapStartTime = nan;
    result.lapEndTime = nan;
    result.lapDurationS = nan;
    result.lapCrossTimes = [];
    result.lapRadiusM = nan;
    result.transformSource = "";
    result.cropMinX = nan;
    result.visualXY = [];
    result.nSamplesBeforeFreezeFilter = nan;
    result.nSamplesRejectedFrozenOptitrack = nan;
    result.nSamplesBeforeCrop = nan;
    result.optitrackFreezeIntervals = zeros(0, 2);
    result.optitrackFreezeIntervalCount = nan;
    result.optitrackFreezeDurationS = nan;
    result.optitrackFreezeSamples = nan;
    result.ekfT = [];
    result.ekfXY = [];
    result.gtXY = [];
    result.errorM = [];
    result.map = struct('ok', false);
    result.meanErrorM = nan;
    result.medianErrorM = nan;
    result.rmseErrorM = nan;
    result.stdErrorM = nan;
    result.p95ErrorM = nan;
    result.maxErrorM = nan;
end
