function analyze_benchmark_bags()
% ANALYZE_BENCHMARK_BAGS  Read and analyze benchmark localization bags.
%
% This script assumes ROS 2 bags (.db3) created by ros2 bag record
% and requires MATLAB ROS Toolbox with ros2bagreader support.
%
% It will:
%   - Load both benchmark bags from the benchmarkBags folder
%   - Extract poses from '/car_pos/pose' (car odometry / estimate)
%   - Extract poses from the ego pose topic (any topic containing 'ego_pose')
%   - Time-align the two pose streams
%   - Plot trajectories and position error over time
%
% Update paths below if your checkout is in a different location.

%% Configuration
rootDir = '/home/pascal/Documents/BachelorProject';
benchmarkDir = fullfile(rootDir, 'f1tenth_localization', 'Benchmark');
bagsRoot   = fullfile(benchmarkDir, 'bags', 'benchmarkBags');

bagDirs = { ...
    fullfile(bagsRoot, 'GroundTruth'); ...
    fullfile(bagsRoot, 'GroundTruthFastWork') ...
};

% Use the confirmed topic names from your bags
carPoseTopicPattern = '/vrpn_mocap/car_pos/pose';
egoPoseSubstring    = 'ego_pose_world';  % search for this exact ego pose topic

%% Process each bag
for i = 1:numel(bagDirs)
    bagDir = bagDirs{i};
    fprintf('\n=== Processing bag %d/%d: %s ===\n', i, numel(bagDirs), bagDir);

    if ~isfolder(bagDir)
        warning('Bag directory not found: %s', bagDir);
        continue;
    end

    try
        % For ROS 2 bags: pass the directory containing metadata.yaml
        bag = ros2bagreader(bagDir);
    catch ME
        warning('Failed to open bag at %s: %s', bagDir, ME.message);
        continue;
    end

    topicsTbl = bag.AvailableTopics;
    % Handle different MATLAB versions / layouts: topic names may be in a
    % column, or as row names of the table.
    vars = topicsTbl.Properties.VariableNames;
    if ismember("TopicName", vars)
        allTopics = topicsTbl.TopicName;
    elseif ismember("Name", vars)
        allTopics = topicsTbl.Name;
    elseif ismember("Topic", vars)
        allTopics = topicsTbl.Topic;
    else
        % Fallback: use row names as topic names (typical layout where
        % columns are NumMessages / MessageType / MessageDefinition).
        rowNames = topicsTbl.Properties.RowNames;
        if ~isempty(rowNames)
            allTopics = string(rowNames);
        else
            error('Unknown AvailableTopics table format. Columns: %s', strjoin(vars, ', '));
        end
    end

    % Find /car_pos/pose topic (or closest match)
    carTopic = '';
    if any(strcmp(allTopics, carPoseTopicPattern))
        carTopic = carPoseTopicPattern;
    else
        % Try to find something that ends with car_pos/pose
        idx = contains(allTopics, 'car_pos') & contains(allTopics, 'pose');
        if any(idx)
            carTopic = allTopics{find(idx, 1, 'first')};
        end
    end

    if isempty(carTopic)
        warning('No car_pos pose topic found in bag %s', bagDir);
        continue;
    end

    % Find ego pose topic by substring
    egoTopic = '';
    idxEgo = contains(allTopics, egoPoseSubstring);
    if any(idxEgo)
        egoTopic = allTopics{find(idxEgo, 1, 'first')};
    else
        warning('No ego pose topic containing "%s" found in bag %s', egoPoseSubstring, bagDir);
        continue;
    end

    fprintf('Using car topic: %s\n', carTopic);
    fprintf('Using ego topic: %s\n', egoTopic);

    % Read messages
    carSel = select(bag, 'Topic', carTopic);
    egoSel = select(bag, 'Topic', egoTopic);

    % For your MATLAB version, readMessages only accepts the selection
    % object and returns a cell array of ROS 2 message objects.
    carMsgs = readMessages(carSel);
    egoMsgs = readMessages(egoSel);

    if isempty(carMsgs) || isempty(egoMsgs)
        warning('No messages for one of the topics in bag %s', bagDir);
        continue;
    end

    % Extract time and position from PoseStamped / Odometry-like messages
    [tCar, posCar] = extractPoseSeries(carMsgs);
    [tEgo, posEgo] = extractPoseSeries(egoMsgs);

    if isempty(tCar) || isempty(tEgo)
        warning('Failed to extract pose series from bag %s', bagDir);
        continue;
    end

    % Interpolate ego poses onto car timestamps for comparison
    [tCommon, posCarInterp, posEgoInterp] = alignPoseSeries(tCar, posCar, tEgo, posEgo);

    % Compute position error
    posError = vecnorm(posCarInterp - posEgoInterp, 2, 2);

    % Plot results
    figure('Name', sprintf('Trajectories - %s', bagDir), 'NumberTitle', 'off');
    plot(posCar(:,1), posCar(:,2), 'b-', 'DisplayName', 'Car pose'); hold on;
    plot(posEgo(:,1), posEgo(:,2), 'r--', 'DisplayName', 'Ego pose');
    axis equal; grid on;
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Trajectory comparison: %s', bagDir));
    legend('Location', 'best');

    figure('Name', sprintf('Position error - %s', bagDir), 'NumberTitle', 'off');
    plot(tCommon - tCommon(1), posError, 'k-'); grid on;
    xlabel('time [s]'); ylabel('position error [m]');
    title(sprintf('Position error over time: %s', bagDir));

end

end

function [t, pos] = extractPoseSeries(msgs)
% EXTRACTPOSESERIES  Extract time and XYZ position from ROS pose-like messages.
% Works with both ROS 2 message objects and structs by converting and then
% searching for fields case-insensitively.

num = numel(msgs);
t   = zeros(num, 1);
pos = zeros(num, 3);

for k = 1:num
    m = msgs{k};

    % Convert objects to struct for easier field handling
    if ~isstruct(m)
        m = struct(m);
    end

    % ---- Extract time ----
    t(k) = k; % default fallback

    [header, hasHeader] = getFieldIgnoreCase(m, 'header');
    if hasHeader && isstruct(header)
        [stamp, hasStamp] = getFieldIgnoreCase(header, 'stamp');
        if hasStamp && isstruct(stamp)
            % Look for sec + (nanosec or nsec) with any capitalization
            [secVal, hasSec]   = getNumericFieldIgnoreCase(stamp, 'sec');
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

    % ---- Extract position ----
    % Try PoseStamped / Odometry-like layouts: pose.pose.position or pose.position
    p = struct('x', NaN, 'y', NaN, 'z', NaN);

    [poseField, hasPose] = getFieldIgnoreCase(m, 'pose');
    if hasPose && isstruct(poseField)
        % Maybe pose.pose.position
        [innerPose, hasInnerPose] = getFieldIgnoreCase(poseField, 'pose');
        if hasInnerPose && isstruct(innerPose)
            [position, hasPosition] = getFieldIgnoreCase(innerPose, 'position');
        else
            % Maybe pose.position
            [position, hasPosition] = getFieldIgnoreCase(poseField, 'position');
        end
    else
        hasPosition = false;
        position = [];
    end

    if ~hasPosition
        % Maybe top-level position
        [position, hasPosition] = getFieldIgnoreCase(m, 'position');
    end

    if hasPosition && isstruct(position)
        [px, hasX] = getNumericFieldIgnoreCase(position, 'x');
        [py, hasY] = getNumericFieldIgnoreCase(position, 'y');
        [pz, hasZ] = getNumericFieldIgnoreCase(position, 'z');
        if hasX, p.x = px; end
        if hasY, p.y = py; end
        if hasZ, p.z = pz; end
    end

    pos(k, :) = [double(p.x), double(p.y), double(p.z)];
end

% Remove NaN rows if any
valid = all(isfinite(pos), 2);

t   = t(valid);
pos = pos(valid, :);

end

function [value, found] = getFieldIgnoreCase(s, name)
% GETFIELDIGNORECASE  Get a (sub)field by name, ignoring case.

value = [];
found = false;

if ~isstruct(s)
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
% GETNUMERICFIELDIGNORECASE  Get a numeric field by name, ignoring case.

[v, found] = getFieldIgnoreCase(s, name);
if found
    value = double(v);
else
    value = NaN;
end

end

function [tCommon, posAInterp, posBInterp] = alignPoseSeries(tA, posA, tB, posB)
% ALIGNPOSESERIES  Align two pose time series on a common time base.

% Use overlapping time interval
tStart = max(min(tA), min(tB));
tEnd   = min(max(tA), max(tB));

if tEnd <= tStart
    warning('No overlapping time interval between two pose series.');
    tCommon   = [];
    posAInterp = [];
    posBInterp = [];
    return;
end

% Choose common time base as car timestamps within overlap
maskA = tA >= tStart & tA <= tEnd;
tCommon = tA(maskA);
posAInterp = posA(maskA, :);

% Ensure tCommon is strictly increasing
[tCommon, sortIdx] = sort(tCommon);
posAInterp = posAInterp(sortIdx, :);

% Prepare ego series: ensure unique, sorted time stamps for interp1
[tBUnique, ia] = unique(tB(:));
posBUnique = posB(ia, :);

% Interpolate ego onto tCommon
posBInterp = zeros(numel(tCommon), 3);
for dim = 1:3
    posBInterp(:, dim) = interp1(tBUnique, posBUnique(:, dim), tCommon, 'linear', 'extrap');
end

end
