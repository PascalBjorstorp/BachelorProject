clc;
close all;
% Extract the map and logged local raceline from one ROS 2 bag.
%
% Optional workspace inputs before running:
%   bagPath       - bag folder containing metadata.yaml
%   outputDir     - folder for extracted files
%   racelineStride - read every Nth /local_raceline message

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

if ~exist('bagPath', 'var') || isempty(bagPath)
    bagPath = fullfile(scriptDir, '..', 'bags', 'OptitrackBags', 'AMCL', ...
        'ParticleCount800', 'OptitrackBenchmark_20260430_120324');
end
if ~exist('outputDir', 'var') || isempty(outputDir)
    outputDir = fullfile(scriptDir, 'plots', 'BagMapAndRaceline');
end
if ~exist('racelineStride', 'var') || isempty(racelineStride)
    racelineStride = 25;
end

ensureDirectory(outputDir);

bag = ros2bagreader(bagPath);
[~, bagName] = fileparts(stripTrailingSeparator(bagPath));
bagOutputDir = fullfile(outputDir, bagName);
ensureDirectory(bagOutputDir);

fprintf('Bag        : %s\n', bagPath);
fprintf('Output     : %s\n', bagOutputDir);
fprintf('Stride     : %d local-raceline messages\n', racelineStride);

mapPaths = extractMapTopic(bag, bagOutputDir);
racelinePaths = extractLocalRacelineTopic(bag, bagOutputDir, racelineStride);

fprintf('\nMap YAML   : %s\n', mapPaths.yaml);
fprintf('Map PGM    : %s\n', mapPaths.pgm);
fprintf('Raceline   : %s\n', racelinePaths.uniqueCsv);

function paths = extractMapTopic(bag, outputDir)
paths = struct('yaml', "", 'pgm', "", 'metadataCsv', "");
topics = string(bag.AvailableTopics.Properties.RowNames);
if ~any(topics == "/map")
    error('Bag has no /map topic.');
end

msgs = readMessages(select(bag, 'Topic', '/map'), 1);
if isempty(msgs)
    error('/map topic is empty.');
end

msg = struct(msgs{1});
info = struct(msg.info);
origin = struct(info.origin);
pos = struct(origin.position);
orient = struct(origin.orientation);

width = double(info.width);
height = double(info.height);
resolution = double(info.resolution);
originX = double(pos.x);
originY = double(pos.y);
originYaw = quatToYaw([double(orient.w), double(orient.x), double(orient.y), double(orient.z)]);

data = double(msg.data(:));
if numel(data) < width * height
    error('/map data has %d cells, expected %d.', numel(data), width * height);
end
data = data(1:(width * height));
grid = reshape(data, width, height).';

image = uint8(205 * ones(height, width));
image(grid == 0) = 254;
image(grid >= 50) = 0;
image = flipud(image);

paths.pgm = fullfile(outputDir, 'bag_map.pgm');
paths.yaml = fullfile(outputDir, 'bag_map.yaml');
paths.metadataCsv = fullfile(outputDir, 'bag_map_metadata.csv');

imwrite(image, paths.pgm);

fid = fopen(paths.yaml, 'w');
if fid < 0
    error('Could not write %s', paths.yaml);
end
fprintf(fid, 'image: bag_map.pgm\n');
fprintf(fid, 'mode: trinary\n');
fprintf(fid, 'resolution: %.12g\n', resolution);
fprintf(fid, 'origin: [%.12g, %.12g, %.12g]\n', originX, originY, originYaw);
fprintf(fid, 'negate: 0\n');
fprintf(fid, 'occupied_thresh: 0.65\n');
fprintf(fid, 'free_thresh: 0.25\n');
fclose(fid);

T = table(width, height, resolution, originX, originY, originYaw, ...
    originX + width * resolution, originY + height * resolution, ...
    'VariableNames', {'width_cells', 'height_cells', 'resolution_m', ...
    'origin_x_m', 'origin_y_m', 'origin_yaw_rad', 'max_x_m', 'max_y_m'});
writetable(T, paths.metadataCsv);

fprintf('Extracted /map: %dx%d, %.3f m/cell, origin [%.3f %.3f %.3f]\n', ...
    width, height, resolution, originX, originY, originYaw);
end

function paths = extractLocalRacelineTopic(bag, outputDir, stride)
paths = struct('sampledCsv', "", 'uniqueCsv', "", 'trajectoryCsv', "", 'summaryCsv', "");
topics = string(bag.AvailableTopics.Properties.RowNames);
if ~any(topics == "/local_raceline")
    warning('Bag has no /local_raceline topic.');
    return;
end

sel = select(bag, 'Topic', '/local_raceline');
nMessages = sel.NumMessages;
rows = unique([1:stride:nMessages, nMessages]);
msgs = readMessages(sel, rows);

sampleMessage = zeros(0, 1);
samplePoint = zeros(0, 1);
x = zeros(0, 1);
y = zeros(0, 1);
yaw = zeros(0, 1);

for i = 1:numel(msgs)
    msg = struct(msgs{i});
    poses = msg.poses;
    for j = 1:numel(poses)
        ps = struct(poses(j));
        pose = struct(ps.pose);
        pos = struct(pose.position);
        orient = struct(pose.orientation);
        sampleMessage(end + 1, 1) = rows(i); %#ok<AGROW>
        samplePoint(end + 1, 1) = j; %#ok<AGROW>
        x(end + 1, 1) = double(pos.x); %#ok<AGROW>
        y(end + 1, 1) = double(pos.y); %#ok<AGROW>
        yaw(end + 1, 1) = quatToYaw([double(orient.w), double(orient.x), ...
            double(orient.y), double(orient.z)]); %#ok<AGROW>
    end
end

T = table(sampleMessage, samplePoint, x, y, yaw, ...
    'VariableNames', {'message_index', 'point_index', 'x_m', 'y_m', 'yaw_rad'});
paths.sampledCsv = fullfile(outputDir, 'bag_local_raceline_sampled.csv');
writetable(T, paths.sampledCsv);

rounded = round([x, y] / 0.01) * 0.01;
[~, keep] = unique(rounded, 'rows', 'stable');
U = T(sort(keep), :);
paths.uniqueCsv = fullfile(outputDir, 'bag_local_raceline_unique_1cm.csv');
writetable(U, paths.uniqueCsv);
paths.trajectoryCsv = fullfile(outputDir, 'bag_local_raceline_trajectory.csv');
writeTrajectoryCsv(U, paths.trajectoryCsv);

if height(U) >= 2
    stepDistance = hypot(diff(U.x_m), diff(U.y_m));
    closureDistance = hypot(U.x_m(1) - U.x_m(end), U.y_m(1) - U.y_m(end));
    summary = table(height(U), median(stepDistance, 'omitnan'), ...
        max(stepDistance, [], 'omitnan'), nnz(stepDistance > 0.2), closureDistance, ...
        'VariableNames', {'unique_points', 'median_step_m', 'max_step_m', ...
        'n_steps_above_20cm', 'closure_distance_m'});
else
    summary = table(height(U), NaN, NaN, NaN, NaN, ...
        'VariableNames', {'unique_points', 'median_step_m', 'max_step_m', ...
        'n_steps_above_20cm', 'closure_distance_m'});
end
paths.summaryCsv = fullfile(outputDir, 'bag_local_raceline_summary.csv');
writetable(summary, paths.summaryCsv);

fprintf('Extracted /local_raceline: %d sampled messages, %d sampled points, %d unique 1 cm points\n', ...
    numel(rows), height(T), height(U));
end

function writeTrajectoryCsv(T, csvPath)
if height(T) < 3
    warning('Not enough raceline points to write trajectory CSV.');
    return;
end

x = double(T.x_m(:));
y = double(T.y_m(:));
ds = hypot(diff(x), diff(y));
s = [0; cumsum(ds)];
trackLength = s(end) + hypot(x(1) - x(end), y(1) - y(end));

% The /local_raceline pose orientation is not guaranteed to be tangent to
% the ordered path points. The simulator trajectory expects tangent heading.
xPrev = [x(end); x(1:end - 1)];
yPrev = [y(end); y(1:end - 1)];
xNext = [x(2:end); x(1)];
yNext = [y(2:end); y(1)];
yaw = unwrap(atan2(yNext - yPrev, xNext - xPrev));

curvature = zeros(numel(x), 1);
for i = 1:numel(x)
    iPrev = i - 1;
    iNext = i + 1;
    if iPrev < 1
        iPrev = numel(x);
    end
    if iNext > numel(x)
        iNext = 1;
    end
    dsLocal = hypot(x(iNext) - x(iPrev), y(iNext) - y(iPrev));
    if dsLocal > 1e-6
        dyaw = atan2(sin(yaw(iNext) - yaw(iPrev)), cos(yaw(iNext) - yaw(iPrev)));
        curvature(i) = dyaw / dsLocal;
    end
end

maxVelocityMps = 3.5;
velocityScale = 0.80;
vx = (maxVelocityMps * velocityScale) * ones(numel(x), 1);
ax = zeros(numel(x), 1);
dLeft = 1.0 * ones(numel(x), 1);
dRight = 1.0 * ones(numel(x), 1);

fid = fopen(csvPath, 'w');
if fid < 0
    error('Could not write %s', csvPath);
end
fprintf(fid, '# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left_m,d_right_m\n');
for i = 1:numel(x)
    fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
        s(i), x(i), y(i), atan2(sin(yaw(i)), cos(yaw(i))), curvature(i), ...
        vx(i), ax(i), dLeft(i), dRight(i));
end
fclose(fid);

fprintf('Wrote simulator trajectory: %s (length %.2f m)\n', csvPath, trackLength);
end

function yaw = quatToYaw(q)
w = q(1);
x = q(2);
y = q(3);
z = q(4);
yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
end

function ensureDirectory(pathName)
if ~isfolder(pathName)
    mkdir(pathName);
end
end

function out = stripTrailingSeparator(in)
out = char(in);
while numel(out) > 1 && (out(end) == filesep || out(end) == '/' || out(end) == '\')
    out(end) = [];
end
end
