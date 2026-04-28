function data = loadPipelineMonitorData(csvDir, topicCsvDir)
%LOADPIPELINEMONITORDATA Read benchmark CSV exports into a plotting struct.

data = struct();
data.csvDir = csvDir;
data.topicCsvDir = topicCsvDir;
data.pipeline = struct('hasData', false);
data.cpu = struct('hasLong', false, 'hasShort', false);
data.gpu = struct('hasFile', false, 'hasLong', false, 'hasShort', false);
data.perCore = struct('hasData', false);
data.node = struct('hasData', false);
data.amcl = struct('hasTimingParticles', false);

pipelinePatterns = {'pipeline_*.csv', 'Pipeline_*.csv', 'pipeline_latency_*.csv', 'PipeLine_*.csv'};
pipelineFile = latestFileAnyOptional(csvDir, pipelinePatterns);
if isempty(pipelineFile)
    parentDir = fileparts(csvDir);
    if ~isempty(parentDir)
        pipelineFile = latestFileAnyOptional(parentDir, pipelinePatterns);
        if ~isempty(pipelineFile)
            fprintf('No pipeline CSV in input folder, using parent folder pipeline CSV.\n');
        end
    end
end

if ~isempty(pipelineFile)
    fprintf('Using pipeline CSV: %s\n', pipelineFile);
    Tp = readtable(pipelineFile, 'VariableNamingRule', 'preserve');
    if height(Tp) > 0
        assertHasColumns(Tp, {'wall_time_ns', 'amcl_to_ekf_ms', 'scan_to_ekf_ms'}, 'Pipeline CSV');
        data.pipeline.hasData = true;
        data.pipeline.file = pipelineFile;
        data.pipeline.table = Tp;
        data.pipeline.t = (double(Tp.wall_time_ns) - double(Tp.wall_time_ns(1))) * 1e-9;
        data.pipeline.warmupSeconds = 3.0;
        data.pipeline.maxLatencyMs = 200.0;
        data.pipeline.hasScanStamp2Scan = ismember('scan_stamp_to_scan_ms', Tp.Properties.VariableNames);
        data.pipeline.hasE2D = ismember('ekf_to_command_ms', Tp.Properties.VariableNames) || ...
            ismember('ekf_to_drive_ms', Tp.Properties.VariableNames);
        data.pipeline.hasD2A = ismember('drive_to_ackermann_ms', Tp.Properties.VariableNames) || ...
            ismember('drive_to_ackermann_cmd_ms', Tp.Properties.VariableNames);
        data.pipeline.hasS2A = ismember('scan_to_ackermann_ms', Tp.Properties.VariableNames) || ...
            ismember('scan_to_command_ms', Tp.Properties.VariableNames) || ...
            ismember('scan_to_drive_ms', Tp.Properties.VariableNames);

        if data.pipeline.hasScanStamp2Scan
            data.pipeline.scanStamp2Scan = sanitizeLatency(double(Tp.scan_stamp_to_scan_ms), data.pipeline.t, data.pipeline.warmupSeconds, data.pipeline.maxLatencyMs);
        else
            data.pipeline.scanStamp2Scan = nan(height(Tp), 1);
        end

        if ismember('scan_to_amcl_ms', Tp.Properties.VariableNames)
            data.pipeline.scan2amcl = double(Tp.scan_to_amcl_ms);
            data.pipeline.scan2amclLabel = 'scan->amcl';
        elseif ismember('walls_to_amcl_ms', Tp.Properties.VariableNames)
            data.pipeline.scan2amcl = double(Tp.walls_to_amcl_ms);
            data.pipeline.scan2amclLabel = 'scan_{walls}->amcl';
        else
            error('Pipeline CSV is missing scan_to_amcl_ms or walls_to_amcl_ms');
        end

        data.pipeline.amcl2ekf = double(Tp.amcl_to_ekf_ms);
        data.pipeline.scan2ekf = double(Tp.scan_to_ekf_ms);
        data.pipeline.e2d = optionalLatency(Tp, {'ekf_to_command_ms', 'ekf_to_drive_ms'}, data.pipeline.t, data.pipeline.warmupSeconds, data.pipeline.maxLatencyMs);
        data.pipeline.d2a = optionalLatency(Tp, {'drive_to_ackermann_ms', 'drive_to_ackermann_cmd_ms'}, data.pipeline.t, data.pipeline.warmupSeconds, data.pipeline.maxLatencyMs);
        data.pipeline.s2a = optionalLatency(Tp, {'scan_to_ackermann_ms', 'scan_to_command_ms', 'scan_to_drive_ms'}, data.pipeline.t, data.pipeline.warmupSeconds, data.pipeline.maxLatencyMs);

        if ismember('amcl_timing_ms', Tp.Properties.VariableNames) && ...
                ismember('amcl_particle_count', Tp.Properties.VariableNames)
            timing = double(Tp.amcl_timing_ms);
            particles = double(Tp.amcl_particle_count);
            validAmcl = isfinite(timing) & timing >= 0 & ...
                isfinite(particles) & particles >= 0;
            if any(validAmcl)
                data.amcl.hasTimingParticles = true;
                data.amcl.timingMs = timing(validAmcl);
                data.amcl.particleCount = particles(validAmcl);
                data.amcl.sampleIndex = (1:nnz(validAmcl))';
                data.amcl.pairMethod = 'pipeline CSV rows';
            end
        end
    end
else
    fprintf('No pipeline file found in input folder.\n');
end

data = loadSystemUsageCsvs(data, csvDir);
data = loadAmclTopicCsvs(data, topicCsvDir);
end

function data = loadSystemUsageCsvs(data, csvDir)
longCpuFile = fullfile(csvDir, 'SystemUsageLong.csv');
shortCpuFile = fullfile(csvDir, 'SystemUsageShort.csv');
gpuFile = fullfile(csvDir, 'SystemUsageGpu.csv');
perCoreCpuFile = fullfile(csvDir, 'SystemUsagePerCore.csv');
nodeProcessCpuFile = fullfile(csvDir, 'SystemUsageNodeProcesses.csv');

if isfile(longCpuFile)
    fprintf('Using long CPU CSV:  %s\n', longCpuFile);
    Tlong = readtable(longCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tlong, {'monotonic_time_ns', 'cpu_long_window_percent'}, 'Long CPU CSV');
    if height(Tlong) > 0
        data.cpu.hasLong = true;
        data.cpu.long = double(Tlong.cpu_long_window_percent);
        data.cpu.tLong = relTimeSeconds(Tlong.monotonic_time_ns);
        data.gpu.hasLong = ismember('gpu_percent', Tlong.Properties.VariableNames);
        if data.gpu.hasLong
            data.gpu.long = double(Tlong.gpu_percent);
        end
    end
else
    fprintf('Long CPU CSV not found. Expected: %s\n', longCpuFile);
end

if isfile(shortCpuFile)
    fprintf('Using short CPU CSV: %s\n', shortCpuFile);
    Tshort = readtable(shortCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tshort, {'monotonic_time_ns', 'cpu_short_window_percent'}, 'Short CPU CSV');
    if height(Tshort) > 0
        data.cpu.hasShort = true;
        data.cpu.short = double(Tshort.cpu_short_window_percent);
        data.cpu.tShort = relTimeSeconds(Tshort.monotonic_time_ns);
        data.gpu.hasShort = ismember('gpu_percent', Tshort.Properties.VariableNames);
        if data.gpu.hasShort
            data.gpu.short = double(Tshort.gpu_percent);
        end
    end
else
    fprintf('Short CPU CSV not found. Expected: %s\n', shortCpuFile);
end

if isfile(gpuFile)
    fprintf('Using GPU CSV:       %s\n', gpuFile);
    Tgpu = readtable(gpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tgpu, {'monotonic_time_ns', 'gpu_percent'}, 'GPU CSV');
    if height(Tgpu) > 0
        data.gpu.hasFile = true;
        data.gpu.standalone = double(Tgpu.gpu_percent);
        data.gpu.t = relTimeSeconds(Tgpu.monotonic_time_ns);
    end
else
    fprintf('GPU CSV not found. Expected: %s\n', gpuFile);
end

if isfile(perCoreCpuFile)
    fprintf('Using per-core CPU CSV: %s\n', perCoreCpuFile);
    Tcore = readtable(perCoreCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tcore, {'monotonic_time_ns'}, 'Per-core CPU CSV');
    coreCols = startsWith(Tcore.Properties.VariableNames, 'cpu_core_') & ...
        endsWith(Tcore.Properties.VariableNames, '_percent');
    coreNames = Tcore.Properties.VariableNames(coreCols);
    if height(Tcore) > 0 && ~isempty(coreNames)
        data.perCore.hasData = true;
        data.perCore.names = coreNames;
        data.perCore.t = relTimeSeconds(Tcore.monotonic_time_ns);
        data.perCore.cpu = zeros(height(Tcore), numel(coreNames));
        for i = 1:numel(coreNames)
            data.perCore.cpu(:, i) = double(Tcore.(coreNames{i}));
        end
    end
else
    fprintf('Per-core CPU CSV not found. Expected: %s\n', perCoreCpuFile);
end

if isfile(nodeProcessCpuFile)
    fprintf('Using node-process CPU CSV: %s\n', nodeProcessCpuFile);
    Tnode = readtable(nodeProcessCpuFile, 'VariableNamingRule', 'preserve');
    assertHasColumns(Tnode, {'monotonic_time_ns', 'node_name', 'pid', 'cpu_percent'}, 'Node Process CPU CSV');
    if height(Tnode) > 0
        data.node.hasData = true;
        data.node.table = Tnode;
    end
else
    fprintf('Node-process CPU CSV not found. Expected: %s\n', nodeProcessCpuFile);
end
end

function data = loadAmclTopicCsvs(data, topicCsvDir)
if data.amcl.hasTimingParticles
    return;
end

amclTimingFile = latestFileAnyOptional(topicCsvDir, {
    'amcl_timing*.csv', '*amcl_timing*.csv', '*amcl-timing*.csv', '*amcl*timing*.csv'});
amclParticleFile = latestFileAnyOptional(topicCsvDir, {
    'amcl_particle_count*.csv', '*amcl_particle_count*.csv', '*amcl-particle-count*.csv', ...
    '*particle_count*.csv', '*amcl*particle*.csv'});

data.amcl.timingFile = amclTimingFile;
data.amcl.particleFile = amclParticleFile;

if ~isempty(amclTimingFile) && ~isempty(amclParticleFile)
    fprintf('Using AMCL timing CSV:         %s\n', amclTimingFile);
    fprintf('Using AMCL particle count CSV: %s\n', amclParticleFile);
    [~, timingValues, timingTimeNs] = readStampedScalarCsv(amclTimingFile, ...
        {'amcl_timing_ms', 'amcl_timing', 'timing_ms', 'processing_time_ms', 'pf_ms', 'data'});
    [~, particleValues, particleTimeNs] = readStampedScalarCsv(amclParticleFile, ...
        {'amcl_particle_count', 'particle_count', 'num_particles', 'particles', 'count', 'data'});

    [timing, particles, method] = pairAmclTimingParticles( ...
        timingValues, timingTimeNs, particleValues, particleTimeNs);

    data.amcl.hasTimingParticles = ~isempty(timing);
    data.amcl.timingMs = timing;
    data.amcl.particleCount = particles;
    data.amcl.sampleIndex = (1:numel(timing))';
    data.amcl.pairMethod = method;

    if ~data.amcl.hasTimingParticles
        fprintf('AMCL timing/particle CSVs were found but no valid paired rows could be created.\n');
    end
elseif isempty(amclTimingFile) && isempty(amclParticleFile)
    fprintf('AMCL timing and particle-count CSVs not found in topic CSV folder.\n');
elseif isempty(amclTimingFile)
    fprintf('AMCL timing CSV not found in topic CSV folder.\n');
else
    fprintf('AMCL particle-count CSV not found in topic CSV folder.\n');
end
end

function y = optionalLatency(T, aliases, t, warmupSeconds, maxLatencyMs)
y = nan(height(T), 1);
for i = 1:numel(aliases)
    if ismember(aliases{i}, T.Properties.VariableNames)
        y = sanitizeLatency(double(T.(aliases{i})), t, warmupSeconds, maxLatencyMs);
        return;
    end
end
end

function y = sanitizeLatency(y, t, warmupSeconds, maxLatencyMs)
y = double(y);
y(y < 0) = NaN;
y(t < warmupSeconds) = NaN;
y(y > maxLatencyMs) = NaN;
end

function t = relTimeSeconds(timeNs)
t = (double(timeNs) - double(timeNs(1))) * 1e-9;
end

function fp = latestFileAnyOptional(dirPath, patterns)
allMatches = [];
for i = 1:numel(patterns)
    matches = dir(fullfile(dirPath, patterns{i}));
    if ~isempty(matches)
        allMatches = [allMatches; matches(:)]; %#ok<AGROW>
    end
end

if isempty(allMatches)
    fp = '';
    return;
end

[~, idx] = max([allMatches.datenum]);
fp = fullfile(dirPath, allMatches(idx).name);
end

function [T, values, timeNs] = readStampedScalarCsv(filePath, valueAliases)
T = readtable(filePath, 'VariableNamingRule', 'preserve');
values = [];
timeNs = [];
if height(T) == 0
    return;
end

valueCol = findColumnByAliases(T, valueAliases);
if isempty(valueCol)
    warning('No scalar value column found in %s', filePath);
    return;
end

values = toDoubleColumn(T.(valueCol));
timeNs = extractTimeNs(T);

valid = isfinite(values);
if ~isempty(timeNs)
    timeNs = timeNs(:);
    valid = valid & isfinite(timeNs);
end

values = values(valid);
if ~isempty(timeNs)
    timeNs = timeNs(valid);
end
end

function timeNs = extractTimeNs(T)
timeNs = [];
singleTimeCol = findColumnByAliases(T, {
    'receive_time_ns', 'recv_time_ns', 'bag_time_ns', 'timestamp_ns', ...
    'time_ns', 'wall_time_ns', 'monotonic_time_ns', 'source_timestamp'});
if ~isempty(singleTimeCol)
    timeNs = toDoubleColumn(T.(singleTimeCol));
    return;
end

secNsecAliases = {
    {'receive_time_sec', 'receive_time_nsec'}, {'recv_time_sec', 'recv_time_nsec'}, ...
    {'bag_time_sec', 'bag_time_nsec'}, {'timestamp_sec', 'timestamp_nsec'}, ...
    {'time_sec', 'time_nsec'}, {'ros_time_sec', 'ros_time_nsec'}, ...
    {'stamp_sec', 'stamp_nsec'}, {'sec', 'nanosec'}, {'sec', 'nsec'}};

for i = 1:numel(secNsecAliases)
    secCol = findColumnByAliases(T, {secNsecAliases{i}{1}});
    nsecCol = findColumnByAliases(T, {secNsecAliases{i}{2}});
    if ~isempty(secCol) && ~isempty(nsecCol)
        timeNs = toDoubleColumn(T.(secCol)) * 1e9 + toDoubleColumn(T.(nsecCol));
        return;
    end
end
end

function colName = findColumnByAliases(T, aliases)
colName = '';
names = T.Properties.VariableNames;
normNames = cellfun(@normalizeColumnName, names, 'UniformOutput', false);
normAliases = cellfun(@normalizeColumnName, aliases, 'UniformOutput', false);

for i = 1:numel(normAliases)
    idx = find(strcmp(normNames, normAliases{i}), 1, 'first');
    if ~isempty(idx)
        colName = names{idx};
        return;
    end
end

for i = 1:numel(normAliases)
    idx = find(endsWith(normNames, normAliases{i}), 1, 'first');
    if ~isempty(idx)
        colName = names{idx};
        return;
    end
end
end

function nameOut = normalizeColumnName(nameIn)
nameOut = lower(regexprep(char(nameIn), '[^a-zA-Z0-9]', ''));
end

function values = toDoubleColumn(col)
if isnumeric(col) || islogical(col)
    values = double(col);
elseif iscell(col)
    values = str2double(string(col));
elseif isstring(col) || ischar(col)
    values = str2double(string(col));
elseif iscategorical(col)
    values = str2double(string(col));
else
    values = double(col);
end
values = values(:);
end

function [timingOut, particleOut, method] = pairAmclTimingParticles( ...
    timingValues, timingTimeNs, particleValues, particleTimeNs)
timingValues = double(timingValues(:));
particleValues = double(particleValues(:));
method = 'receive order';

timingValid = isfinite(timingValues) & timingValues >= 0;
particleValid = isfinite(particleValues) & particleValues >= 0;
if ~isempty(timingTimeNs)
    timingTimeNs = double(timingTimeNs(:));
    timingValid = timingValid & isfinite(timingTimeNs);
end
if ~isempty(particleTimeNs)
    particleTimeNs = double(particleTimeNs(:));
    particleValid = particleValid & isfinite(particleTimeNs);
end

timingValues = timingValues(timingValid);
particleValues = particleValues(particleValid);
if ~isempty(timingTimeNs)
    timingTimeNs = timingTimeNs(timingValid);
end
if ~isempty(particleTimeNs)
    particleTimeNs = particleTimeNs(particleValid);
end

if isempty(timingValues) || isempty(particleValues)
    timingOut = [];
    particleOut = [];
    return;
end

if ~isempty(timingTimeNs) && ~isempty(particleTimeNs)
    [timingTimeNs, timingOrder] = sort(timingTimeNs, 'ascend');
    timingValues = timingValues(timingOrder);
    [timingTimeNs, uniqueTimingIdx] = unique(timingTimeNs, 'stable');
    timingValues = timingValues(uniqueTimingIdx);

    [particleTimeNs, particleOrder] = sort(particleTimeNs, 'ascend');
    particleValues = particleValues(particleOrder);
    [particleTimeNs, uniqueParticleIdx] = unique(particleTimeNs, 'stable');
    particleValues = particleValues(uniqueParticleIdx);

    if numel(particleValues) == 1
        particleOut = repmat(particleValues, size(timingValues));
    else
        timeBase = min([timingTimeNs(:); particleTimeNs(:)]);
        particleRel = particleTimeNs - timeBase;
        timingRel = timingTimeNs - timeBase;
        nearestParticleIdx = interp1(particleRel, 1:numel(particleRel), ...
            timingRel, 'nearest', 'extrap');
        nearestParticleIdx = round(nearestParticleIdx(:));
        nearestParticleIdx = max(1, min(numel(particleValues), nearestParticleIdx));
        particleOut = particleValues(nearestParticleIdx);
    end
    timingOut = timingValues;
    method = 'nearest receive time';
else
    n = min(numel(timingValues), numel(particleValues));
    timingOut = timingValues(1:n);
    if numel(particleValues) >= numel(timingValues)
        particleOut = particleValues(end - n + 1:end);
        method = 'receive order, last N particle samples';
    else
        particleOut = particleValues(1:n);
    end
end

validPair = isfinite(timingOut) & timingOut >= 0 & isfinite(particleOut) & particleOut >= 0;
timingOut = timingOut(validPair);
particleOut = particleOut(validPair);
end
