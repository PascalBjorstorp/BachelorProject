clc;
close all;
% Main plotting driver for localization/MPC benchmark CSV exports.
%
% Optional workspace inputs before pressing Run:
%   csvRootDir   - root folder recursively searched for CSV run folders
%   plotsRootDir - output root for PNG plots
%   showPlots    - true: save and show figures, false: save only

% Get script directory - handle both file and editor temporary paths
scriptDir = fileparts(mfilename('fullpath'));
plotFunctionsDir = fullfile(scriptDir, 'matlab plotting functions');

% If the functions directory doesn't exist (editor temp path), use current working directory
if ~isfolder(plotFunctionsDir)
    % Try current working directory
    scriptDir = pwd;
    plotFunctionsDir = fullfile(scriptDir, 'matlab plotting functions');
    
    % If still not found, try looking for 'csv' folder to infer the correct directory
    if ~isfolder(plotFunctionsDir) && ~isfolder(fullfile(scriptDir, 'csv'))
        % Search up one level
        parentDir = fileparts(scriptDir);
        if isfolder(fullfile(parentDir, 'matlab plotting functions'))
            scriptDir = parentDir;
            plotFunctionsDir = fullfile(scriptDir, 'matlab plotting functions');
        end
    end
end

if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    warning('Could not find matlab plotting functions directory at %s', plotFunctionsDir);
end

% Add relative paths
if isfolder(fullfile(scriptDir, 'csv'))
    addpath(fullfile(scriptDir, 'csv'));
end
if isfolder(fullfile(scriptDir, 'plots'))
    addpath(fullfile(scriptDir, 'plots'));
end

% ===== BATCH INPUT PATHS =====
% Root folder recursively searched for CSV-containing run folders.
if ~exist('csvRootDir', 'var') || isempty(csvRootDir)
    csvRootDir = fullfile(scriptDir, 'csv', '10LapsRun');
end

% Output plots root directory (relative to script directory)
if ~exist('plotsRootDir', 'var') || isempty(plotsRootDir)
    plotsRootDir = fullfile(scriptDir, 'plots');
end

% Show plots flag
if ~exist('showPlots', 'var') || isempty(showPlots)
    showPlots = false;
end

% ===== END BATCH INPUT PATHS =====

if ~isfolder(csvRootDir)
    error('Input CSV root directory does not exist: %s', csvRootDir);
end

if ~isfolder(plotsRootDir)
    mkdir(plotsRootDir);
end

csvRunDirs = findCsvRunFolders(csvRootDir);
if isempty(csvRunDirs)
    error('No CSV files found under input root: %s', csvRootDir);
end

fprintf('Input CSV root    : %s\n', csvRootDir);
fprintf('CSV run folders   : %d\n', numel(csvRunDirs));
fprintf('Show plots        : %s\n', mat2str(logical(showPlots)));

failedRuns = {};

for runIdx = 1:numel(csvRunDirs)
    csvDir = csvRunDirs{runIdx};
    topicCsvDir = csvDir;
    runName = relativeRunName(csvDir, csvRootDir);

    fprintf('\n[%d/%d] Input CSV folder  : %s\n', runIdx, numel(csvRunDirs), csvDir);
    fprintf('[%d/%d] Topic CSV folder  : %s\n', runIdx, numel(csvRunDirs), topicCsvDir);

    try
        %% Load selected run data
        data = loadPipelineMonitorData(csvDir, topicCsvDir);

        [runName, outputDir] = prepareOutputDirectory(runName, plotsRootDir);
        fprintf('[%d/%d] Output plot folder: %s\n', runIdx, numel(csvRunDirs), outputDir);

        %% Figure of pipeline timings
        plotPipelineLatencyOverTime(data, outputDir, showPlots);

        %% Figure of latency histograms
        plotLatencyHistograms(data, outputDir, showPlots);

        %% Figure of latency boxplot
        plotLatencyBoxplot(data, outputDir, showPlots);

        %% Figure of CPU windows
        plotCpuWindows(data, outputDir, showPlots);

        %% Figure of per-core CPU
        plotPerCoreCpu(data, outputDir, showPlots);

        %% Figure of GPU usage
        plotGpuUsage(data, outputDir, showPlots);

        %% Figure of per-node CPU
        plotPerNodeCpu(data, outputDir, showPlots);

        %% Figure of AMCL timings with particle heatmap
        plotAmclTimingParticleHeatmap(data, outputDir, showPlots);

        %% Summary statistics
        printBenchmarkSummary(data);

        fprintf('[%d/%d] Plots saved to %s\n', runIdx, numel(csvRunDirs), outputDir);
    catch ME
        warning('Skipping run folder %s: %s', csvDir, ME.message);
        failedRuns(end + 1, :) = {runName, ME.message}; %#ok<SAGROW>
    end
end

fprintf('\nBatch complete. Processed %d CSV folder(s).\n', numel(csvRunDirs) - size(failedRuns, 1));
if ~isempty(failedRuns)
    fprintf('Failed folder(s):\n');
    for i = 1:size(failedRuns, 1)
        fprintf('  %s: %s\n', failedRuns{i, 1}, failedRuns{i, 2});
    end
end

function csvRunDirs = findCsvRunFolders(rootDir)
%FINDCSVRUNFOLDERS Return every folder under rootDir that contains CSV files.

csvRunDirs = {};
visit(rootDir);

    function visit(folder)
        csvFiles = dir(fullfile(folder, '*.csv'));
        if ~isempty(csvFiles)
            csvRunDirs{end + 1} = folder; %#ok<AGROW>
        end

        entries = dir(folder);
        for j = 1:numel(entries)
            entry = entries(j);
            if entry.isdir && ~strcmp(entry.name, '.') && ~strcmp(entry.name, '..')
                visit(fullfile(folder, entry.name));
            end
        end
    end
end

function runName = relativeRunName(runDir, rootDir)
%RELATIVERUNNAME Build a stable plot folder name from runDir relative to rootDir.

runDir = stripTrailingSeparator(char(runDir));
rootDir = stripTrailingSeparator(char(rootDir));

rootPrefix = [rootDir filesep];
if startsWith(runDir, rootPrefix)
    runName = runDir((numel(rootPrefix) + 1):end);
elseif strcmp(runDir, rootDir)
    [~, runName] = fileparts(rootDir);
else
    [~, runName] = fileparts(runDir);
end

runName = strrep(char(runName), filesep, '_');
end
