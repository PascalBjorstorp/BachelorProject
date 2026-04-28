clc;
close all;
% Main plotting driver for localization/MPC benchmark CSV exports.
%
% Optional workspace inputs before pressing Run:
%   csvDir       - exact CSV run folder to plot
%   topicCsvDir  - folder containing topic CSVs such as amcl_timing*.csv
%   csvRootDir   - folder where CSV run folders are searched
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

% ===== HARDCODED PATHS =====
% Input CSV folder (relative to script directory)
csvDir = fullfile(scriptDir, 'csv', 'TestRun');

% Output plots root directory (relative to script directory)
plotsRootDir = fullfile(scriptDir, 'plots');

% Show plots flag
showPlots = true;

% ===== END HARDCODED PATHS =====

if ~isdir(csvDir)
    error('Input CSV directory does not exist: %s\nMake sure you are running this script from the Matlab folder or set your working directory to: %s', csvDir, fullfile(scriptDir, '..', '..'));
end

if ~isdir(plotsRootDir)
    error('Output plots directory does not exist: %s', plotsRootDir);
end

topicCsvDir = csvDir;

fprintf('Input CSV folder  : %s\n', csvDir);
fprintf('Topic CSV folder  : %s\n', topicCsvDir);
fprintf('Show plots        : %s\n', mat2str(logical(showPlots)));

%% Load selected run data
data = loadPipelineMonitorData(csvDir, topicCsvDir);

[~, runName] = fileparts(csvDir);
[runName, outputDir] = prepareOutputDirectory(runName, plotsRootDir);

fprintf('Output plot folder: %s\n', outputDir);

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

fprintf('\nPlots saved to %s\n', outputDir);
