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

scriptDir = fileparts(mfilename('fullpath'));
plotFunctionsDir = fullfile(scriptDir, 'matlab plotting functions');
addpath(plotFunctionsDir);

defaultCsvRootDir = fullfile(scriptDir, 'csv');
defaultPlotsRootDir = fullfile(scriptDir, 'plots');

if ~exist('csvRootDir', 'var') || isempty(csvRootDir)
    csvRootDir = defaultCsvRootDir;
end
if ~exist('plotsRootDir', 'var') || isempty(plotsRootDir)
    plotsRootDir = defaultPlotsRootDir;
end
if ~exist('showPlots', 'var') || isempty(showPlots)
    showPlots = true;
end

csvRootDir = stripTrailingSeparator(csvRootDir);
plotsRootDir = stripTrailingSeparator(plotsRootDir);

%% Find CSV runs
csvRuns = discoverCsvRunFolders(csvRootDir);

fprintf('CSV search folder : %s\n', csvRootDir);
fprintf('Found %d CSV run folder(s).\n', numel(csvRuns));

if ~exist('csvDir', 'var') || isempty(csvDir)
    if isempty(csvRuns)
        error('No CSV run folders found under %s', csvRootDir);
    end
    csvDir = csvRuns(1).path;
    fprintf('No csvDir supplied; using latest CSV run folder.\n');
end

if ~exist('topicCsvDir', 'var') || isempty(topicCsvDir)
    topicCsvDir = csvDir;
end

csvDir = stripTrailingSeparator(csvDir);
topicCsvDir = stripTrailingSeparator(topicCsvDir);

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
