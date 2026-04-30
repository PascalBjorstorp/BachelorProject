clc;
close all;
% Main driver for OptiTrack-vs-EKF benchmark bags.
%
% Run this file directly from MATLAB. All settings are edited below; there
% are no command-line inputs or launch arguments.

% ===================== USER SETTINGS =====================
skipStartupAndIncompleteLaps = true;
showPlots = true;

matlabRootDir = '/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/Matlab';
bagRootDir = fullfile(matlabRootDir, '..', 'bags', 'OptitrackBags', 'ParticleCount1000');
plotsRootDir = fullfile(matlabRootDir, 'plots', 'OptiTrackBenchmark');

config = struct();
config.ekfTopic = '/ekf_pose';
config.optitrackTopic = '/vrpn_mocap/Car2/pose';
config.staticTfTopic = '/tf_static';
config.mapFrame = 'map';
config.optitrackFrame = 'world';
config.skipStartupAndIncompleteLaps = skipStartupAndIncompleteLaps;

% Lap detection settings. Tune only if lap trimming misses crossings.
config.lapCloseRadiusM = 0.45;
config.lapRearmRadiusM = 0.90;
config.minLapDurationS = 2.0;
config.minLapDistanceM = 2.0;
% =================== END USER SETTINGS ===================

plotFunctionsDir = fullfile(matlabRootDir, 'matlab plotting functions');
if isfolder(plotFunctionsDir)
    addpath(plotFunctionsDir);
else
    error('Could not find plotting functions directory: %s', plotFunctionsDir);
end

if ~isfolder(bagRootDir)
    error('Bag root does not exist: %s', bagRootDir);
end

fprintf('Bag root        : %s\n', bagRootDir);
fprintf('Skip lap trim   : %s\n', mat2str(skipStartupAndIncompleteLaps));
fprintf('EKF topic       : %s\n', config.ekfTopic);
fprintf('OptiTrack topic : %s\n', config.optitrackTopic);

results = loadOptiTrackBenchmarkBags(bagRootDir, config);
if isempty(results)
    error('No valid OptiTrack benchmark bags were loaded from %s', bagRootDir);
end

[~, runName] = fileparts(bagRootDir);
if skipStartupAndIncompleteLaps
    runName = [runName, '_trimmed'];
else
    runName = [runName, '_full'];
end
[runName, outputDir] = prepareOutputDirectory(runName, plotsRootDir);
fprintf('Output plot folder: %s\n', outputDir);

plotOptiTrackTrajectoryComparison(results, outputDir, showPlots);
plotOptiTrackXYErrorScatter(results, outputDir, showPlots);
plotOptiTrackXYErrorOverTime(results, outputDir, showPlots);
plotOptiTrackYawErrorOverTime(results, outputDir, showPlots);
plotOptiTrackBenchmarkStatistics(results, outputDir, showPlots);

summaryTable = printOptiTrackBenchmarkSummary(results);
summaryPath = fullfile(outputDir, 'OptiTrackBenchmark_Summary.csv');
writetable(summaryTable, summaryPath);

fprintf('\nSummary CSV saved to %s\n', summaryPath);
fprintf('Plots saved to %s\n', outputDir);
