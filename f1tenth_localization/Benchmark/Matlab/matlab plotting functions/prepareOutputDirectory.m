function [runName, outputDir] = prepareOutputDirectory(runName, plotsRootDir)
%PREPAREOUTPUTDIRECTORY Create output folder for one benchmark run.

if ~exist(plotsRootDir, 'dir')
    mkdir(plotsRootDir);
end

if isempty(runName)
    runName = 'benchmark_run';
end
runName = sanitizeFileName(runName);

outputDir = fullfile(plotsRootDir, runName);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
end
