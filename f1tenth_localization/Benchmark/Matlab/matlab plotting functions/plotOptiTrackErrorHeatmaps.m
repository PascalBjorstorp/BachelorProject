function plotOptiTrackErrorHeatmaps(results, outputDir, showPlot)
%PLOTOPTITRACKERRORHEATMAPS Plot track-position heatmaps for XY and yaw error.

plotOneOptiTrackHeatmap(results, outputDir, showPlot, ...
    'xy', 'XY error [m]', 'OptiTrack XY Error Heatmap', 'OptiTrack_XY_Error_Heatmap');
plotOneOptiTrackHeatmap(results, outputDir, showPlot, ...
    'yaw', '|yaw error| [deg]', 'OptiTrack Yaw Error Heatmap', 'OptiTrack_Yaw_Error_Heatmap');
end

function plotOneOptiTrackHeatmap(results, outputDir, showPlot, metricName, colorLabel, figTitle, fileStem)
fig = makePlotFigure(figTitle, showPlot);
n = numel(results);
nCols = ceil(sqrt(n));
nRows = ceil(n / nCols);
tiledlayout(nRows, nCols, 'Padding', 'compact', 'TileSpacing', 'compact');
colormap(fig, greenYellowRedColormap(256));

[cMin, cMax] = globalColorLimits(results, metricName);
fallbackMap = getFirstMapData(results);

for i = 1:n
    nexttile;
    hold on;

    if isfield(results(i), 'mapData') && ~isempty(results(i).mapData)
        drawOccupancyMapBackground(results(i).mapData);
    else
        drawOccupancyMapBackground(fallbackMap);
    end

    [xyMean, errMean, greyMask, colorMask] = buildMeanLapHeatmap(results(i), metricName);

    scatter(xyMean(greyMask, 1), xyMean(greyMask, 2), 18, [0.70 0.70 0.70], 'filled', ...
        'MarkerFaceAlpha', 0.55);
    scatter(xyMean(colorMask, 1), xyMean(colorMask, 2), 22, errMean(colorMask), 'filled');

    axis equal;
    grid on;
    xlabel('map x [m]');
    ylabel('map y [m]');
    title(sprintf('%s | mean of %d lap(s)', results(i).bagName, results(i).nLaps), ...
        'Interpreter', 'none');
    clim([cMin, cMax]);
    cb = colorbar;
    ylabel(cb, colorLabel);
end

savePlotFigure(fig, outputDir, fileStem, showPlot);
end

function err = getHeatmapMetric(result, metricName)
switch metricName
    case 'xy'
        err = result.xyError;
    case 'yaw'
        err = rad2deg(abs(result.yawError));
    otherwise
        error('Unknown heatmap metric: %s', metricName);
end
end

function [cMin, cMax] = globalColorLimits(results, metricName)
allErr = [];
for i = 1:numel(results)
    [~, errMean, ~, colorMask] = buildMeanLapHeatmap(results(i), metricName);
    allErr = [allErr; errMean(colorMask)]; %#ok<AGROW>
end
allErr = allErr(isfinite(allErr) & allErr >= 0);
if isempty(allErr)
    cMin = 0;
    cMax = 1;
    return;
end
cMin = min(allErr);
cMax = max(allErr);
if ~isfinite(cMin)
    cMin = 0;
end
if ~isfinite(cMax)
    cMax = 1;
end
if cMax <= cMin
    cMax = cMin + 1;
end
end

function [xyMean, errMean, greyMask, colorMask] = buildMeanLapHeatmap(result, metricName)
gridCount = 700;
progressGrid = linspace(0, 1, gridCount)';
laps = getLapArray(result);
nLaps = numel(laps);

xByLap = NaN(gridCount, nLaps);
yByLap = NaN(gridCount, nLaps);
errByLap = NaN(gridCount, nLaps);

for i = 1:nLaps
    xy = laps(i).gtPos(:, 1:2);
    err = getHeatmapMetric(laps(i), metricName);
    progress = normalizedArcLength(xy);

    xByLap(:, i) = interpolateFiniteSeries(progress, xy(:, 1), progressGrid);
    yByLap(:, i) = interpolateFiniteSeries(progress, xy(:, 2), progressGrid);
    errByLap(:, i) = interpolateValidSegments(progress, err, progressGrid);
end

xyMean = [mean(xByLap, 2, 'omitnan'), mean(yByLap, 2, 'omitnan')];
errMean = mean(errByLap, 2, 'omitnan');
finiteXY = all(isfinite(xyMean), 2);
colorMask = finiteXY & isfinite(errMean);
greyMask = finiteXY & ~colorMask;
end

function laps = getLapArray(result)
if isfield(result, 'laps') && ~isempty(result.laps)
    laps = result.laps;
else
    laps = struct( ...
        'gtPos', result.gtPos, ...
        'xyError', result.xyError, ...
        'yawError', result.yawError);
end
end

function progress = normalizedArcLength(xy)
progress = NaN(size(xy, 1), 1);
finiteXY = all(isfinite(xy), 2);
if nnz(finiteXY) < 2
    return;
end

stepLength = [0; vecnorm(diff(xy(:, 1:2), 1, 1), 2, 2)];
stepLength(~isfinite(stepLength)) = 0;
s = cumsum(stepLength);
if s(end) <= 0
    return;
end
progress(finiteXY) = s(finiteXY) ./ s(end);
end

function valuesOut = interpolateFiniteSeries(progress, values, progressGrid)
valid = isfinite(progress) & isfinite(values);
valuesOut = interpolateRun(progress(valid), values(valid), progressGrid);
end

function valuesOut = interpolateValidSegments(progress, values, progressGrid)
valuesOut = NaN(size(progressGrid));
valid = isfinite(progress) & isfinite(values);
if nnz(valid) < 2
    return;
end

runStarts = find(diff([false; valid]) == 1);
runEnds = find(diff([valid; false]) == -1);
for i = 1:numel(runStarts)
    idx = runStarts(i):runEnds(i);
    if numel(idx) < 2
        continue;
    end
    segmentValues = interpolateRun(progress(idx), values(idx), progressGrid);
    fillMask = isfinite(segmentValues);
    valuesOut(fillMask) = segmentValues(fillMask);
end
end

function valuesOut = interpolateRun(progress, values, progressGrid)
valuesOut = NaN(size(progressGrid));
valid = isfinite(progress) & isfinite(values);
progress = progress(valid);
values = values(valid);
if numel(progress) < 2
    return;
end

[progress, uniqueIdx] = unique(progress, 'stable');
values = values(uniqueIdx);
if numel(progress) < 2
    return;
end

valuesOut = interp1(progress, values, progressGrid, 'linear', NaN);
end

function cmap = greenYellowRedColormap(n)
if nargin < 1
    n = 256;
end
green = [0.00, 0.55, 0.20];
yellow = [1.00, 0.85, 0.10];
red = [0.85, 0.05, 0.05];
half = ceil(n / 2);
first = [linspace(green(1), yellow(1), half)', ...
    linspace(green(2), yellow(2), half)', ...
    linspace(green(3), yellow(3), half)'];
secondCount = n - half;
second = [linspace(yellow(1), red(1), secondCount + 1)', ...
    linspace(yellow(2), red(2), secondCount + 1)', ...
    linspace(yellow(3), red(3), secondCount + 1)'];
cmap = [first; second(2:end, :)];
end
