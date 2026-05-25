function plotOptiTrackXYErrorScatter(results, outputDir, showPlot)
%PLOTOPTITRACKXYERRORSCATTER Scatter EKF minus OptiTrack x/y errors.

fig = makePlotFigure('OptiTrack XY Error Scatter', showPlot);
colors = lines(max(numel(results), 1));
n = numel(results);
nCols = ceil(sqrt(n));
nRows = ceil(n / nCols);
tiledlayout(nRows, nCols, 'Padding', 'compact', 'TileSpacing', 'compact');
[xLim, yLim] = globalScatterLimits(results);

for i = 1:numel(results)
    nexttile;
    hold on;
    metricMask = getMetricMask(results(i));
    scatter(results(i).xError(metricMask), results(i).yError(metricMask), 16, colors(i, :), ...
        'filled', 'MarkerFaceAlpha', 0.35);
    xline(0, 'k-');
    yline(0, 'k-');
    axis equal;
    grid on;
    xlim(xLim);
    ylim(yLim);
    xlabel('x error [m]');
    ylabel('y error [m]');
    title(results(i).bagName, 'Interpreter', 'none');
end

savePlotFigure(fig, outputDir, 'XY_Error_Scatter', showPlot);
end

function metricMask = getMetricMask(result)
if isfield(result, 'metricMask') && numel(result.metricMask) == numel(result.xError)
    metricMask = result.metricMask(:);
else
    metricMask = true(numel(result.xError), 1);
end
end

function [xLim, yLim] = globalScatterLimits(results)
xAll = [];
yAll = [];
for i = 1:numel(results)
    metricMask = getMetricMask(results(i));
    x = results(i).xError(metricMask);
    y = results(i).yError(metricMask);
    finite = isfinite(x) & isfinite(y);
    xAll = [xAll; x(finite)]; %#ok<AGROW>
    yAll = [yAll; y(finite)]; %#ok<AGROW>
end

if isempty(xAll) || isempty(yAll)
    xLim = [-1, 1];
    yLim = [-1, 1];
    return;
end

xPad = max(0.01, 0.05 * rangeOrOne(xAll));
yPad = max(0.01, 0.05 * rangeOrOne(yAll));
xLim = [min(xAll) - xPad, max(xAll) + xPad];
yLim = [min(yAll) - yPad, max(yAll) + yPad];
end

function r = rangeOrOne(values)
r = max(values) - min(values);
if ~isfinite(r) || r <= 0
    r = 1;
end
end
