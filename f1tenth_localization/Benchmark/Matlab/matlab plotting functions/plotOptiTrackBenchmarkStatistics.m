function plotOptiTrackBenchmarkStatistics(results, outputDir, showPlot)
%PLOTOPTITRACKBENCHMARKSTATISTICS Plot per-bag error boxplots.

fig = makePlotFigure('OptiTrack Benchmark Statistics', showPlot);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plotErrorBox(results, 'x', 'signed x error [m]', 'Signed X Error');

nexttile;
plotErrorBox(results, 'y', 'signed y error [m]', 'Signed Y Error');

nexttile;
plotErrorBox(results, 'xy', 'XY error magnitude [m]', 'XY Magnitude Error');

nexttile;
plotErrorBox(results, 'yaw', '|yaw error| [deg]', 'Absolute Yaw Error');

savePlotFigure(fig, outputDir, 'OptiTrack_Error_Statistics', showPlot);
end

function plotErrorBox(results, metricName, yLabelText, titleText)
[values, groups, labels] = collectMetricValues(results, metricName);
if isempty(values)
    text(0.5, 0.5, 'No metric samples', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center');
    title(titleText);
    grid on;
    return;
end

boxchart(groups, values, 'BoxFaceColor', [0.00, 0.45, 0.74], ...
    'MarkerStyle', '.', 'MarkerColor', [0.25, 0.25, 0.25]);
grid on;
ylabel(yLabelText);
title(titleText);
set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'TickLabelInterpreter', 'none');
xtickangle(25);
end

function [values, groups, labels] = collectMetricValues(results, metricName)
values = [];
groups = [];
labels = string({results.bagName});

for i = 1:numel(results)
    v = getMetricValues(results(i), metricName);
    values = [values; v]; %#ok<AGROW>
    groups = [groups; repmat(i, numel(v), 1)]; %#ok<AGROW>
end
end

function values = getMetricValues(result, metricName)
mask = true(numel(result.xyError), 1);
if isfield(result, 'metricMask') && numel(result.metricMask) == numel(result.xyError)
    mask = result.metricMask(:);
end

switch metricName
    case 'x'
        values = result.xError(:);
    case 'y'
        values = result.yError(:);
    case 'xy'
        values = result.xyError(:);
    case 'yaw'
        values = rad2deg(abs(result.yawError(:)));
    otherwise
        error('Unknown metric: %s', metricName);
end

values = values(mask & isfinite(values));
end
