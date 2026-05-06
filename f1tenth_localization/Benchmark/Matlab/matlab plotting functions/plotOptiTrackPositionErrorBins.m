function plotOptiTrackPositionErrorBins(results, outputDir, showPlot)
%PLOTOPTITRACKPOSITIONERRORBINS Plot signed x/y position error bins per bag.

fig = makePlotFigure('OptiTrack Position Error Bins', showPlot);
n = numel(results);
nCols = ceil(sqrt(n));
nRows = ceil(n / nCols);
tiledlayout(nRows, nCols, 'Padding', 'compact', 'TileSpacing', 'compact');

[edges, xLimit] = globalErrorBins(results);
xColor = [0.00, 0.45, 0.74];
yColor = [0.85, 0.33, 0.10];

for i = 1:n
    nexttile;
    [xValues, yValues] = metricPositionErrors(results(i));
    if isempty(xValues) && isempty(yValues)
        text(0.5, 0.5, 'No metric samples', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center');
        title(results(i).bagName, 'Interpreter', 'none');
        grid on;
        continue;
    end

    hold on;
    histogram(xValues, edges, 'FaceColor', xColor, 'FaceAlpha', 0.55, ...
        'EdgeColor', 'none', 'DisplayName', 'x error');
    histogram(yValues, edges, 'FaceColor', yColor, 'FaceAlpha', 0.55, ...
        'EdgeColor', 'none', 'DisplayName', 'y error');
    xline(0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(mean(xValues, 'omitnan'), '-', 'Color', xColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off');
    xline(mean(yValues, 'omitnan'), '-', 'Color', yColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off');
    grid on;
    xlim(xLimit);
    xlabel('signed position error [m]');
    ylabel('sample count');
    title(results(i).bagName, 'Interpreter', 'none');
    if i == 1
        legend('Location', 'best');
    end
end

savePlotFigure(fig, outputDir, 'Position_Error_Bins', showPlot);
end

function [xValues, yValues] = metricPositionErrors(result)
xValues = result.xError(:);
yValues = result.yError(:);
mask = true(size(xValues));
if isfield(result, 'metricMask') && numel(result.metricMask) == numel(xValues)
    mask = result.metricMask(:);
end
xValues = xValues(mask & isfinite(xValues));
yValues = yValues(mask & isfinite(yValues));
end

function [edges, xLimit] = globalErrorBins(results)
allValues = [];
for i = 1:numel(results)
    [xValues, yValues] = metricPositionErrors(results(i));
    allValues = [allValues; xValues; yValues]; %#ok<AGROW>
end

if isempty(allValues)
    edges = linspace(-1, 1, 41);
    xLimit = [-1, 1];
    return;
end

limit = max(abs(allValues));
if ~isfinite(limit) || limit <= 0
    limit = 1;
end
limit = max(limit, 0.01);

edges = linspace(-limit, limit, 41);
xLimit = [-limit, limit];
end
