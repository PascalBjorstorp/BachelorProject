function plotOptiTrackTrajectoryComparison(results, outputDir, showPlot)
%PLOTOPTITRACKTRAJECTORYCOMPARISON Plot OptiTrack and EKF XY trajectories.

fig = makePlotFigure('OptiTrack vs EKF Trajectory', showPlot);
hold on;
colors = lines(max(numel(results), 1));

drawOccupancyMapBackground(getFirstMapData(results));

for i = 1:numel(results)
    c = colors(i, :);
    segments = getTrajectorySegments(results(i));
    for j = 1:numel(segments)
        metricMask = getMetricMask(segments(j));
        plotMaskedTrajectory(segments(j).gtPos(:, 1:2), ~metricMask, '-', ...
            'Color', [0.70 0.70 0.70], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        plotMaskedTrajectory(segments(j).ekfPos(:, 1:2), ~metricMask, '--', ...
            'Color', [0.70 0.70 0.70], 'LineWidth', 1.0, 'HandleVisibility', 'off');

        if j == 1
            gtName = sprintf('%s OptiTrack', results(i).bagName);
            ekfName = sprintf('%s EKF', results(i).bagName);
            handleVisibility = 'on';
        else
            gtName = '';
            ekfName = '';
            handleVisibility = 'off';
        end
        plotMaskedTrajectory(segments(j).gtPos(:, 1:2), metricMask, '-', ...
            'Color', c, 'LineWidth', 1.8, 'DisplayName', gtName, ...
            'HandleVisibility', handleVisibility);
        plotMaskedTrajectory(segments(j).ekfPos(:, 1:2), metricMask, '--', ...
            'Color', c, 'LineWidth', 1.2, 'DisplayName', ekfName, ...
            'HandleVisibility', handleVisibility);
    end
end

axis equal;
grid on;
xlabel('map x [m]');
ylabel('map y [m]');
title('OptiTrack Ground Truth vs EKF Trajectory');
legend('Location', 'bestoutside', 'Interpreter', 'none');

savePlotFigure(fig, outputDir, 'OptiTrack_Trajectory_vs_EKF', showPlot);
end

function segments = getTrajectorySegments(result)
if isfield(result, 'laps') && ~isempty(result.laps)
    segments = result.laps;
else
    segments = result;
end
end

function metricMask = getMetricMask(result)
if isfield(result, 'metricMask') && numel(result.metricMask) == size(result.gtPos, 1)
    metricMask = result.metricMask(:);
elseif isfield(result, 'xyError') && numel(result.xyError) == size(result.gtPos, 1)
    metricMask = isfinite(result.xyError(:));
else
    metricMask = true(size(result.gtPos, 1), 1);
end
end

function plotMaskedTrajectory(xy, mask, lineStyle, varargin)
xyPlot = xy;
mask = mask(:) & all(isfinite(xyPlot), 2);
xyPlot(~mask, :) = NaN;
plot(xyPlot(:, 1), xyPlot(:, 2), lineStyle, varargin{:});
end
