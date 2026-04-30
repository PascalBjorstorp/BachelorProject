function plotOptiTrackXYErrorOverTime(results, outputDir, showPlot)
%PLOTOPTITRACKXYERROROVERTIME Plot x, y, and XY norm error over elapsed time.

fig = makePlotFigure('OptiTrack XY Error Over Time', showPlot);
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
colors = lines(max(numel(results), 1));

nexttile;
hold on;
for i = 1:numel(results)
    plot(results(i).tRel, results(i).xError, 'Color', colors(i, :), 'LineWidth', 1.0);
end
grid on;
ylabel('x error [m]');
title('EKF - OptiTrack Error Over Time');

nexttile;
hold on;
for i = 1:numel(results)
    plot(results(i).tRel, results(i).yError, 'Color', colors(i, :), 'LineWidth', 1.0);
end
grid on;
ylabel('y error [m]');

nexttile;
hold on;
for i = 1:numel(results)
    plot(results(i).tRel, results(i).xyError, 'Color', colors(i, :), 'LineWidth', 1.0, ...
        'DisplayName', results(i).bagName);
end
grid on;
xlabel('time after analysis window start [s]');
ylabel('XY error [m]');
legend('Location', 'bestoutside', 'Interpreter', 'none');

savePlotFigure(fig, outputDir, 'XY_Error_Over_Time', showPlot);
end
