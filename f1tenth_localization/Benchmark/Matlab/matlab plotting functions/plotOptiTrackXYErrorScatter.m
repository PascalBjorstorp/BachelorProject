function plotOptiTrackXYErrorScatter(results, outputDir, showPlot)
%PLOTOPTITRACKXYERRORSCATTER Scatter EKF minus OptiTrack x/y errors.

fig = makePlotFigure('OptiTrack XY Error Scatter', showPlot);
hold on;
colors = lines(max(numel(results), 1));

for i = 1:numel(results)
    scatter(results(i).xError, results(i).yError, 16, colors(i, :), ...
        'filled', 'MarkerFaceAlpha', 0.35, ...
        'DisplayName', results(i).bagName);
end

xline(0, 'k-');
yline(0, 'k-');
axis equal;
grid on;
xlabel('x error, EKF - OptiTrack [m]');
ylabel('y error, EKF - OptiTrack [m]');
title('XY Error Scatter');
legend('Location', 'bestoutside', 'Interpreter', 'none');

savePlotFigure(fig, outputDir, 'XY_Error_Scatter', showPlot);
end
