function plotOptiTrackYawErrorOverTime(results, outputDir, showPlot)
%PLOTOPTITRACKYAWERROROVERTIME Plot yaw error over elapsed time.

fig = makePlotFigure('OptiTrack Yaw Error Over Time', showPlot);
hold on;
colors = lines(max(numel(results), 1));

for i = 1:numel(results)
    plot(results(i).tRel, rad2deg(results(i).yawError), ...
        'Color', colors(i, :), 'LineWidth', 1.0, ...
        'DisplayName', results(i).bagName);
end

yline(0, 'k-');
grid on;
xlabel('time after analysis window start [s]');
ylabel('yaw error, EKF - OptiTrack [deg]');
title('Yaw Error Over Time');
legend('Location', 'bestoutside', 'Interpreter', 'none');

savePlotFigure(fig, outputDir, 'Yaw_Error_Over_Time', showPlot);
end
