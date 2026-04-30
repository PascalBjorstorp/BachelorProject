function plotOptiTrackTrajectoryComparison(results, outputDir, showPlot)
%PLOTOPTITRACKTRAJECTORYCOMPARISON Plot OptiTrack and EKF XY trajectories.

fig = makePlotFigure('OptiTrack vs EKF Trajectory', showPlot);
hold on;
colors = lines(max(numel(results), 1));

for i = 1:numel(results)
    c = colors(i, :);
    plot(results(i).gtPos(:, 1), results(i).gtPos(:, 2), '-', ...
        'Color', c, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('%s OptiTrack', results(i).bagName));
    plot(results(i).ekfPos(:, 1), results(i).ekfPos(:, 2), '--', ...
        'Color', c, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('%s EKF', results(i).bagName));
end

axis equal;
grid on;
xlabel('map x [m]');
ylabel('map y [m]');
title('OptiTrack Ground Truth vs EKF Trajectory');
legend('Location', 'bestoutside', 'Interpreter', 'none');

savePlotFigure(fig, outputDir, 'OptiTrack_Trajectory_vs_EKF', showPlot);
end
