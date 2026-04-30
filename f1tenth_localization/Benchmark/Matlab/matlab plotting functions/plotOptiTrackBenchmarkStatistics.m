function plotOptiTrackBenchmarkStatistics(results, outputDir, showPlot)
%PLOTOPTITRACKBENCHMARKSTATISTICS Plot per-bag error statistics.

fig = makePlotFigure('OptiTrack Benchmark Statistics', showPlot);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

labels = string({results.bagName});
idx = 1:numel(results);

nexttile;
bar(idx, [results.meanXError]);
hold on;
errorbar(idx, [results.meanXError], [results.stdXError], 'k.', 'LineWidth', 1.0);
grid on;
ylabel('x error [m]');
title('Mean x Error with Std');
set(gca, 'XTick', idx, 'XTickLabel', labels, 'TickLabelInterpreter', 'none');
xtickangle(25);

nexttile;
bar(idx, [results.meanYError]);
hold on;
errorbar(idx, [results.meanYError], [results.stdYError], 'k.', 'LineWidth', 1.0);
grid on;
ylabel('y error [m]');
title('Mean y Error with Std');
set(gca, 'XTick', idx, 'XTickLabel', labels, 'TickLabelInterpreter', 'none');
xtickangle(25);

nexttile;
bar(idx, [results.rmseXYError]);
grid on;
ylabel('XY RMSE [m]');
title('XY RMSE per Bag');
set(gca, 'XTick', idx, 'XTickLabel', labels, 'TickLabelInterpreter', 'none');
xtickangle(25);

nexttile;
bar(idx, rad2deg([results.meanAbsYawError]));
grid on;
ylabel('mean |yaw error| [deg]');
title('Yaw Error per Bag');
set(gca, 'XTick', idx, 'XTickLabel', labels, 'TickLabelInterpreter', 'none');
xtickangle(25);

savePlotFigure(fig, outputDir, 'OptiTrack_Error_Statistics', showPlot);
end
