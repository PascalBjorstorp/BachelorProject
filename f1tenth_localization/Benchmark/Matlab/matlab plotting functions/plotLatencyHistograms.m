function plotLatencyHistograms(data, outputDir, showPlot)
%PLOTLATENCYHISTOGRAMS Figure 2: latency histograms.

fig = makePlotFigure('Latency Histograms', showPlot);
if ~data.pipeline.hasData
    axis off;
    text(0.1, 0.5, 'Pipeline CSV was not found', 'FontSize', 12);
    savePlotFigure(fig, outputDir, 'Latency_Histograms', showPlot);
    return;
end

p = data.pipeline;
scan2ekfValid = p.scan2ekf(isfinite(p.scan2ekf) & p.scan2ekf >= 0);
if p.hasS2A
    s2aValid = p.s2a(isfinite(p.s2a) & p.s2a >= 0);
else
    s2aValid = [];
end

tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
histogram(scan2ekfValid, 80);
grid on;
title('Histogram: scan->ekf', 'FontSize', 13);
xlabel('scan->ekf [ms]', 'FontSize', 12);
ylabel('Count', 'FontSize', 12);
set(gca, 'FontSize', 11, 'LineWidth', 0.9);
if ~isempty(scan2ekfValid)
    xlim([0, max(5, prctile(scan2ekfValid, 99.5))]);
end

nexttile;
if ~isempty(s2aValid)
    histogram(s2aValid, 80);
    grid on;
    title('Histogram: scan->ackermann', 'FontSize', 13);
    xlabel('scan->ackermann [ms]', 'FontSize', 12);
    ylabel('Count', 'FontSize', 12);
    set(gca, 'FontSize', 11, 'LineWidth', 0.9);
    xlim([0, max(5, prctile(s2aValid, 99.5))]);
else
    axis off;
    text(0.1, 0.5, 'No scan->ackermann data available', 'FontSize', 11);
end

savePlotFigure(fig, outputDir, 'Latency_Histograms', showPlot);
end
