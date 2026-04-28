function plotPipelineLatencyOverTime(data, outputDir, showPlot)
%PLOTPIPELINELATENCYOVERTIME Figure 1: pipeline stage latency over time.

fig = makePlotFigure('Pipeline Latency Over Time', showPlot);
if ~data.pipeline.hasData
    axis off;
    text(0.1, 0.5, 'Pipeline CSV was not found', 'FontSize', 12);
    savePlotFigure(fig, outputDir, 'Pipeline_Latency_Over_Time', showPlot);
    return;
end

p = data.pipeline;
tiledlayout(4, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
if p.hasScanStamp2Scan
    plotLatencyTrace(p.t, p.scanStamp2Scan, 'scan_stamp->scan_rx', [0.35 0.35 0.35]);
else
    axis off;
    text(0.1, 0.5, 'No scan_stamp->scan_rx column', 'FontSize', 11);
end

nexttile;
plotLatencyTrace(p.t, p.scan2amcl, p.scan2amclLabel, [0.0 0.45 0.74]);

nexttile;
plotLatencyTrace(p.t, p.amcl2ekf, 'amcl->ekf', [0.85 0.33 0.10]);

nexttile;
plotLatencyTrace(p.t, p.scan2ekf, 'scan->ekf', [0.93 0.69 0.13]);

nexttile;
if p.hasE2D
    plotLatencyTrace(p.t, p.e2d, 'ekf->drive', [0.49 0.18 0.56]);
else
    axis off;
    text(0.1, 0.5, 'No ekf->drive column', 'FontSize', 11);
end

nexttile;
if p.hasD2A
    plotLatencyTrace(p.t, p.d2a, 'drive->ackermann', [0.08 0.50 0.18]);
else
    axis off;
    text(0.1, 0.5, 'No drive->ackermann column', 'FontSize', 11);
end

nexttile;
if p.hasS2A
    plotLatencyTrace(p.t, p.s2a, 'scan->ackermann', [0.00 0.35 0.75]);
else
    axis off;
    text(0.1, 0.5, 'No scan->ackermann column', 'FontSize', 11);
end

nexttile;
axis off;

savePlotFigure(fig, outputDir, 'Pipeline_Latency_Over_Time', showPlot);
end
