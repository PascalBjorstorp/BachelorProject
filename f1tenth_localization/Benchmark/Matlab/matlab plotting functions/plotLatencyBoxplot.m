function plotLatencyBoxplot(data, outputDir, showPlot)
%PLOTLATENCYBOXPLOT Figure 3: latency distribution by stage.

fig = makePlotFigure('Latency Boxplot', showPlot);
if ~data.pipeline.hasData
    axis off;
    text(0.1, 0.5, 'Pipeline CSV was not found', 'FontSize', 12);
    savePlotFigure(fig, outputDir, 'Latency_Boxplot', showPlot);
    return;
end

p = data.pipeline;
b1 = clipForBoxplot(p.scanStamp2Scan(isfinite(p.scanStamp2Scan) & p.scanStamp2Scan >= 0));
b2 = clipForBoxplot(p.scan2amcl(isfinite(p.scan2amcl) & p.scan2amcl >= 0));
b3 = clipForBoxplot(p.amcl2ekf(isfinite(p.amcl2ekf) & p.amcl2ekf >= 0));
b4 = clipForBoxplot(p.scan2ekf(isfinite(p.scan2ekf) & p.scan2ekf >= 0));
b5 = clipForBoxplot(p.e2d(isfinite(p.e2d) & p.e2d >= 0));
b6 = clipForBoxplot(p.d2a(isfinite(p.d2a) & p.d2a >= 0));
b7 = clipForBoxplot(p.s2a(isfinite(p.s2a) & p.s2a >= 0));

allVals = [b1; b2; b3; b4; b5; b6; b7];
grp = [
    repmat({'scan_stamp->scan_rx'}, numel(b1), 1);
    repmat({p.scan2amclLabel}, numel(b2), 1);
    repmat({'amcl->ekf'}, numel(b3), 1);
    repmat({'scan->ekf'}, numel(b4), 1);
    repmat({'ekf->drive'}, numel(b5), 1);
    repmat({'drive->ackermann'}, numel(b6), 1);
    repmat({'scan->ackermann'}, numel(b7), 1)];

if isempty(allVals)
    axis off;
    text(0.1, 0.5, 'No data available for boxplot', 'FontSize', 12);
else
    boxplot(allVals, grp, 'Whisker', 1.5, 'Symbol', '');
    ylabel('Latency [ms]');
    title('Latency Distribution by Stage');
    grid on;
    xtickangle(20);
    ylim([0, max(5, prctile(allVals, 99.5) * 1.2)]);
end

savePlotFigure(fig, outputDir, 'Latency_Boxplot', showPlot);
end
