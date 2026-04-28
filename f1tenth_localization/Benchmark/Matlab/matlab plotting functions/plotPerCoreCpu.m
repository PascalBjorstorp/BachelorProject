function plotPerCoreCpu(data, outputDir, showPlot)
%PLOTPERCORECPU Figure 5: per-core CPU usage heatmap.

fig = makePlotFigure('Per-Core CPU', showPlot);
if data.perCore.hasData
    nCores = size(data.perCore.cpu, 2);
    imagesc(data.perCore.t, 1:nCores, data.perCore.cpu');
    axis xy;
    colormap(parula);
    caxis([0, 100]);
    cb = colorbar;
    cb.Label.String = 'CPU [%]';
    xlabel('Time [s]');
    ylabel('Core index');
    title('Per-core CPU over time');
    if nCores <= 16
        yticks(1:nCores);
    else
        yticks(1:2:nCores);
    end
    set(gca, 'FontSize', 11, 'LineWidth', 0.9);
else
    axis off;
    text(0.1, 0.5, 'Per-core CPU CSV was not found', 'FontSize', 12);
end

savePlotFigure(fig, outputDir, 'Per-Core_CPU', showPlot);
end
