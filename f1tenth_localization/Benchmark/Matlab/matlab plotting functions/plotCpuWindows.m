function plotCpuWindows(data, outputDir, showPlot)
%PLOTCPUWINDOWS Figure 4: CPU windows over time.

fig = makePlotFigure('CPU Windows', showPlot);
if data.cpu.hasLong || data.cpu.hasShort
    hold on;
    handles = gobjects(0);
    labels = cell(0, 1);
    if data.cpu.hasShort
        hShort = plot(data.cpu.tShort, data.cpu.short, '-', ...
            'Color', [0.75 0.78 0.82], 'LineWidth', 0.9);
        handles(end + 1) = hShort; %#ok<AGROW>
        labels{end + 1} = 'CPU short window'; %#ok<AGROW>
    end
    if data.cpu.hasLong
        hLong = stairs(data.cpu.tLong, data.cpu.long, '-', ...
            'Color', [0.05 0.05 0.05], 'LineWidth', 2.4);
        handles(end + 1) = hLong; %#ok<AGROW>
        labels{end + 1} = 'CPU long window'; %#ok<AGROW>
    end
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('CPU [%]');
    title('CPU windows over time');
    ylim([0, 100]);
    legend(handles, labels, 'Location', 'northeast', 'FontSize', 12);
else
    axis off;
    text(0.1, 0.5, 'CPU window CSVs were not found', 'FontSize', 12);
end

savePlotFigure(fig, outputDir, 'CPU_Windows', showPlot);
end
