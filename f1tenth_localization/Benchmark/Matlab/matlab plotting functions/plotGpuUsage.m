function plotGpuUsage(data, outputDir, showPlot)
%PLOTGPUUSAGE Figure 6: GPU usage over time.

fig = makePlotFigure('GPU Usage', showPlot);
if data.gpu.hasFile || data.gpu.hasShort
    hold on;
    handles = gobjects(0);
    labels = cell(0, 1);
    gpuUpdateHz = NaN;

    if data.gpu.hasFile
        gpuRaw = double(data.gpu.standalone);
        gpuRaw(~isfinite(gpuRaw) | gpuRaw < 0 | gpuRaw > 100) = NaN;
        gpuRawPlot = maskIsolatedGpuDropouts(gpuRaw, 4.0, 20.0, 35.0, 6);

        if numel(data.gpu.t) >= 2
            dtRaw = median(diff(data.gpu.t));
            if isfinite(dtRaw) && dtRaw > 0
                gpuUpdateHz = 1.0 / dtRaw;
            end
        end

        hGpuRaw = plot(data.gpu.t, gpuRawPlot, '-', ...
            'Color', [0.80 0.86 0.80], 'LineWidth', 0.9);
        handles(end + 1) = hGpuRaw; %#ok<AGROW>
        labels{end + 1} = 'GPU monitor (raw, filtered)'; %#ok<AGROW>

        if numel(data.gpu.t) >= 3
            dtGpu = median(diff(data.gpu.t));
            if ~isfinite(dtGpu) || dtGpu <= 0
                dtGpu = 0.2;
            end
            smoothWinGpu = max(5, round(1.0 / dtGpu));
            if mod(smoothWinGpu, 2) == 0
                smoothWinGpu = smoothWinGpu + 1;
            end
            gpuTrend = movmedian(gpuRawPlot, smoothWinGpu, 'omitnan');
        else
            gpuTrend = gpuRawPlot;
        end

        hGpuTrend = plot(data.gpu.t, gpuTrend, '-', ...
            'Color', [0.15 0.55 0.20], 'LineWidth', 2.0);
        handles(end + 1) = hGpuTrend; %#ok<AGROW>
        labels{end + 1} = 'GPU monitor (1s trend)'; %#ok<AGROW>
    end

    if data.gpu.hasShort && ~data.gpu.hasFile && ~data.gpu.hasLong
        hGpuShort = plot(data.cpu.tShort, data.gpu.short, ':', ...
            'Color', [0.64 0.08 0.18], 'LineWidth', 1.5);
        handles(end + 1) = hGpuShort; %#ok<AGROW>
        labels{end + 1} = 'GPU from short window'; %#ok<AGROW>
    end

    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('GPU [%]');
    if isfinite(gpuUpdateHz)
        title(sprintf('GPU usage over time (monitor ~%.1f Hz)', gpuUpdateHz));
    else
        title('GPU usage over time');
    end
    ylim([0, 100]);
    if ~isempty(handles)
        legend(handles, labels, 'Location', 'northeast', 'FontSize', 12);
    end
else
    axis off;
    text(0.1, 0.5, 'GPU CSVs were not found', 'FontSize', 12);
end

savePlotFigure(fig, outputDir, 'GPU_Usage', showPlot);
end
