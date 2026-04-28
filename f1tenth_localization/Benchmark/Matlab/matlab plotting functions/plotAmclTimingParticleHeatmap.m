function plotAmclTimingParticleHeatmap(data, outputDir, showPlot)
%PLOTAMCLTIMINGPARTICLEHEATMAP AMCL stage timing colored by particle count.

fig = makePlotFigure('AMCL Stage Timing vs Particle Count', showPlot);
if data.amcl.hasTimingParticles
    x = data.amcl.sampleIndex(:);
    y = data.amcl.timingMs(:);
    particleCount = data.amcl.particleCount(:);
    valid = isfinite(x) & isfinite(y) & isfinite(particleCount);

    hold on;
    if nnz(valid) >= 2
        xLine = x;
        yLine = y;
        particleLine = particleCount;
        xLine(~valid) = NaN;
        yLine(~valid) = NaN;
        particleLine(~valid) = NaN;

        surface([xLine.'; xLine.'], [yLine.'; yLine.'], zeros(2, numel(xLine)), ...
            [particleLine.'; particleLine.'], ...
            'FaceColor', 'none', ...
            'EdgeColor', 'interp', ...
            'LineWidth', 1.8);
    end
    scatter(x(valid), y(valid), 12, particleCount(valid), 'filled', ...
        'MarkerFaceAlpha', 0.85, 'MarkerEdgeAlpha', 0.08);
    hold off;
    grid on;
    xlabel('AMCL output sample index');
    ylabel('scan->amcl [ms]');
    title(sprintf('scan->amcl colored by /amcl\\_particle\\_count (%s)', ...
        data.amcl.pairMethod), 'Interpreter', 'tex');
    cb = colorbar;
    cb.Label.String = '/amcl\_particle\_count';
    cb.Label.Interpreter = 'tex';
    if exist('turbo', 'file') == 2
        colormap(turbo);
    else
        colormap(parula);
    end
    validTiming = data.amcl.timingMs(isfinite(data.amcl.timingMs) & data.amcl.timingMs >= 0);
    if ~isempty(validTiming)
        ylim([0, max(1.0, prctile(validTiming, 99.5) * 1.15)]);
    end
    set(gca, 'FontSize', 11, 'LineWidth', 0.9);
else
    axis off;
    text(0.08, 0.55, 'No paired scan->amcl timing and /amcl\_particle\_count CSV data found', ...
        'FontSize', 12, 'Interpreter', 'tex');
    text(0.08, 0.45, sprintf('Topic CSV folder: %s', data.topicCsvDir), ...
        'FontSize', 10, 'Interpreter', 'none');
end

savePlotFigure(fig, outputDir, 'AMCL_Timing_vs_Particle_Count', showPlot);
end
