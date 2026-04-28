function savePlotFigure(fig, outputDir, fileStem, showPlot)
%SAVEPLOTFIGURE Save one figure as PNG and close it when showPlot is false.

if nargin < 4 || isempty(showPlot)
    showPlot = true;
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

set(fig, 'Color', 'w');
set(fig, 'InvertHardcopy', 'off');
set(fig, 'Units', 'pixels');
axesHandles = findall(fig, 'Type', 'axes');
for i = 1:numel(axesHandles)
    if isprop(axesHandles(i), 'Toolbar') && ~isempty(axesHandles(i).Toolbar)
        axesHandles(i).Toolbar.Visible = 'off';
    end
end
pos = get(fig, 'Position');
if pos(3) < 1600 || pos(4) < 900
    set(fig, 'Position', [pos(1), pos(2), 1800, 1000]);
end

pngPath = fullfile(outputDir, [sanitizeFileName(fileStem), '.png']);
drawnow;
try
    exportgraphics(fig, pngPath, 'Resolution', 450);
catch
    print(fig, pngPath, '-dpng', '-r450');
end

if ~showPlot && isgraphics(fig, 'figure')
    close(fig);
end
end
