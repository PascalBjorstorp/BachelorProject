function didDraw = drawOccupancyMapBackground(mapData)
%DRAWOCCUPANCYMAPBACKGROUND Draw an occupancy-grid RGB background.

didDraw = false;
if nargin < 1 || isempty(mapData)
    return;
end
if ~isfield(mapData, 'rgb') || ~isfield(mapData, 'xWorld') || ~isfield(mapData, 'yWorld')
    return;
end

h = image('XData', mapData.xWorld, 'YData', mapData.yWorld, 'CData', mapData.rgb, ...
    'HandleVisibility', 'off');
set(gca, 'YDir', 'normal');
try
    uistack(h, 'bottom');
catch
end
didDraw = true;
end
