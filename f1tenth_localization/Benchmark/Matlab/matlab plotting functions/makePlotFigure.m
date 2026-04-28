function fig = makePlotFigure(figName, showPlot)
%MAKEPLOTFIGURE Create a white figure, optionally hidden.

if nargin < 2 || isempty(showPlot)
    showPlot = true;
end

if showPlot
    visibility = 'on';
else
    visibility = 'off';
end

fig = figure('Name', figName, 'Color', 'w', 'Visible', visibility);
end
