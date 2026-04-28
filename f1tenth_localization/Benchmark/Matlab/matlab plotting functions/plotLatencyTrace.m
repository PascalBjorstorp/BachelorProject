function plotLatencyTrace(t, y, titleText, color)
%PLOTLATENCYTRACE Plot rolling median latency with a moving spread band.

y = double(y);
valid = isfinite(y) & y >= 0;
t = t(valid);
y = y(valid);

if isempty(y)
    axis off;
    text(0.1, 0.5, ['No data for ' titleText], 'FontSize', 11);
    return;
end

win = max(11, floor(numel(y) * 0.01));
if mod(win, 2) == 0
    win = win + 1;
end

yTrend = movmedian(y, win);
ySpread = movstd(y, win, 0, 'omitnan');
yLo = max(0, yTrend - ySpread);
yHi = yTrend + ySpread;

fill([t; flipud(t)], [yLo; flipud(yHi)], color, ...
    'FaceAlpha', 0.18, 'EdgeColor', 'none');
hold on;
plot(t, yTrend, '-', 'Color', color, 'LineWidth', 1.6);
hold off;

yl = prctile(y, [1 99]);
yTop = max(yl(2) * 1.15, yl(1) + 1.0);
yBot = max(0, yl(1) - 0.2);
ylim([yBot, yTop]);
grid on;
xlabel('Time [s]', 'FontSize', 10);
ylabel('Latency [ms]', 'FontSize', 10);
title(titleText, 'FontSize', 11);
set(gca, 'FontSize', 9, 'LineWidth', 0.8);
end
