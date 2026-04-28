function plotPerNodeCpu(data, outputDir, showPlot)
%PLOTPERNODECPU Figure 7: per-node CPU usage summary.

fig = makePlotFigure('Per-Node CPU', showPlot);
if ~data.node.hasData
    axis off;
    text(0.1, 0.5, 'Node-process CPU CSV was not found', 'FontSize', 12);
    savePlotFigure(fig, outputDir, 'Per-Node_CPU', showPlot);
    return;
end

Tnode = data.node.table;
nodeNames = string(Tnode.node_name);
nodeCpu = double(Tnode.cpu_percent);
validNode = isfinite(nodeCpu) & nodeCpu >= 0 & strlength(nodeNames) > 0;
nodeNames = nodeNames(validNode);
nodeCpu = nodeCpu(validNode);

if isempty(nodeCpu)
    axis off;
    text(0.1, 0.5, 'No per-node CPU samples available', 'FontSize', 11);
    savePlotFigure(fig, outputDir, 'Per-Node_CPU', showPlot);
    return;
end

[uniqueNodes, ~, nodeIdx] = unique(nodeNames, 'stable');
nodeMean = accumarray(nodeIdx, nodeCpu, [], @mean, NaN);
[~, sortIdx] = sort(nodeMean, 'descend');
topK = min(8, numel(uniqueNodes));
topNodes = uniqueNodes(sortIdx(1:topK));

nodeMeanTop = nan(topK, 1);
nodeP95Top = nan(topK, 1);
for i = 1:topK
    idx = (nodeNames == topNodes(i));
    vals = nodeCpu(idx);
    vals = vals(isfinite(vals));
    if isempty(vals)
        continue;
    end
    nodeMeanTop(i) = mean(vals);
    nodeP95Top(i) = prctile(vals, 95);
end

bar([nodeMeanTop, nodeP95Top], 'grouped');
grid on;
ylabel('CPU [%]');
xlabel('Node');
title('Per-node CPU summary (mean and P95)');
legend('Mean', 'P95', 'Location', 'northeast', 'FontSize', 12);
xticks(1:topK);
xticklabels(cellstr(topNodes));
xtickangle(20);
ylim([0, 100]);
set(gca, 'FontSize', 11, 'LineWidth', 0.9);

savePlotFigure(fig, outputDir, 'Per-Node_CPU', showPlot);
end
