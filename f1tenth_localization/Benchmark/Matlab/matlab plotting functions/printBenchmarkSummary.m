function printBenchmarkSummary(data)
%PRINTBENCHMARKSUMMARY Print text summaries for loaded benchmark data.

if data.pipeline.hasData
    fprintf('\nLatency summary\n');
    if data.pipeline.hasScanStamp2Scan
        printStats('scan_stamp->scan', data.pipeline.scanStamp2Scan, 'ms');
    end
    printStats(data.pipeline.scan2amclLabel, data.pipeline.scan2amcl, 'ms');
    printStats('amcl->ekf', data.pipeline.amcl2ekf, 'ms');
    printStats('scan->ekf', data.pipeline.scan2ekf, 'ms');
    if data.pipeline.hasE2D
        printStats('ekf->drive', data.pipeline.e2d, 'ms');
    end
    if data.pipeline.hasD2A
        printStats('drive->ackermann', data.pipeline.d2a, 'ms');
    end
    if data.pipeline.hasS2A
        printStats('scan->ackermann', data.pipeline.s2a, 'ms');
    end
end

if data.amcl.hasTimingParticles
    fprintf('\nAMCL timing and particle summary\n');
    fprintf('Pairing method      : %s\n', data.amcl.pairMethod);
    printStats('/amcl_timing', data.amcl.timingMs, 'ms');
    printStats('/amcl_particle_count', data.amcl.particleCount, 'particles');
end

if data.cpu.hasLong || data.cpu.hasShort
    fprintf('\nCPU window summary\n');
    if data.cpu.hasLong
        printStats('cpu_long_window', data.cpu.long, '%');
    end
    if data.cpu.hasShort
        printStats('cpu_short_window', data.cpu.short, '%');
    end
end

if data.gpu.hasFile || data.gpu.hasLong || data.gpu.hasShort
    fprintf('\nGPU summary\n');
    if data.gpu.hasFile
        printStats('gpu_monitor', data.gpu.standalone, '%');
    end
    if data.gpu.hasLong
        printStats('gpu_long_window', data.gpu.long, '%');
    end
    if data.gpu.hasShort
        printStats('gpu_short_window', data.gpu.short, '%');
    end
end

if data.perCore.hasData
    coreMean = mean(data.perCore.cpu, 1, 'omitnan');
    coreP95 = prctile(data.perCore.cpu, 95, 1);
    [~, coreOrder] = sort(coreMean, 'descend');
    topK = min(8, numel(coreOrder));
    fprintf('\nPer-core summary (top %d by mean usage)\n', topK);
    for i = 1:topK
        c = coreOrder(i);
        fprintf('%-18s : mean=%8.3f %%   p95=%8.3f %%\n', ...
            data.perCore.names{c}, coreMean(c), coreP95(c));
    end
end

if data.node.hasData
    Tnode = data.node.table;
    nodeNames = string(Tnode.node_name);
    nodeCpu = double(Tnode.cpu_percent);
    nodePid = double(Tnode.pid);
    validNode = isfinite(nodeCpu) & nodeCpu >= 0 & strlength(nodeNames) > 0;
    nodeNames = nodeNames(validNode);
    nodeCpu = nodeCpu(validNode);
    nodePid = nodePid(validNode);

    if ~isempty(nodeCpu)
        [uniqueNodes, ~, nodeIdx] = unique(nodeNames, 'stable');
        nodeMean = accumarray(nodeIdx, nodeCpu, [], @mean, NaN);
        [~, sortIdx] = sort(nodeMean, 'descend');
        topK = min(8, numel(uniqueNodes));
        topNodes = uniqueNodes(sortIdx(1:topK));

        fprintf('\nPer-node CPU summary (top %d by mean)\n', topK);
        for i = 1:topK
            idx = (nodeNames == topNodes(i));
            vals = nodeCpu(idx);
            vals = vals(isfinite(vals));
            if isempty(vals)
                continue;
            end
            pids = unique(nodePid(idx));
            fprintf('%-28s : mean=%8.3f %%   p95=%8.3f %%   p99=%8.3f %%   pids=%d\n', ...
                char(topNodes(i)), mean(vals), prctile(vals, 95), prctile(vals, 99), numel(pids));
        end
    end
end
end
