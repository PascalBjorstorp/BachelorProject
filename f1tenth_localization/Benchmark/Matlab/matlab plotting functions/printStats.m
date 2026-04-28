function printStats(name, data, unit)
%PRINTSTATS Print mean/std/p95/p99 summary for one metric.

if nargin < 3 || isempty(unit)
    unit = 'ms';
end

data = double(data);
data = data(isfinite(data));
if isempty(data)
    fprintf('%-24s : no data\n', name);
    return;
end

fprintf('%-24s : mean=%8.3f %s   std=%8.3f   p95=%8.3f   p99=%8.3f\n', ...
    name, mean(data), unit, std(data), prctile(data, 95), prctile(data, 99));
end
