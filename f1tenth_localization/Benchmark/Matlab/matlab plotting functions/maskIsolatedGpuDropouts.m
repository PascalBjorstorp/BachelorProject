function yOut = maskIsolatedGpuDropouts(yIn, lowThreshold, neighborMin, jumpThreshold, maxDropRun)
%MASKISOLATEDGPUDROPOUTS Suppress isolated GPU monitor glitches.

yOut = double(yIn(:));
n = numel(yOut);
if n < 3
    return;
end

if nargin < 5 || isempty(maxDropRun)
    maxDropRun = 3;
end

for i = 2:(n - 1)
    yPrev = yOut(i - 1);
    yCur = yOut(i);
    yNext = yOut(i + 1);

    if ~isfinite(yPrev) || ~isfinite(yCur) || ~isfinite(yNext)
        continue;
    end

    if yCur <= lowThreshold && yPrev >= neighborMin && yNext >= neighborMin
        yOut(i) = NaN;
        continue;
    end

    jumpIn = abs(yCur - yPrev);
    jumpOut = abs(yCur - yNext);
    neighborDelta = abs(yNext - yPrev);
    if jumpIn >= jumpThreshold && jumpOut >= jumpThreshold && neighborDelta <= 12.0
        yOut(i) = NaN;
    end
end

isLow = isfinite(yOut) & (yOut <= lowThreshold);
edges = diff([false; isLow; false]);
runStarts = find(edges == 1);
runEnds = find(edges == -1) - 1;

for k = 1:numel(runStarts)
    s = runStarts(k);
    e = runEnds(k);
    runLen = e - s + 1;
    if runLen > maxDropRun || s <= 1 || e >= n
        continue;
    end

    yPrev = yOut(s - 1);
    yNext = yOut(e + 1);
    if isfinite(yPrev) && isfinite(yNext) && yPrev >= neighborMin && yNext >= neighborMin
        yOut(s:e) = NaN;
    end
end
end
