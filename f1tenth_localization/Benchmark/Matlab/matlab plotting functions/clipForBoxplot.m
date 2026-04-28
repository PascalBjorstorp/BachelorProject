function yOut = clipForBoxplot(yIn)
%CLIPFORBOXPLOT Remove extreme tails for readable boxplots.

yIn = double(yIn(:));
yIn = yIn(isfinite(yIn));
if isempty(yIn)
    yOut = yIn;
    return;
end

lo = prctile(yIn, 0.5);
hi = prctile(yIn, 99.5);
yOut = yIn(yIn >= lo & yIn <= hi);
end
