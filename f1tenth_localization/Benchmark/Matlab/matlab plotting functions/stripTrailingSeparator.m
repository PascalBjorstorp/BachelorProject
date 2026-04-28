function outPath = stripTrailingSeparator(inPath)
%STRIPTRAILINGSEPARATOR Remove trailing file separators from a path string.

outPath = char(inPath);
while numel(outPath) > 1 && (outPath(end) == '/' || outPath(end) == '\')
    outPath(end) = [];
end
end
