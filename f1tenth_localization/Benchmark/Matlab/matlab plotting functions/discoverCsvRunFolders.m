function runs = discoverCsvRunFolders(csvRootDir)
%DISCOVERCSVRUNFOLDERS Find CSV run folders below one root.

runs = struct('name', {}, 'path', {}, 'datenum', {});

if ~exist(csvRootDir, 'dir')
    return;
end

if hasCsvFiles(csvRootDir)
    [~, rootName] = fileparts(csvRootDir);
    runs(end + 1) = struct('name', rootName, 'path', csvRootDir, ...
        'datenum', newestCsvDatenum(csvRootDir)); %#ok<AGROW>
end

entries = dir(csvRootDir);
for i = 1:numel(entries)
    if ~entries(i).isdir || startsWith(entries(i).name, '.')
        continue;
    end

    runPath = fullfile(csvRootDir, entries(i).name);
    if hasCsvFiles(runPath)
        runs(end + 1) = struct('name', entries(i).name, 'path', runPath, ...
            'datenum', newestCsvDatenum(runPath)); %#ok<AGROW>
    end
end

runs = sortStructByDatenumDesc(runs);
end

function tf = hasCsvFiles(dirPath)
tf = ~isempty(dir(fullfile(dirPath, '*.csv')));
end

function dn = newestCsvDatenum(dirPath)
matches = dir(fullfile(dirPath, '*.csv'));
if isempty(matches)
    dn = 0;
else
    dn = max([matches.datenum]);
end
end

function out = sortStructByDatenumDesc(in)
out = in;
if isempty(out)
    return;
end
[~, idx] = sort([out.datenum], 'descend');
out = out(idx);
end
