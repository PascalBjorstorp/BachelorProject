function mapData = getFirstMapData(results)
%GETFIRSTMAPDATA Return the first non-empty mapData field from a result array.

mapData = [];
for i = 1:numel(results)
    if isfield(results(i), 'mapData') && ~isempty(results(i).mapData)
        mapData = results(i).mapData;
        return;
    end
end
end
