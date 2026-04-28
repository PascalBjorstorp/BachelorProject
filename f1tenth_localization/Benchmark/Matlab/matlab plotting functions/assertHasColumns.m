function assertHasColumns(T, required, tableLabel)
%ASSERTHASCOLUMNS Error if a table is missing required columns.

for i = 1:numel(required)
    if ~ismember(required{i}, T.Properties.VariableNames)
        error('%s is missing required column: %s', tableLabel, required{i});
    end
end
end
