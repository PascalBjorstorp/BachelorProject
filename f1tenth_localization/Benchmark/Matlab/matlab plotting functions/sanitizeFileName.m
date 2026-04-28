function nameOut = sanitizeFileName(nameIn)
%SANITIZEFILENAME Convert arbitrary text to a simple file-name stem.

nameOut = regexprep(char(nameIn), '[^a-zA-Z0-9_-]+', '_');
nameOut = regexprep(nameOut, '_+', '_');
nameOut = regexprep(nameOut, '^_+', '');
nameOut = regexprep(nameOut, '_+$', '');
if isempty(nameOut)
    nameOut = 'figure';
end
end
