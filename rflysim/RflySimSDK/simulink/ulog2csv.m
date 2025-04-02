function ulog2csv(fileName)

if exist([pwd,'/',fileName],'file')
    fileName=[pwd,'/',fileName];
end

if ~ exist(fileName,'file')
    disp('Error: Failed to find file ',fileName);
    return
end

if ~ endsWith(fileName,'.ulg')
    disp('Error: Wrong file format, *.ulg file is required.');
    return
end

system(['%PSP_PATH%\Python38\Scripts\ulog2csv.exe ',fileName])