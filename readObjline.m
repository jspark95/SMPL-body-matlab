function string_set = readObjline(fname)

fid = fopen(fname);
i = 1;

% parse .obj file 
while 1    
    tline = fgetl(fid);
    string_set{i} =tline;
    i = i+1;
    if ~ischar(tline),   break,   end  % exit at end of file 
end
fclose(fid);