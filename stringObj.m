function stringObj(string_set, v, name)
% VERTFACE2OBJ Save a set of vertice coordinates and faces as a Wavefront/Alias Obj file
% VERTFACE2OBJ(v,f,fname)
%     v is a Nx3 matrix of vertex coordinates.
%     f is a Mx3 matrix of vertex indices. 
%     fname is the filename to save the obj file.

fid = fopen(name,'w');

for i =1:size(string_set,2)
    if (i > 6 && i < 6897)
        fprintf(fid,'v %f %f %f \n',v(i-6,1),v(i-6,2),v(i-6,3));
    else
        fprintf(fid, '%s\n', string_set{i});
    end
end

fclose(fid);