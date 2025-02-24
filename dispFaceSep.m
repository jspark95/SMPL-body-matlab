function dispFaceSep (shp, tl, color1, color2, mask)
% Note, shp has nx3 elements and tl has mx3 elements
% Note, If saveMesh activated, mesh is saved as obj file format

    nvert = size(shp,1);
    color = repmat(color1', [1, nvert] ); % (nvert x 3)
    color = color';
    for i = 1:length(mask)
        color(mask(i), :) = color2;
    end

    alpha = 1;
	
    FV.vertices = shp;
    FV.faces = tl;
    
    patch(FV,  'facecolor', 'interp', 'FaceVertexCData', color, 'edgecolor', 'none', 'vertexnormalsmode', 'auto', 'FaceAlpha', alpha);

    
    camlight('headlight');
%     camlight('right')
    lighting phong;
    material dull;
    axis vis3d
    axis equal;
end