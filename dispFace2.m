function dispFace2(shp1, tl1, shp2, tl2)
    FV1.vertices = shp1;
    FV1.faces = tl1;
    
    FV2.vertices = shp2;
    FV2.faces = tl2;
    
    patch(FV1, 'facecolor', [0.8 0.0 0.0], 'edgecolor', 'none', 'vertexnormalsmode', 'auto', 'FaceAlpha', 0.5);
    patch(FV2, 'facecolor', [0.0 0.8 0.0], 'edgecolor', 'none', 'vertexnormalsmode', 'auto', 'FaceAlpha', 0.5);
    
    camlight('headlight');
    lighting phong;
    material dull;
    axis vis3d
    axis equal;
end