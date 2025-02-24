clear;
addpath('./npy_matlab');

%% Arguments Setting
load('SMPL_M.mat'); % Male Core
load('m_model.mat'); % male template
% load('w_model.mat'); % female template
load('target.mat'); % target template
load('lm_idx.mat'); % s_lm, t_lm, 1~7 : similarity fitting, 8~24 : pose landmark
beta = zeros(10, 1);	% Shape param (10)
theta = zeros(24, 3);	% Pose param (1 Global + 23 Joints x 3)

%% Shape PCA
v_shaped = squeeze(sum(permute(shapedirs, [3 1 2]) .* beta)) + v_temp;

%% Joint Regressor
J = C.regJoint * v_shaped;   

%% Pose Regressor
nPosedir = size(posedirs, 3); % 23 x 9 = 207
dTheta = zeros(nPosedir, 1);
for i = 2:length(theta) % Except Global Transformation
    dTemp = rod(theta(i,:)) - eye(3);
    dTheta((i-2)*9+(1:9)) = dTemp(:);
end
v_posed = squeeze(sum(permute(posedirs, [3 1 2]) .* dTheta)) + v_shaped;

%% Linear Blend Skinning
% Get Rotation Matrix along the Joint
rot_J = zeros(4, 4, length(kintree)); % 4 x 4 x 24
rot_J(:, :, 1) = rod(theta(1, :), J(1,:));
for j = 2:length(kintree)
    childRot = rod(theta(j, :), (J(j, :) - J((kintree(1, j)+1), :)));
    rot_J(:,:,j) = rot_J(:, :, (kintree(1, j)+1)) * childRot;
end

rot_J_global = rot_J; % Global Rotation matrix of the Joint
for i = 1:length(kintree)
    rot_J(:, :, i) = rot_J(:, :, i) - [zeros(4,3) (rot_J(:,:,i) * [J(i,:) 0]')];
end

%% Make Skinned Vertices
nJoint = size(weights, 2); % 24
nVertex = size(weights, 1); % 6890

% rot_J [(4 x 4) x 24] * weights [(24 x 6890)']
T = reshape(reshape(rot_J, [16, nJoint]) * weights', [4, 4, nVertex]); % 4 x 4 x 6890

rest_shape_h = repmat([v_posed ones(length(v_posed),1)], 1, 1, 4); % 6890 x 4 x 4
v = squeeze(sum(permute(T, [3 2 1]) .* rest_shape_h, 2))';

v = v(1:3, :) ./ v(4, :); % Homogenous to Cartesian
v = v(1:3, :)'; % Final Src vertices set

Jtr = squeeze(rot_J_global(1:3, 4, :)); 

% Display Initial Src Model & Tar Model
h = figure;
dispFace2(v, f, points, tl);
saveas(h, '1.png');

%% Similarity Fitting (Initial pose matching)
[Tm, Ts, TR, Tt] = similarity_fitting(points, v, t_lm(1:7), s_lm(1:7)); % Tm : position normalize, Ts : scale. TR : Rotation matrix, Tt : translation vector
points = (TR * (points - Tm)' * Ts)' + Tt;

% Display Initlal Pose Matched Initial Src Model & Tar Model 
% h = figure('visible','off');
h = figure;
dispFace2(v, f, points, tl);
saveas(h, '2.png');


opts = optimoptions(@lsqnonlin, 'Display', 'iter');
Wreg = 0;

%% Joint Regressor
J = C.regJoint * v_shaped;   
%% Shape PCA
v_shaped_sampled = v_shaped(s_lm(8:24), :);

[theta, ~, ~, ~, ~] = lsqnonlin(@(theta)EfuncOptimThetaLand(C, v_shaped_sampled, J, theta, s_lm(8:24), points(t_lm(8:24), :), Wreg), theta, [], [], opts);
%%%%%%%%%
%%%%%%%%%%%
%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Shape PCA
v_shaped = squeeze(sum(permute(shapedirs, [3 1 2]) .* beta)) + v_temp;

%% Joint Regressor
J = reg_J * v_shaped;

%% Pose Regressor
nPosedir = size(posedirs, 3); % 23 x 9 = 207
dTheta = zeros(nPosedir, 1);
for i = 2:length(theta) % Except Global
    dTemp = rod(theta(i,:)) - eye(3);
    dTheta((i-2)*9+(1:9)) = dTemp(:);
end
v_posed = squeeze(sum(permute(posedirs, [3 1 2]) .* dTheta)) + v_shaped;

%% Linear Blend Skinning
% Get Rotation Matrix along the Joint
rot_J = zeros(4, 4, length(kintree)); % 4 x 4 x 24
rot_J(:, :, 1) = rod(theta(1, :), J(1,:));
for j = 2:length(kintree)
    childRot = rod(theta(j, :), (J(j, :) - J((kintree(1, j)+1), :)));
    rot_J(:,:,j) = rot_J(:, :, (kintree(1, j)+1)) * childRot;
end

rot_J_global = rot_J;
for i = 1:length(kintree)
    rot_J(:, :, i) = rot_J(:, :, i) - [zeros(4,3) (rot_J(:,:,i) * [J(i,:) 0]')];
end

%% Make Skinned Vertices
nJoint = size(weights, 2); % 24
nVertex = size(weights, 1); % 6890

% rot_J [(4 x 4) x 24] * weights [(24 x 6890)']
T = reshape(reshape(rot_J, [16, nJoint]) * weights', [4, 4, nVertex]); % 4 x 4 x 6890

rest_shape_h = repmat([v_posed ones(length(v_posed),1)], 1, 1, 4); % 6890 x 4 x 4
v = squeeze(sum(permute(T, [3 2 1]) .* rest_shape_h, 2))';

v = v(1:3, :) ./ v(4, :); % Homogenous to Cartesian
v = v(1:3, :)';

Jtr = squeeze(rot_J_global(1:3, 4, :)); 

h = figure;
dispFace2(v, f, points, tl);
saveas(h, '3.png');


%% Sampling 
nOfSrcSampling = 0.2;
nOfTarSampling = 0.2;

if (nOfSrcSampling <= 1.0)
    nOfSrcSampling = round(length(C.meanVerts) * nOfSrcSampling);
end
if (nOfTarSampling <= 1.0)
    nOfTarSampling = round(length(points) * nOfTarSampling );
end

rndSrcIdx =  randsample(length(C.meanVerts), nOfSrcSampling);
rndTarIdx = randsample(length(points), nOfTarSampling);

srcVertices = v(rndSrcIdx, :);
tarVertices = points(rndTarIdx, :);

%%%%%%%%%%%%%%%%%%%%%%%

Src = pointCloud(srcVertices);
Tar = pointCloud(tarVertices); % Tar to Src
Src.Normal = pcnormals(Src);

[Tform, ~] = pcregrigid(Src, Tar, 'Extrapolate', true, 'InlierRatio', 0.7, 'Metric', 'pointToPlane', 'MaxIterations', 50);
ptPoints = pctransform(pointCloud(points), invert(Tform));
points = ptPoints.Location;

tarVertices = points(rndTarIdx, :);


%[Tm, Ts, TR, Tt] = similarity_fitting(points, v, t_lm(1:7), s_lm(1:7));
%points = (TR * (points - Tm)' * Ts)' + Tt;

%h = figure('visible','off');
h = figure;
dispFace2(v, f, points, tl);
saveas(h, '4.png');


%% Make Block
minGrid = min(min(points), min(srcVertices));
maxGrid = max(max(points), max(srcVertices));

nGrid = ceil((maxGrid - minGrid) ./ 0.25);
xGrid = nGrid(1); yGrid = nGrid(2); zGrid = nGrid(3);

gridStep = (maxGrid - minGrid) ./ [xGrid yGrid zGrid];

cenGrid = ndgrid(minGrid(1)+gridStep(1)/2:gridStep(1):maxGrid(1), minGrid(2)+gridStep(2)/2:gridStep(2):maxGrid(2), minGrid(3)+gridStep(3)/2:gridStep(3):maxGrid(3));
gridGroup = cell(xGrid, yGrid, zGrid);

gridOverlap = 1.3;

% Grid Division
for x = 1:xGrid
    for y = 1:yGrid
        for z = 1:zGrid
            cenX = minGrid(1)-gridStep(1)/2 + gridStep(1)*x;
            cenY = minGrid(2)-gridStep(2)/2 + gridStep(2)*y;
            cenZ = minGrid(3)-gridStep(3)/2 + gridStep(3)*z;
            gridGroup{x, y, z} = [];
            cStep = 1.0;
            while (isempty(gridGroup{x, y, z}))
                cStep = cStep * gridOverlap;
                gridGroup{x, y, z} = 1:nOfTarSampling;
                gridGroup{x, y, z} = gridGroup{x, y, z}(all(abs(tarVertices(gridGroup{x, y, z}, :) - [cenX cenY cenZ]) <= (gridStep/2) * cStep, 2));
            end
        end
    end
end

%%

VselGrid = max(min(floor((srcVertices - minGrid) ./ gridStep) +1, [xGrid, yGrid, zGrid]), [1, 1, 1]);

Wreg = 0.0001;
Wland = 0.001;
[beta, ~, ~, ~, ~] = lsqnonlin(@(beta)EfuncOptimBeta(C, beta, theta, gridGroup, VselGrid, rndSrcIdx, s_lm, tarVertices, points(t_lm, :), Wland, Wreg), beta, [], [], opts);


%%%%%%%%%
%%%%%%%%%%%
%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Shape PCA
v_shaped = squeeze(sum(permute(shapedirs, [3 1 2]) .* beta)) + v_temp;

%% Joint Regressor
J = reg_J * v_shaped;

%% Pose Regressor
nPosedir = size(posedirs, 3); % 23 x 9 = 207
dTheta = zeros(nPosedir, 1);
for i = 2:length(theta) % Except Global
    dTemp = rod(theta(i,:)) - eye(3);
    dTheta((i-2)*9+(1:9)) = dTemp(:);
end
v_posed = squeeze(sum(permute(posedirs, [3 1 2]) .* dTheta)) + v_shaped;

%% Linear Blend Skinning
% Get Rotation Matrix along the Joint
rot_J = zeros(4, 4, length(kintree)); % 4 x 4 x 24
rot_J(:, :, 1) = rod(theta(1, :), J(1,:));
for j = 2:length(kintree)
    childRot = rod(theta(j, :), (J(j, :) - J((kintree(1, j)+1), :)));
    rot_J(:,:,j) = rot_J(:, :, (kintree(1, j)+1)) * childRot;
end

rot_J_global = rot_J;
for i = 1:length(kintree)
    rot_J(:, :, i) = rot_J(:, :, i) - [zeros(4,3) (rot_J(:,:,i) * [J(i,:) 0]')];
end

%% Make Skinned Vertices
nJoint = size(weights, 2); % 24
nVertex = size(weights, 1); % 6890

% rot_J [(4 x 4) x 24] * weights [(24 x 6890)']
T = reshape(reshape(rot_J, [16, nJoint]) * weights', [4, 4, nVertex]); % 4 x 4 x 6890

rest_shape_h = repmat([v_posed ones(length(v_posed),1)], 1, 1, 4); % 6890 x 4 x 4
v = squeeze(sum(permute(T, [3 2 1]) .* rest_shape_h, 2))';

v = v(1:3, :) ./ v(4, :); % Homogenous to Cartesian
v = v(1:3, :)';

Jtr = squeeze(rot_J_global(1:3, 4, :)); 

h = figure;
dispFace2(v, f, points, tl);
saveas(h, '5.png');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Tm, Ts, TR, Tt] = similarity_fitting(v_temp, points, s_lm(1:11), t_lm(1:11));

%figure, dispFace(v, f, [.8 .8 .8]);

function F = EfuncOptimThetaLand(Core, v_shaped_sampled, J, theta, srcLandIdx, tarLandVert, Wreg)
    A = genPosedShape(Core, v_shaped_sampled, J, theta, srcLandIdx);
    B = tarLandVert;
    
    E = zeros(16, 3);
    E(1, :) = GetRotErr(A(1, :)-A(2, :), B(1, :)-B(2, :));
    E(2, :) = GetRotErr(A(2, :)-A(3, :), B(2, :)-B(3, :));
    E(3, :) = GetRotErr(A(3, :)-A(4, :), B(3, :)-B(4, :));
    E(4, :) = GetRotErr(A(4, :)-A(5, :), B(4, :)-B(5, :));
    E(5, :) = GetRotErr(A(5, :)-A(12, :), B(5, :)-B(12, :));
    E(6, :) = GetRotErr(A(12, :)-A(13, :), B(12, :)-B(13, :));
    E(7, :) = GetRotErr(A(13, :)-A(14, :), B(13, :)-B(14, :));
    E(8, :) = GetRotErr(A(5, :)-A(15, :), B(5, :)-B(15, :));
    E(9, :) = GetRotErr(A(15, :)-A(16, :), B(15, :)-B(16, :));
    E(10, :) = GetRotErr(A(16, :)-A(17, :), B(16, :)-B(17, :));
    E(15, :) = GetRotErr(A(2, :)-A(6, :), B(2, :)-B(6, :));
    E(16, :) = GetRotErr(A(6, :)-A(7, :), B(6, :)-B(7, :));
    E(11, :) = GetRotErr(A(7, :)-A(8, :), B(7, :)-B(8, :));
    E(12, :) = GetRotErr(A(2, :)-A(9, :), B(2, :)-B(9, :));
    E(13, :) = GetRotErr(A(9, :)-A(10, :), B(9, :)-B(10, :));
    E(14, :) = GetRotErr(A(10, :)-A(11, :), B(10, :)-B(11, :));
    
    R = theta;
    F = [E(:); sqrt(Wreg) .* R(:)];
end

function F = EfuncOptimBeta(Core, beta, theta, gridGroup, VselGrid, srcICPIdx, srcLandIdx, tarVerts, tarLandVert, Wland, Wreg)
    %% Shape PCA
    v_shaped = squeeze(sum(permute(Core.shapeDirs, [3 1 2]) .* beta)) + Core.meanVerts;
    J = Core.regJoint * v_shaped;   

    % ICP Term
    VsrcICP = genPosedShape(Core, v_shaped(srcICPIdx, :), J, theta, srcICPIdx); % Vsrc_Trans
    
    nSrc = length(VsrcICP);
	E_ICP = zeros(nSrc, 3);
	for i = 1:nSrc
        Vgroup = tarVerts(gridGroup{VselGrid(i, 1), VselGrid(i, 2), VselGrid(i, 3)}, :);
		[~, mIdx] = min(sum((Vgroup - VsrcICP(i, :)).^2, 2));
        E_ICP(i, :) =  Vgroup(mIdx, :) - VsrcICP(i, :);
    end
    E_ICP = E_ICP ./ sqrt(nSrc);

    % Land Term
    E_land = genPosedShape(Core, v_shaped(srcLandIdx, :), J, theta, srcLandIdx) - tarLandVert;
    E_land = E_land ./ sqrt(length(E_land));

    % Regularization Term
    R = beta ./ sqrt(length(beta));
    F = [E_ICP(:); sqrt(Wland) .* E_land(:); sqrt(Wreg) .* R(:)];
end

function E = GetRotErr(D1, D2)
    E = (D1 ./ (norm(D1))) - (D2 ./ (norm(D2)));
end

function vertShape = genPosedShape(C, v_shaped_sampled, J_sampled, theta, vertIdx)
    %% Pose Regressor
    poseDirs = C.poseDirs(vertIdx, :, :);
    nPosedir = size(poseDirs, 3); % 23 x 9 = 207
    dTheta = zeros(nPosedir, 1);
    for i = 2:length(theta) % Except Global
        dTemp = rod(theta(i,:)) - eye(3);
        dTheta((i-2)*9+(1:9)) = dTemp(:);
    end
    v_posed = squeeze(sum(permute(poseDirs, [3 1 2]) .* dTheta)) + v_shaped_sampled;

    %% Linear Blend Skinning
    % Get Rotation Matrix along the Joint
    rot_J = zeros(4, 4, length(C.kinTree)); % 4 x 4 x 24
    rot_J(:, :, 1) = rod(theta(1, :), J_sampled(1,:));
    for j = 2:length(C.kinTree)
        childRot = rod(theta(j, :), (J_sampled(j, :) - J_sampled((C.kinTree(1, j)+1), :)));
        rot_J(:,:,j) = rot_J(:, :, (C.kinTree(1, j)+1)) * childRot;
    end

    %rot_J_global = rot_J;
    for i = 1:length(C.kinTree)
        rot_J(:, :, i) = rot_J(:, :, i) - [zeros(4,3) (rot_J(:,:,i) * [J_sampled(i,:) 0]')];
    end

    %% Make Skinned Vertices
    nJoint = size(C.blendWeights, 2); % 24
    nVertex = length(vertIdx); % 6890

    % rot_J [(4 x 4) x 24] * weights [(24 x 6890)']
    T = reshape(reshape(rot_J, [16, nJoint]) * C.blendWeights(vertIdx, :)', [4, 4, nVertex]); % 4 x 4 x 6890

    rest_shape_h = repmat([v_posed ones(length(v_posed),1)], 1, 1, 4); % 6890 x 4 x 4
    v = squeeze(sum(permute(T, [3 2 1]) .* rest_shape_h, 2))';

    v = v(1:3, :) ./ v(4, :); % Homogenous to Cartesian
    vertShape = v(1:3, :)';
    
    %Jtr = squeeze(rot_J_global(1:3, 4, :)); 
end

function R = rod(V, J)
    if all(V == 0)
        R = eye(3);
    else
        theta=sqrt(V(1)^2 + V(2)^2 + V(3)^2);
        omega=[0 -V(3) V(2); V(3) 0 -V(1); -V(2) V(1) 0];
        R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
    end

    if (nargin > 1)
        if size(J, 2) == 3
            J = J';
        end
        
        R = [R J];
        R = [R; [0 0 0 1]];
    end
end

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