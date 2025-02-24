%% Fit SMPL 2 SCAN, Optimize beta & theta
clear;
close all;
addpath('./npy_matlab');
delete('./*.png');

%% Arguments Setting
maxIter = 30;
GMPenalty = 0.01;
ICPMachingRatio = 0.7;
ICPRatioIncrement = 0.05;
ICPRatioMax = 0.9;

load('PoseStatics2.mat');
load('SMPL_M.mat'); % Male Core
load('m_model.mat'); % male template
% load('w_model.mat'); % female template
%load('target.mat'); % target template

% obj = readObj('SH01.obj');
% points = obj.v;
% tl = obj.f.v;
% 
% obj = readObj('JS03.obj');
% points = obj.v;
% tl = obj.f.v;
%
load('TR_mesh.mat');

% load('lm_idx.mat'); % lm_tree(parent, child), s_lm, t_lm, 1~7 : similarity fitting, 8~24 : pose landmark
% load('lm_idx_sh.mat'); % lm_tree(parent, child), s_lm, t_lm, 1~7 : similarity fitting, 8~24 : pose landmark
% load('lm_idx_js.mat'); % lm_tree(parent, child), s_lm, t_lm, 1~7 : similarity fitting, 8~24 : pose landmark
load('lm_idx_tr.mat'); % lm_tree(parent, child), s_lm, t_lm, 1~7 : similarity fitting, 8~24 : pose landmark

beta = zeros(10, 1);	% Shape param (10)
theta = zeros(24, 3);	% Pose param (1 Global + 23 Joints x 3)

v = SMPLModel(C, beta, theta);

opts = optimoptions(@lsqnonlin, 'Display', 'off');
%opts = optimoptions(@lsqnonlin, 'Display', 'iter');

% Display Initial Src Model & Tar Model
h = figure('visible','off');
% h = figure;
dispFace2(v, f, points, tl);
saveas(h, '1.png');
close(h);


%% Similarity Fitting (Initial pose matching)
[Tm, Ts, TR, Tt] = similarity_fitting(points, v, t_lm(1:7), s_lm(1:7)); % Tm : position normalize, Ts : scale. TR : Rotation matrix, Tt : translation vector
points = (TR * (points - Tm)' * Ts)' + Tt;

% Display Initlal Pose Matched Initial Src Model & Tar Model 
h = figure('visible','off');
% h = figure;
dispFace2(v, f, points, tl);
saveas(h, '2.png');
close(h)


%% Initially Pose Optimizing
v_shaped = squeeze(sum(permute(C.shapeDirs, [3 1 2]) .* beta)) + C.meanVerts;
J = C.regJoint * v_shaped; % Joint Regressor  
v_shaped_sampled = v_shaped(s_lm(8:22), :); % Shape PCA
Wreg = 0.05;
[theta, ~, ~, ~, ~] = lsqnonlin(@(theta)EfuncOptimThetaLand(C, v_shaped_sampled, J, theta, s_lm(8:22), points(t_lm(8:22), :), lm_tree, Wreg, PoseStatics), theta, [], [], opts);

v = SMPLModel(C, beta, theta);

% Display Pose Matched Src Model & Tar Model 
h = figure('visible','off');
% h = figure;
dispFace2(v, f, points, tl);
saveas(h, '3.png');
close(h)


%% Sampling 
nOfSrcSampling = 0.1;
nOfTarSampling = 0.05;

% ratio
if (nOfSrcSampling <= 1.0)
    nOfSrcSampling = round(length(C.meanVerts) * nOfSrcSampling);
end
if (nOfTarSampling <= 1.0)
    nOfTarSampling = round(length(points) * nOfTarSampling );
end

for iter = 1:maxIter
    % randomly extract
    rndSrcIdx =  randsample(length(C.meanVerts), nOfSrcSampling);
    rndTarIdx = randsample(length(points), nOfTarSampling);
    srcVertices = v(rndSrcIdx, :);
    tarVertices = points(rndTarIdx, :);
    
    fprintf ('Iter %03d\n', iter);
    
    %% Rigid Transform
    fprintf ('  Rigid Transformation\n');
    % Extract Normal
    Src = pointCloud(srcVertices);
    Tar = pointCloud(tarVertices);
    Src.Normal = pcnormals(Src);

    [Tform, ~] = pcregrigid(Src, Tar, 'Extrapolate', true, 'InlierRatio', ICPMachingRatio, 'Metric', 'pointToPlane', 'MaxIterations', 50);
    ptPoints = pctransform(pointCloud(points), invert(Tform)); % Tar to Src, Tform : Src -> Tar
    points = ptPoints.Location;
    pointsNormal = pcnormals(ptPoints);
    tarVertices = points(rndTarIdx, :);

    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(v, f, points, tl);
    saveas(h, sprintf('4_%03d_1.png', iter));
    close(h)
    

    %% Optimize Beta
    % Make Block
    minGrid = min(min(points), min(v));
    maxGrid = max(max(points), max(v));

    nGrid = ceil((maxGrid - minGrid) ./ 0.5); % point refered setting
    xGrid = nGrid(1); yGrid = nGrid(2); zGrid = nGrid(3);

    gridStep = (maxGrid - minGrid) ./ [xGrid yGrid zGrid];

    cenGrid = ndgrid(minGrid(1)+gridStep(1)/2:gridStep(1):maxGrid(1), minGrid(2)+gridStep(2)/2:gridStep(2):maxGrid(2), minGrid(3)+gridStep(3)/2:gridStep(3):maxGrid(3));
    gridGroup = cell(xGrid, yGrid, zGrid);

    gridOverlap = 1.3;
    sN = nOfSrcSampling;
    sV = srcVertices;
    tV = tarVertices;

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
                    gridGroup{x, y, z} = 1:sN;
                    gridGroup{x, y, z} = gridGroup{x, y, z}(all(abs(sV(gridGroup{x, y, z}, :) - [cenX cenY cenZ]) <= (gridStep/2) * cStep, 2));
                end
            end
        end
    end

    % Optimizing
    VselGrid = max(min(floor((tV - minGrid) ./ gridStep) +1, [xGrid, yGrid, zGrid]), [1, 1, 1]);

    Wreg = 0.05;
    %Wreg = 0;
    Wland = 0;

    fprintf ('  Optimizing Beta\n');
    [beta, ~, ~, ~, ~] = lsqnonlin(@(beta)EfuncOptimBeta(C, beta, theta, gridGroup, VselGrid, rndSrcIdx, s_lm(8:22), tarVertices, points(t_lm(8:22), :), Wland, Wreg, GMPenalty), beta, [], [], opts);

    v = SMPLModel(C, beta, theta);

    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(v, f, points, tl);
    saveas(h, sprintf('4_%03d_2.png', iter));
    close(h)
    
    if(iter == 1)
        continue;
    end
    
    if (iter >= 10)
       %% Rigid Transform
        fprintf ('  Rigid Transformation\n');
        % Extract Normal
        srcVertices = v(rndSrcIdx, :);
        Src = pointCloud(srcVertices);
        Tar = pointCloud(tarVertices);
        Src.Normal = pcnormals(Src);

        [Tform, ~] = pcregrigid(Src, Tar, 'Extrapolate', true, 'InlierRatio', ICPMachingRatio, 'Metric', 'pointToPlane', 'MaxIterations', 50);
        ptPoints = pctransform(pointCloud(points), invert(Tform)); % Tar to Src, Tform : Src -> Tar
        points = ptPoints.Location;
        pointsNormal = pcnormals(ptPoints);
        tarVertices = points(rndTarIdx, :);

        % Display Rigid Transform Matched Src Model & Tar Model 
        h = figure('visible','off');
        %h = figure;
        dispFace2(v, f, points, tl);
        saveas(h, sprintf('4_%03d_3.png', iter));
        close(h)


        Wreg = 0.05;
        Wland = 0;
        %Wland = 0.001;
        v_shaped = squeeze(sum(permute(C.shapeDirs, [3 1 2]) .* beta)) + C.meanVerts;
        J = C.regJoint * v_shaped; % Joint Regressor

        fprintf ('  Optimizing Theta\n');
        %if (mod(iter, 2) == 1)
            [theta, ~, ~, ~, ~] = lsqnonlin(@(theta)EfuncOptimTheta(C, theta, v_shaped, J, gridGroup, VselGrid, rndSrcIdx, s_lm(8:22), tarVertices, points(t_lm(8:22), :), Wland, Wreg, PoseStatics, GMPenalty), theta, [], [], opts);
        %else
        %    [theta, ~, ~, ~, ~] = lsqnonlin(@(theta)EfuncOptimThetaRev(C, theta, v_shaped, J, gridGroup, VselGrid, rndSrcIdx, s_lm(8:22), tarVertices, points(t_lm(8:22), :), Wland, Wreg, PoseStatics, GMPenalty), theta, [], [], opts);
        %end
        
        %[theta, ~, ~, ~, ~] = lsqnonlin(@(theta)EfuncOptimTheta(C, theta, v_shaped, J, gridGroup, VselGrid, rndSrcIdx, s_lm(8:22), tarVertices, points(t_lm(8:22), :), Wland, Wreg, maxJointAnglePenalty, 1.0), theta, [], [], opts);

        v = SMPLModel(C, beta, theta);

        % Display Rigid Transform Matched Src Model & Tar Model 
        h = figure('visible','off');
        %h = figure;
        dispFace2(v, f, points, tl);
        saveas(h, sprintf('4_%03d_4.png', iter));
        close(h);

        ICPMachingRatio = ICPMachingRatio + ICPRatioIncrement;
        ICPMachingRatio = min (ICPRatioMax, ICPMachingRatio);
    end
end

h1 = figure; dispFace2(v, f, points, tl);
saveas(h1, '5_1.png');
h2 = figure; dispFace(points, tl, [.8 .8 .8]);
saveas(h2, '5_2.png');
h3 = figure; dispFace(v, f, [.8 .8 .8]);
saveas(h3, '5_3.png');

save temp_mesh v f beta theta;
save scan_mesh points tl;
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Function part

function F = EfuncOptimThetaLand(Core, v_shaped_sampled, J, theta, srcLandIdx, tarLandVert, lm_tree, Wreg, PoseStatics)
    A = genPosedShape(Core, v_shaped_sampled, J, theta, srcLandIdx);
    B = tarLandVert;
    
    E = zeros(length(lm_tree)-1,3);
    
    for i=1:size(E,1)
        E(i, :) = GetRotErr(A(lm_tree(1,i+1), :)-A(lm_tree(2,i+1), :), B(lm_tree(1,i+1), :)-B(lm_tree(2,i+1), :));
    end
    
    R = reshape(theta(2:end, :), [1 69]) - PoseStatics.Mu;
    R = sqrt(abs(R * PoseStatics.ICov * R'))./ sqrt(69);
    F = [E(:) ./ sqrt(length(E)); sqrt(Wreg) .* R(:)];
end

function F = EfuncOptimBeta(Core, beta, theta, gridGroup, VselGrid, srcICPIdx, srcLandIdx, tarVerts, tarLandVert, Wland, Wreg, GMPenalty)
    %% Shape PCA
    v_shaped = squeeze(sum(permute(Core.shapeDirs, [3 1 2]) .* beta)) + Core.meanVerts;
    J = Core.regJoint * v_shaped;   

    % ICP Term
    VsrcICP = genPosedShape(Core, v_shaped(srcICPIdx, :), J, theta, srcICPIdx); % Vsrc_Trans
    
    nSrc = length(tarVerts);
    
	E_ICP = zeros(nSrc, 3);
    for i = 1:nSrc
        Vgroup = VsrcICP(gridGroup{VselGrid(i, 1), VselGrid(i, 2), VselGrid(i, 3)}, :);
		[~, mIdx] = min(sum((Vgroup - tarVerts(i, :)).^2, 2));
        E_ICP(i, :) =  Vgroup(mIdx, :) - tarVerts(i, :);
    end
    
    E_ICP = (E_ICP ./ sqrt(sum(E_ICP.^2, 2) + (GMPenalty^2))) ./ sqrt(nSrc);

    % Land Term
    E_land = genPosedShape(Core, v_shaped(srcLandIdx, :), J, theta, srcLandIdx) - tarLandVert;
    E_land = E_land ./ sqrt(length(E_land));

    % Regularization Term
    R = beta ./ sqrt(length(beta));
    F = [E_ICP(:); sqrt(Wland) .* E_land(:); sqrt(Wreg) .* R(:)];
end

function F = EfuncOptimTheta(Core, theta, v_shaped, J, gridGroup, VselGrid, srcICPIdx, srcLandIdx, tarVerts, tarLandVert, Wland, Wreg, PoseStatics, GMPenalty)
    % ICP Term
    VsrcICP = genPosedShape(Core, v_shaped(srcICPIdx, :), J, theta, srcICPIdx); % Vsrc_Trans
    
    nSrc = length(tarVerts);
    
	E_ICP = zeros(nSrc, 3);
    for i = 1:nSrc
        Vgroup = VsrcICP(gridGroup{VselGrid(i, 1), VselGrid(i, 2), VselGrid(i, 3)}, :);
		[~, mIdx] = min(sum((Vgroup - tarVerts(i, :)).^2, 2));
        E_ICP(i, :) =  Vgroup(mIdx, :) - tarVerts(i, :);
    end
    
    E_ICP = (E_ICP ./ sqrt(sum(E_ICP.^2, 2) + (GMPenalty^2))) ./ sqrt(nSrc);

    % Land Term
    E_land = genPosedShape(Core, v_shaped(srcLandIdx, :), J, theta, srcLandIdx) - tarLandVert;
    E_land = E_land ./ sqrt(length(E_land));

    % Regularization Term
    R = reshape(theta(2:end, :), [1 69]) - PoseStatics.Mu;
    R = sqrt(abs(R * PoseStatics.ICov * R')) ./ sqrt(69);
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