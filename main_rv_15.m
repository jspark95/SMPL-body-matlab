clear;
close all;
delete('./*.png');

%% Arguments Setting
maxIter = 2;
GMPenalty = 0.01;
ICPMachingRatio = 0.7;
ICPRatioIncrement = 0.05;


%% Load model
% load scan mesh
load('MCS_scan.mat')
% load scan mask
load('shirtsIdx')
% load('shirtsPoints.mat')
% obj = readObj('SH_shirts.obj');
% maskPoints = obj.v;
% maskFace = obj.f.v;
% load('objTrans.mat')
% maskPoints = (TR * (maskPoints - Tm)' * Ts)' + Tt;
% maskIdx = [];
% eps = 1e-5;
% for i = 1:length(maskPoints)
%     tmp = find(points(:,1) < (maskPoints(i,1)+eps) & points(:,1) > (maskPoints(i,1)-eps) & points(:,2) < (maskPoints(i,2)+eps) & points(:,2) > (maskPoints(i,2)-eps) & points(:,3) < (maskPoints(i,3)+eps) & points(:,3) > (maskPoints(i,3)-eps));
%     maskIdx=[maskIdx;tmp];
% end
% [Tm, Ts, TR, Tt] = similarity_fitting(obj.v, points, [1:14966], [1:14966]);
% obj.v = (TR * (obj.v - Tm)' * Ts)' + Tt;
% save objTrans Tm Ts TR Tt;
% load SMPL mesh
load('PoseStatics2.mat');
load('SMPL_M.mat'); % Male Core
load('MCS_SMPL.mat');
% load('m_model.mat'); % male template
% load('w_model.mat'); % female template

% load template mesh
load('MCS_template.mat')

%% Optimization option
% opts = optimoptions(@lsqnonlin, 'Display', 'off');
opts = optimoptions(@lsqnonlin, 'Display', 'iter');
optsg = optimoptions(@lsqnonlin, 'Display', 'iter', 'SpecifyObjectiveGradient', true, 'MaxIterations', 5);

% Display Initial SMPL, Scan, Template
h = figure('visible','off');
%h = figure;
dispFace3(SMPL_v, FIdx, template_v, FIdx, points, tl);
saveas(h, '1.png');
close(h);

%% Matching Labels
%% template_v
%% points
%% maskIdx
% Scan
nPoints = length(points);
nTemplate = length(template_v);
matchingIdx = zeros(nTemplate, 1);
for i = 1:nTemplate
    [~, matchingIdx(i)] = min(sum((template_v(i, :) - points).^2, 2));
end
maskPoints = zeros(nPoints, 1, 'logical');
maskPoints(maskIdx) = 1;
maskTemplate = maskPoints(matchingIdx);
maskTemplateIdx = find(maskTemplate == 1);

% 가져오기
MCPoints = points(maskIdx,:);
MCTemplate_v = template_v(maskTemplateIdx,:);

% Template Mask Face & Ring
nTemplateIdx = length(maskTemplateIdx);
MCFIdx = zeros(length(FIdx), 3);
for i = 1:nTemplateIdx
    idx = find(FIdx == maskTemplateIdx(i));
    MCFIdx(idx) = i;
end
MCF1 = MCFIdx(:, 1); MCF2 = MCFIdx(:, 2); MCF3 = MCFIdx(:, 3);
MCtempRing = unique([MCF1(MCF1 & ~MCF2); MCF2(MCF2 & ~MCF1); MCF2(MCF2 & ~MCF3); MCF3(MCF3 & ~MCF2); MCF3(MCF3 & ~MCF1); MCF1(MCF1 & ~MCF3)]);
MCFIdx = MCFIdx(all(MCFIdx, 2), :);

MCFIdxs = [MCFIdx(:, 1:2); MCFIdx(:, [2 1]); MCFIdx(:, 2:3); MCFIdx(:, [3 2]); MCFIdx(:, [3 1]); MCFIdx(:, [1 3])];
Rings = zeros(length(MCtempRing), length(MCTemplate_v));
for i = 1:length(MCtempRing)
    Rings(i, MCFIdxs(find (MCFIdxs(:, 1) == MCtempRing(i)), 2)) = 1;
    if (sum(Rings(i, :)) == 0)
        Rings(i, MCtempRing(i)) = 1;
    else
        Rings(i, :) = Rings(i, :);
    end
end

nodeThres = 8;
h = figure('visible','off');
G = graph(Rings(:, MCtempRing));
plot(G);
saveas(h, '1_1_Ring_Graph.png');
close(h);

h = figure('visible','off');
T = minspantree(G, 'Type', 'forest');
saveas(h, '1_2_Ring_Min_Spanned_Tree_Graph.png');
close(h);

bins = conncomp(T);
maxPiece = max(bins);
terminalNodes = (degree(T) == 1);

AllRingPath = cell{maxPeice, 1};
targetPieceMask = zeros(1, maxPiece);

for i = 1:maxPiece
    tarMask = (bins == i);
    if (sum(tarMask) < nodeThres)
        continue;
    end
    
    c(i) = 1;
    terNodeIdx = find(terminalNodes(tarMask) == 1);
    
    maxd = 0;
    maxpath = [];
    for j = 1:(length(terNodeIdx)-1)
        for k = (j+1):length(terNodeIdx)
            [path, d] = shortestpath(T, j, k);
            if (d > maxd)
                maxd = d;
                maxpath = path;
            end
        end
    end
    AllRingPath{i} = maxpath;
end

nRing = sum(targetPieceMask);
RingPath = AllRingPath{targetPieceMask};



% Scan Ring Detection
nScanIdx = length(maskIdx);
MCRingIdx = zeros(length(tl), 3);
for i = 1:nScanIdx
    idx = find(tl == maskIdx(i));
    MCRingIdx(idx) = i;
end
MCRing1 = MCRingIdx(:, 1); MCRing2 = MCRingIdx(:, 2); MCRing3 = MCRingIdx(:, 3);
MCscanRing = unique([MCRing1(MCRing1 & ~MCRing2); MCRing2(MCRing2 & ~MCRing1); MCRing2(MCRing2 & ~MCRing3); MCRing3(MCRing3 & ~MCRing2); MCRing3(MCRing3 & ~MCRing1); MCRing1(MCRing1 & ~MCRing3)]);

% 
% maskFace = zeros(length(FIdx), 1, 'logical');
% for i = 1:length(FIdx)
%     if(isempty(find(maskTemplateIdx(:) == FIdx(i,1))) || isempty(find(maskTemplateIdx(:) == FIdx(i,2))) || isempty(find(maskTemplateIdx(:) == FIdx(i,3))))   
%         continue;
%     else
%         maskFace(i)=1;
%     end
% end
% 
% maskFIdx = FIdx(maskFace,:);
% MCFIdx = zeros(size(maskFIdx));
% 
% for i = 1:length(maskFIdx)
%     tmp1 = find(MCTemplate_v(:,1)==template_v(maskFIdx(i,1),1) & MCTemplate_v(:,2)==template_v(maskFIdx(i,1),2) & MCTemplate_v(:,3)==template_v(maskFIdx(i,1),3));
%     tmp2 = find(MCTemplate_v(:,1)==template_v(maskFIdx(i,2),1) & MCTemplate_v(:,2)==template_v(maskFIdx(i,2),2) & MCTemplate_v(:,3)==template_v(maskFIdx(i,2),3));
%     tmp3 = find(MCTemplate_v(:,1)==template_v(maskFIdx(i,3),1) & MCTemplate_v(:,2)==template_v(maskFIdx(i,3),2) & MCTemplate_v(:,3)==template_v(maskFIdx(i,3),3));
%     MCFIdx(i,:) = [tmp1, tmp2, tmp3];
% end
    
h1 = figure('visible','off');
%h = figure;
dispFaceSep(points, tl, [.8, 0, 0], [0, 0.8, 0], maskIdx);
saveas(h1, sprintf('2_1.png'));
close(h1);

h1 = figure('visible','off');
%h = figure;
dispFaceSep(template_v, FIdx, [.8, 0, 0], [0, 0.8, 0], maskTemplateIdx);
saveas(h1, sprintf('2_2.png'));
close(h1);

% points visualization - face
% template_v visualization - face

%% Sampling 
nOfSrcSampling = 0.3;
nOfTarSampling = 0.3;

% ratio
if (nOfSrcSampling <= 1.0)
    nOfSrcSampling = round(length(maskTemplateIdx) * nOfSrcSampling);
end
if (nOfTarSampling <= 1.0)
    nOfTarSampling = round(length(maskTemplateIdx) * nOfTarSampling );
end

%% Optimization
for iter = 1:maxIter
    % Sampling
    rndSrcIdx =  randsample(length(maskTemplateIdx), nOfSrcSampling);
    rndTarIdx = randsample(length(maskTemplateIdx), nOfTarSampling);
    
    fprintf ('Iter %03d\n', iter);
    
    h1 = figure('visible','off');
    %h = figure;
    dispFaceSep(template_v, FIdx, [.8, 0, 0], [0, 0.8, 0], maskTemplateIdx);
    saveas(h1, sprintf('3_%03d_0.png', iter));
    close(h1);
    
    
    %% Rigid Transform 1
    fprintf ('  Rigid Transformation\n');
    % Extract Normal
    Src = pointCloud(MCTemplate_v);
    Tar = pointCloud(MCPoints);
    Src.Normal = pcnormals(Src);

    [Tform, ~] = pcregrigid(Src, Tar, 'Extrapolate', true, 'InlierRatio', ICPMachingRatio, 'Metric', 'pointToPlane', 'MaxIterations', 50);
    
    ptPoints = pctransform(pointCloud(points), invert(Tform)); % Tar to Src, Tform : Src -> Tar
    points = ptPoints.Location;

    MCPoints = points(maskIdx,:);

    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(SMPL_v, FIdx, template_v, FIdx);
    saveas(h, sprintf('3_%03d_1.png', iter));
    close(h);
    
    %% Grid Setting
    minGrid = min(min(MCPoints), min(MCTemplate_v));
    maxGrid = max(max(MCPoints), max(MCTemplate_v));

    nGrid = ceil((maxGrid - minGrid) ./ 0.3); % point refered setting
    xGrid = nGrid(1); yGrid = nGrid(2); zGrid = nGrid(3);

    gridStep = (maxGrid - minGrid) ./ [xGrid yGrid zGrid];

    gridGroup = cell(xGrid, yGrid, zGrid);

    gridOverlap = 1.3;
    sN = length(MCTemplate_v);
    sV = MCTemplate_v;
    tV = MCPoints;
    
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

    VselGrid = max(min(floor((tV - minGrid) ./ gridStep) +1, [xGrid, yGrid, zGrid]), [1, 1, 1]);    
    
    %% Optimizing
    % Data term
    fprintf ('  Optimizing Template\n');
    MCSMPL_v = SMPL_v(maskTemplateIdx, :);
    
    % Z = adjancy matrix
    % H = neighborhood matrix
    nMCTemplate_v = length(MCTemplate_v);
    Z = zeros(nMCTemplate_v, nMCTemplate_v);
    for i=1:length(MCFIdx)
        Z(MCFIdx(i, 1), MCFIdx(i, 2)) = 1; Z(MCFIdx(i, 2), MCFIdx(i, 1)) = 1;
        Z(MCFIdx(i, 2), MCFIdx(i, 3)) = 1; Z(MCFIdx(i, 3), MCFIdx(i, 2)) = 1;    
        Z(MCFIdx(i, 3), MCFIdx(i, 1)) = 1; Z(MCFIdx(i, 1), MCFIdx(i, 3)) = 1;    
    end
    H = sum(Z); 
    exceptIdx = find(H == 0);
    H(exceptIdx) = 1;
    
    % ((Z * MCTemplate_v) ./ H') - MCTemplate_v
    % (I - Z./ H') * MCTemplate_v) 
    
    Wedge = 0.5;
    Wlap = 1;
    Wbound = 1;
    Wsmooth = 100;
    [MCTemplate_v, ~, ~, ~, ~] = lsqnonlin(@(MCTemplate_v)EfuncOptimTemp(MCTemplate_v, MCPoints, MCSMPL_v, MCFIdx, MCtempRing, MCscanRing, Z, H, Rings, gridGroup, VselGrid, Wedge, Wlap, Wbound, Wsmooth), MCTemplate_v, [], [], optsg);
    %     % 갱신
    template_v(maskTemplate, :) = MCTemplate_v;
    
    h = figure('visible','off');
    %h = figure;
    dispFace2(template_v, FIdx, points, tl);
    saveas(h, sprintf('3_%03d_2.png', iter));
    close(h);    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Coupling term
    
    %% Rigid Transform 2
    fprintf ('  Rigid Transformation\n');
    % Extract Normal
    Src = pointCloud(MCTemplate_v);
    Tar = pointCloud(MCSMPL_v);
    Src.Normal = pcnormals(Src);

    [Tform, MCTemplate_v] = pcregrigid(Src, Tar, 'Extrapolate', true, 'InlierRatio', ICPMachingRatio, 'Metric', 'pointToPlane', 'MaxIterations', 50);
    
    MCTemplate_v = MCTemplate_v.Location;
    ptTemplate = pctransform(pointCloud(template_v), Tform);
    template_v = ptTemplate.Location;

    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(SMPL_v, FIdx, template_v, FIdx);
    saveas(h, sprintf('3_%03d_3.png', iter));
    close(h);
    
    %% Sampling
    srcVertices = MCSMPL_v(rndSrcIdx, :);
    tarVertices = MCTemplate_v(rndTarIdx, :);
    
    %% Optimize Beta
    % Make Block
    minGrid = min(min(MCPoints), min(MCTemplate_v));
    maxGrid = max(max(MCPoints), max(MCTemplate_v));

    nGrid = ceil((maxGrid - minGrid) ./ 0.3); % point refered setting
    xGrid = nGrid(1); yGrid = nGrid(2); zGrid = nGrid(3);

    gridStep = (maxGrid - minGrid) ./ [xGrid yGrid zGrid];

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
    
    fprintf ('  Optimizing Beta\n');
    [beta, ~, ~, ~, ~] = lsqnonlin(@(beta)EfuncOptimBeta(C, beta, theta, gridGroup, VselGrid, rndSrcIdx, tarVertices, Wreg, GMPenalty), beta, [], [], opts);

    % Update
    SMPL_v = SMPLModel(C, beta, theta);
    MCSMPL_v = SMPL_v(maskTemplateIdx, :);

    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(SMPL_v, FIdx, template_v, FIdx);
    saveas(h, sprintf('3_%03d_4.png', iter));
    close(h)
    
    
    %% Rigid Transform 3
    fprintf ('  Rigid Transformation\n');
    % Extract Normal
    Src = pointCloud(MCTemplate_v);
    Tar = pointCloud(MCSMPL_v);
    Src.Normal = pcnormals(Src);

    [Tform, MCTemplate_v] = pcregrigid(Src, Tar, 'Extrapolate', true, 'InlierRatio', ICPMachingRatio, 'Metric', 'pointToPlane', 'MaxIterations', 50);
    MCTemplate_v = MCTemplate_v.Location;
    ptTemplate = pctransform(pointCloud(template_v), Tform);
    template_v = ptTemplate.Location;

    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(SMPL_v, FIdx, template_v, FIdx);
    saveas(h, sprintf('3_%03d_5.png', iter));
    close(h);
    
    %% Sampling
    srcVertices = MCSMPL_v(rndSrcIdx, :);
    tarVertices = MCTemplate_v(rndTarIdx, :);
    

    %% Optimize theta
    Wreg = 0.05;
    v_shaped = squeeze(sum(permute(C.shapeDirs, [3 1 2]) .* beta)) + C.meanVerts;
    Joint = C.regJoint * v_shaped; % Joint Regressor

    fprintf ('  Optimizing Theta\n');
    [theta, ~, ~, ~, ~] = lsqnonlin(@(theta)EfuncOptimTheta(C, theta, v_shaped, Joint, gridGroup, VselGrid, rndSrcIdx, tarVertices, Wreg, PoseStatics, GMPenalty), theta, [], [], opts);
    
    % Update
    SMPL_v = SMPLModel(C, beta, theta);
    MCSMPL_v = SMPL_v(maskTemplateIdx, :);


    % Display Rigid Transform Matched Src Model & Tar Model 
    h = figure('visible','off');
    %h = figure;
    dispFace2(SMPL_v, FIdx, template_v, FIdx);
    saveas(h, sprintf('3_%03d_6.png', iter));
    close(h)

    ICPMachingRatio = ICPMachingRatio + ICPRatioIncrement;
    ICPMachingRatio = min (1.0, ICPMachingRatio);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

h1 = figure('visible','off');
%h = figure;
dispFace(template_v, FIdx, [.8 .8 .8]);
saveas(h1, sprintf('4.png'));
close(h1);

h1 = figure('visible','off');
%h = figure;
dispFaceSep(points, tl, [.8, 0, 0], [0, 0.8, 0], maskIdx);
saveas(h1, sprintf('5_1.png'));
close(h1);

h1 = figure('visible','off');
%h = figure;
dispFaceSep(template_v, FIdx, [.8, 0, 0], [0, 0.8, 0], maskTemplateIdx);
saveas(h1, sprintf('5_2.png'));
close(h1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Function part

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function F = EfuncOptimBeta(Core, beta, theta, gridGroup, VselGrid, srcICPIdx, tarVerts, Wreg, GMPenalty)
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

    % Regularization Term
    R = beta ./ sqrt(length(beta));
    F = [E_ICP(:); sqrt(Wreg) .* R(:)];
end

function F = EfuncOptimTheta(Core, theta, v_shaped, J, gridGroup, VselGrid, srcICPIdx, tarVerts, Wreg, PoseStatics, GMPenalty)
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

    % Regularization Term
    R = reshape(theta(2:end, :), [1 69]) - PoseStatics.Mu;
    R = sqrt(abs(R * PoseStatics.ICov * R')) ./ sqrt(69);
    F = [E_ICP(:); sqrt(Wreg) .* R(:)];
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
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [F, J] = EfuncOptimTemp(MCsrcVerts, MCscanVerts, MCSMPLVerts, MCFIdx, tempRing, scanRing, Z, H, Rings, gridGroup, VselGrid, Wedge, Wlap, Wbound, Wsmooth)
%     scanVerts = scanVerts(maskIdx,:); %
%     nVert = length(scanVerts);
        
    % scanVerts MCPoints;
    % srcVerts = MCTemplate_v;
    nVert = length(MCscanVerts);
    % Scan
    E_ICP = zeros(nVert, 3);
    mIdxs = zeros(nVert, 1);
    for i = 1:nVert
        groupIdx = gridGroup{VselGrid(i, 1), VselGrid(i, 2), VselGrid(i, 3)};
        Vgroup = MCsrcVerts(groupIdx, :);
		[~, mIdx] = min(sum((Vgroup - MCscanVerts(i, :)).^2, 2));
        mIdxs(i) = groupIdx(mIdx);
        E_ICP(i, :) =  Vgroup(mIdx, :) - MCscanVerts(i, :);
    end
    
    E_ICP = E_ICP ./ sqrt(length(E_ICP));
    
    % Model
    nFace = length(MCFIdx);
    E_edge = zeros(nFace * 2, 3);
    E_edge1 = MCFIdx(:, 1); E_edge2 = MCFIdx(:, 2); E_edge3 = MCFIdx(:, 3);
    E_edge(1:nFace, :) = (MCSMPLVerts(E_edge1, :) - MCSMPLVerts(E_edge2, :)) - (MCsrcVerts(E_edge1, :) - MCsrcVerts(E_edge2, :));    
    E_edge((nFace+1):end, :) = (MCSMPLVerts(E_edge2, :) - MCSMPLVerts(E_edge3, :)) - (MCsrcVerts(E_edge2, :) - MCsrcVerts(E_edge3, :));
    E_edge = E_edge ./ sqrt(length(E_edge));
    
    % Laplacian
    % (I - Z./ H') * MCTemplate_v) 
    E_lap = ((Z * MCsrcVerts) ./ H') - MCsrcVerts;
    E_lap = E_lap ./ sqrt(length(E_lap));
    
    % Ring
    scanRingVert = MCscanVerts(scanRing,:);
    nTempRingVert = length(tempRing);
    
    E_Ringbound = zeros(nTempRingVert, 3);
    for i = 1:nTempRingVert
		[~, mIdx] = min(sum((scanRingVert - MCsrcVerts(tempRing(i), :)).^2, 2));
        E_Ringbound(i, :) =  scanRingVert(mIdx, :) - MCsrcVerts(tempRing(i), :);
    end
    
    E_Ringbound = E_Ringbound ./ sqrt(length(E_Ringbound));
    
    % Ring smooth
    E_Ringsmooth = (Rings * MCsrcVerts) - MCsrcVerts(tempRing, :);
    E_Ringsmooth = E_Ringsmooth ./ sqrt(length(E_Ringsmooth));
    
    F = [E_ICP(:); sqrt(Wedge)*E_edge(:); sqrt(Wlap)*E_lap(:); sqrt(Wbound)*E_Ringbound(:); sqrt(Wsmooth)*E_Ringsmooth(:)];
    
    if nargout > 1
        % Scan
        J_ICP = zeros(nVert * 3, length(MCsrcVerts(:))); % OUT x IN
        for i = 1:nVert
            % X
            J_ICP(i, mIdxs(i)) = 1;
            % Y
            J_ICP(i+nVert, mIdxs(i)+length(MCsrcVerts)) = 1;
            % Z
            J_ICP(i+2*nVert, mIdxs(i)+2*length(MCsrcVerts)) = 1;
        end
        
        J_ICP = J_ICP ./ sqrt(length(E_ICP));
        
        % Model
        J_Edge = zeros(nFace * 6, length(MCsrcVerts(:))); % OUT x IN
        for i = 1:length(nFace)
            J_Edge(i, E_edge2(i)) = 1;
            J_Edge(i, E_edge1(i)) = -1;
            J_Edge(nFace*2+i, E_edge2(i)+length(MCsrcVerts)) = 1;
            J_Edge(nFace*2+i, E_edge1(i)+length(MCsrcVerts)) = -1;
            J_Edge(nFace*4+i, E_edge2(i)+2*length(MCsrcVerts)) = 1;
            J_Edge(nFace*4+i, E_edge1(i)+2*length(MCsrcVerts)) = -1;

            J_Edge(nFace+i, E_edge3(i)) = 1;
            J_Edge(nFace+i, E_edge2(i)) = -1;
            J_Edge(nFace*3+i, E_edge3(i)+length(MCsrcVerts)) = 1;
            J_Edge(nFace*3+i, E_edge2(i)+length(MCsrcVerts)) = -1;
            J_Edge(nFace*5+i, E_edge3(i)+2*length(MCsrcVerts)) = 1;
            J_Edge(nFace*5+i, E_edge2(i)+2*length(MCsrcVerts)) = -1;
        end
        J_Edge = J_Edge ./ sqrt(length(E_edge));
        
        % Laplacian term
        J_Lap = -1 * eye(numel(MCsrcVerts));
        J_Lap = J_Lap ./ sqrt(length(E_lap));
        
        % Boundary term
        J_RingBound = zeros(nTempRingVert * 3, numel(MCsrcVerts)); % OUT x IN
        for i = 1:nTempRingVert
            % X
            J_RingBound(i, tempRing(i)) = -1;
            % Y
            J_RingBound(nTempRingVert+i, tempRing(i)+length(MCsrcVerts)) = -1;
            % Z
            J_RingBound(nTempRingVert*2+i, tempRing(i)+2*length(MCsrcVerts)) = -1;
        end
        
        J_RingBound = J_RingBound ./ sqrt(length(E_Ringbound));
        
        % Laplacian term
        J_RingSmooth = zeros(nTempRingVert*3, numel(MCsrcVerts));
        for i = 1:nTempRingVert
            % X
            J_RingSmooth(i,tempRing(i)) = -1;
            % Y
            J_RingSmooth(nTempRingVert+i,tempRing(i)+length(MCsrcVerts)) = -1;
            % Z
            J_RingSmooth(2*nTempRingVert+i,tempRing(i)+2*length(MCsrcVerts)) = -1;
        end
        J_RingSmooth = J_RingSmooth ./ sqrt(length(E_Ringsmooth));
        
        J = [J_ICP; sqrt(Wedge)*J_Edge; sqrt(Wlap)*J_Lap; sqrt(Wbound)*J_RingBound; sqrt(Wsmooth)*J_RingSmooth];
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

function dispFace3(shp1, tl1, shp2, tl2, shp3, tl3)
    FV1.vertices = shp1;
    FV1.faces = tl1;
    
    FV2.vertices = shp2;
    FV2.faces = tl2;
    
    FV3.vertices = shp3;
    FV3.faces = tl3;
    
    patch(FV1, 'facecolor', [0.8 0.0 0.0], 'edgecolor', 'none', 'vertexnormalsmode', 'auto', 'FaceAlpha', 0.5);
    patch(FV2, 'facecolor', [0.0 0.8 0.0], 'edgecolor', 'none', 'vertexnormalsmode', 'auto', 'FaceAlpha', 0.5);
    patch(FV3, 'facecolor', [0.0 0.0 0.8], 'edgecolor', 'none', 'vertexnormalsmode', 'auto', 'FaceAlpha', 0.5);
    
    camlight('headlight');
    lighting phong;
    material dull;
    axis vis3d
    axis equal;
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